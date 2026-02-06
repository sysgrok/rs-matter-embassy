use core::pin::pin;

use embassy_futures::select::select;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use rs_matter_stack::mdns::BuiltinMdns;

use crate::ble::{ControllerRef, TroubleBtpGattContext, TroubleBtpGattPeripheral};
use crate::enet::{create_enet_stack, EnetNetif, EnetStack};
use crate::eth::EmbassyNetContext;
use crate::matter::crypto::RngCore;
use crate::matter::dm::clusters::gen_diag::InterfaceTypeEnum;
use crate::matter::dm::clusters::net_comm::NetCtl;
use crate::matter::dm::clusters::wifi_diag::{WifiDiag, WirelessDiag};
use crate::matter::dm::networks::wireless::Wifi;
use crate::matter::dm::networks::NetChangeNotif;
use crate::matter::error::Error;
use crate::matter::utils::select::Coalesce;
use crate::stack::network::{Embedding, Network};
use crate::stack::wireless::{self, Gatt, GattTask};

use super::{BleDriver, BleDriverTask, BleDriverTaskImpl, EmbassyWirelessMatterStack};

#[cfg(feature = "esp")]
pub mod esp_wifi;
#[cfg(feature = "rp")]
pub mod rp_wifi;

/// A type alias for an Embassy Matter stack running over Wifi (and BLE, during commissioning).
///
/// The difference between this and the `WifiMatterStack` is that all resources necessary for the
/// operation of `embassy-net` as well as the BLE controller and pre-allocated inside the stack.
pub type EmbassyWifiMatterStack<'a, const B: usize, E = ()> =
    EmbassyWirelessMatterStack<'a, B, Wifi, EmbassyNetContext, E>;

/// A trait representing a task that needs access to the Wifi driver and controller to perform its work
pub trait WifiDriverTask {
    /// Run the task with the given Wifi driver and controller
    async fn run<D, C>(&mut self, driver: D, net_ctl: C) -> Result<(), Error>
    where
        D: embassy_net::driver::Driver,
        C: NetCtl + NetChangeNotif + WirelessDiag + WifiDiag;
}

impl<T> WifiDriverTask for &mut T
where
    T: WifiDriverTask,
{
    /// Run the task with the given Wifi driver and Wifi controller
    async fn run<D, C>(&mut self, driver: D, net_ctl: C) -> Result<(), Error>
    where
        D: embassy_net::driver::Driver,
        C: NetCtl + NetChangeNotif + WirelessDiag + WifiDiag,
    {
        (*self).run(driver, net_ctl).await
    }
}

/// A trait representing a task that needs access to the Wifi driver and controller,
/// as well as to the BLe controller to perform its work
pub trait WifiCoexDriverTask {
    /// Run the task with the given Wifi driver, Wifi controller and BLE controller
    async fn run<D, C, B>(&mut self, wifi_driver: D, net_ctl: C, ble_ctl: B) -> Result<(), Error>
    where
        D: embassy_net::driver::Driver,
        C: NetCtl + NetChangeNotif + WirelessDiag + WifiDiag,
        B: trouble_host::Controller;
}

impl<T> WifiCoexDriverTask for &mut T
where
    T: WifiCoexDriverTask,
{
    async fn run<D, C, B>(&mut self, wifi_driver: D, net_ctl: C, ble_ctl: B) -> Result<(), Error>
    where
        D: embassy_net::driver::Driver,
        C: NetCtl + NetChangeNotif + WirelessDiag + WifiDiag,
        B: trouble_host::Controller,
    {
        (*self).run(wifi_driver, net_ctl, ble_ctl).await
    }
}

/// A trait for running a task within a context where the Wifi radio is initialized and operable
pub trait WifiDriver {
    /// Setup the Wifi driver and controller and run the given task with these
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: WifiDriverTask;
}

impl<T> WifiDriver for &mut T
where
    T: WifiDriver,
{
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: WifiDriverTask,
    {
        (*self).run(task).await
    }
}

/// A trait for running a task within a context where the Wifi radio - as well as the BLE controller - are initialized and operable
pub trait WifiCoexDriver {
    /// Setup the Wifi driver and controller, as well as the BLE controller run the given task with these
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: WifiCoexDriverTask;
}

impl<T> WifiCoexDriver for &mut T
where
    T: WifiCoexDriver,
{
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: WifiCoexDriverTask,
    {
        (*self).run(task).await
    }
}

/// A Wifi driver provider that uses a pre-existing, already created Wifi driver and controller,
/// rather than creating them when the Matter stack needs them.
pub struct PreexistingWifiDriver<D, C, B>(D, C, B);

impl<D, C, B> PreexistingWifiDriver<D, C, B> {
    /// Create a new instance of the `PreexistingWifiDriver` type.
    pub const fn new(enet_wifi_driver: D, net_ctl: C, ble_ctl: B) -> Self {
        Self(enet_wifi_driver, net_ctl, ble_ctl)
    }
}

impl<D, C, B> WifiDriver for PreexistingWifiDriver<D, C, B>
where
    D: embassy_net::driver::Driver,
    C: NetCtl + NetChangeNotif + WirelessDiag + WifiDiag,
{
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: WifiDriverTask,
    {
        task.run(&mut self.0, &self.1).await
    }
}

impl<D, C, B> BleDriver for PreexistingWifiDriver<D, C, B>
where
    B: trouble_host::Controller,
{
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: BleDriverTask,
    {
        task.run(ControllerRef::new(&self.2)).await
    }
}

impl<D, C, B> WifiCoexDriver for PreexistingWifiDriver<D, C, B>
where
    D: embassy_net::driver::Driver,
    C: NetCtl + NetChangeNotif + WirelessDiag + WifiDiag,
    B: trouble_host::Controller,
{
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: WifiCoexDriverTask,
    {
        task.run(&mut self.0, &self.1, ControllerRef::new(&self.2))
            .await
    }
}

/// A `Wireless` trait implementation for `embassy-net`'s Wifi stack.
pub struct EmbassyWifi<'a, T, R> {
    driver: T,
    rand: R,
    use_ble_random_addr: bool,
    context: &'a EmbassyNetContext,
    ble_context: &'a TroubleBtpGattContext<CriticalSectionRawMutex>,
}

impl<'a, T, R> EmbassyWifi<'a, T, R> {
    /// Create a new instance of the `EmbassyWifi` type.
    pub fn new<const B: usize, E>(
        driver: T,
        rand: R,
        use_ble_random_addr: bool,
        stack: &'a EmbassyWifiMatterStack<'a, B, E>,
    ) -> Self
    where
        E: Embedding + 'static,
    {
        Self::wrap(
            driver,
            rand,
            use_ble_random_addr,
            stack.network().embedding().net_context(),
            stack.network().embedding().ble_context(),
        )
    }

    /// Wrap the `EmbassyWifi` type around a Wifi Driver and BLE controller runner and a network context.
    pub const fn wrap(
        driver: T,
        rand: R,
        use_ble_random_addr: bool,
        context: &'a EmbassyNetContext,
        ble_context: &'a TroubleBtpGattContext<CriticalSectionRawMutex>,
    ) -> Self {
        Self {
            driver,
            rand,
            use_ble_random_addr,
            context,
            ble_context,
        }
    }
}

impl<T, R> wireless::Wifi for EmbassyWifi<'_, T, R>
where
    T: WifiDriver,
    R: RngCore + Copy,
{
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: wireless::WifiTask,
    {
        self.driver
            .run(WifiDriverTaskImpl {
                context: self.context,
                rand: self.rand,
                task,
            })
            .await
    }
}

impl<T, R> wireless::WifiCoex for EmbassyWifi<'_, T, R>
where
    T: WifiCoexDriver,
    R: RngCore + Copy,
{
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: wireless::WifiCoexTask,
    {
        self.driver
            .run(WifiCoexDriverTaskImpl {
                context: self.context,
                ble_context: self.ble_context,
                rand: self.rand,
                use_ble_random_addr: self.use_ble_random_addr,
                task,
            })
            .await
    }
}

impl<T, R> Gatt for EmbassyWifi<'_, T, R>
where
    T: BleDriver,
    R: RngCore + Copy,
{
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: GattTask,
    {
        self.driver
            .run(BleDriverTaskImpl {
                task,
                rand: self.use_ble_random_addr.then_some(self.rand),
                context: self.ble_context,
            })
            .await
    }
}

struct WifiDriverTaskImpl<'a, A, R> {
    context: &'a EmbassyNetContext,
    task: A,
    rand: R,
}

impl<A, R> WifiDriverTask for WifiDriverTaskImpl<'_, A, R>
where
    A: wireless::WifiTask,
    R: RngCore,
{
    async fn run<D, C>(&mut self, driver: D, net_ctl: C) -> Result<(), Error>
    where
        D: embassy_net::driver::Driver,
        C: NetCtl + NetChangeNotif + WirelessDiag + WifiDiag,
    {
        let mut resources = self.context.resources.lock().await;
        let resources = &mut *resources;
        let buffers = &self.context.buffers;

        let mut seed = [0; core::mem::size_of::<u64>()];
        self.rand.fill_bytes(&mut seed);

        let (stack, mut runner) = create_enet_stack(driver, u64::from_le_bytes(seed), resources);

        let net_stack = EnetStack::new(stack, buffers);
        let netif = EnetNetif::new(stack, InterfaceTypeEnum::WiFi);

        let mut main = pin!(self.task.run(&net_stack, &netif, &net_ctl, BuiltinMdns));
        let mut run = pin!(async {
            runner.run().await;
            #[allow(unreachable_code)]
            Ok(())
        });

        select(&mut main, &mut run).coalesce().await
    }
}

struct WifiCoexDriverTaskImpl<'a, A, R> {
    context: &'a EmbassyNetContext,
    ble_context: &'a TroubleBtpGattContext<CriticalSectionRawMutex>,
    task: A,
    rand: R,
    use_ble_random_addr: bool,
}

impl<A, R> WifiCoexDriverTask for WifiCoexDriverTaskImpl<'_, A, R>
where
    A: wireless::WifiCoexTask,
    R: RngCore + Copy,
{
    async fn run<D, C, B>(&mut self, wifi_driver: D, net_ctl: C, ble_ctl: B) -> Result<(), Error>
    where
        D: embassy_net::driver::Driver,
        C: NetCtl + NetChangeNotif + WirelessDiag + WifiDiag,
        B: trouble_host::Controller,
    {
        let mut resources = self.context.resources.lock().await;
        let resources = &mut *resources;
        let buffers = &self.context.buffers;

        let mut seed = [0; core::mem::size_of::<u64>()];
        self.rand.fill_bytes(&mut seed);

        let (stack, mut runner) =
            create_enet_stack(wifi_driver, u64::from_le_bytes(seed), resources);

        let net_stack = EnetStack::new(stack, buffers);
        let netif = EnetNetif::new(stack, InterfaceTypeEnum::WiFi);
        let mut peripheral = TroubleBtpGattPeripheral::new(
            ble_ctl,
            self.use_ble_random_addr.then_some(self.rand),
            self.ble_context,
        );

        let mut main =
            pin!(self
                .task
                .run(&net_stack, &netif, &net_ctl, BuiltinMdns, &mut peripheral));
        let mut run = pin!(async {
            runner.run().await;
            #[allow(unreachable_code)]
            Ok(())
        });

        select(&mut main, &mut run).coalesce().await
    }
}

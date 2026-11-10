use core::marker::PhantomData;
use core::pin::pin;

use crate::ble::{BtpGattContext, Controller};
use crate::enet::{create_enet_stack, EnetNetif, EnetStack};
use crate::eth::EmbassyNetContext;
use crate::matter::crypto::Rng;
use crate::matter::dm::clusters::gen_diag::InterfaceTypeEnum;
use crate::matter::dm::clusters::net_comm::NetCtl;
use crate::matter::dm::clusters::wifi_diag::{WifiDiag, WirelessDiag};
use crate::matter::dm::networks::wireless::WifiNetworks;
use crate::matter::dm::networks::NetChangeNotif;
use crate::matter::error::Error;
use crate::matter::utils::futures::{select, Coalesce};
use crate::stack::ble::GattPeripheral;
use crate::stack::network::{Embedding, StackNetwork, WifiStackNetwork};
use crate::stack::wireless::{
    GattDriver, GattTask, WifiCoexDriver, WifiCoexTask, WifiDriver, WifiTask,
};
use crate::stack::MatterStack;

use super::{ControllerGattPeripheral, EmbassyGatt};

#[cfg(feature = "esp")]
pub mod esp_wifi;
#[cfg(feature = "rp")]
pub mod rp_wifi;

/// The network of a `MatterStack` that runs Wifi with [`EmbassyWifi`]: the networks store
/// `S`, and the contexts of `embassy-net` and of the BLE controller, which the stack holds
/// so that they can be statically allocated.
pub struct EmbassyWifiNetwork<S = WifiNetworks>(PhantomData<fn() -> S>);

impl<S> StackNetwork for EmbassyWifiNetwork<S>
where
    S: WifiStackNetwork<Context = ()>,
{
    type Store = S::Store;
    type Context = EmbassyGatt<EmbassyNetContext>;
}

impl<S> WifiStackNetwork for EmbassyWifiNetwork<S> where S: WifiStackNetwork<Context = ()> {}

/// The radio of a pre-existing, already created Wifi driver, network controller and BLE controller,
/// rather than one which creates them when the Matter stack needs them.
pub struct PreexistingWifiRadio<D, C, B>(D, C, B);

impl<D, C, B> PreexistingWifiRadio<D, C, B> {
    /// Create a new instance of the `PreexistingWifiRadio` type.
    pub const fn new(enet_wifi_driver: D, net_ctl: C, ble_ctl: B) -> Self {
        Self(enet_wifi_driver, net_ctl, ble_ctl)
    }
}

impl<D, C, B, R> WifiDriver for EmbassyWifi<'_, PreexistingWifiRadio<D, C, B>, R>
where
    D: embassy_net::driver::Driver,
    C: NetCtl + NetChangeNotif + WirelessDiag + WifiDiag,
    R: Rng + Copy,
{
    // The operational task receives `&self.radio.1` (a `&C`), so the chain net-ctl type
    // is `&'a C`. `&C` satisfies the diag/net-ctl bounds via the blanket impls.
    type NetCtl<'a>
        = &'a C
    where
        Self: 'a;

    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: WifiTask,
    {
        self.runner
            .run(&mut self.radio.0, &self.radio.1, task)
            .await
    }
}

impl<D, C, B, R> WifiCoexDriver for EmbassyWifi<'_, PreexistingWifiRadio<D, C, B>, R>
where
    D: embassy_net::driver::Driver,
    C: NetCtl + NetChangeNotif + WirelessDiag + WifiDiag,
    B: Controller,
    R: Rng + Copy,
{
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: WifiCoexTask,
    {
        let gatt = ControllerGattPeripheral::new(
            &self.radio.2,
            self.runner.ble_rand(),
            self.runner.ble_context(),
        );

        self.runner
            .run_coex(&mut self.radio.0, &self.radio.1, gatt, task)
            .await
    }
}

impl<D, C, B, R> GattDriver for EmbassyWifi<'_, PreexistingWifiRadio<D, C, B>, R>
where
    B: Controller,
    R: Rng + Copy,
{
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: GattTask,
    {
        task.run(ControllerGattPeripheral::new(
            &self.radio.2,
            self.runner.ble_rand(),
            self.runner.ble_context(),
        ))
        .await
    }
}

/// The Wifi drivers of the stack over `embassy-net`, one for each of the radios `T`.
pub struct EmbassyWifi<'a, T, R> {
    radio: T,
    runner: EnetWifiRunner<'a, R>,
}

impl<'a, T, R> EmbassyWifi<'a, T, R> {
    /// Create a new instance of the `EmbassyWifi` type.
    pub fn new<const B: usize, S, E>(
        radio: T,
        rand: R,
        use_ble_random_addr: bool,
        stack: &'a MatterStack<'a, B, S, E>,
    ) -> Self
    where
        S: StackNetwork<Context = EmbassyGatt<EmbassyNetContext>>,
        E: Embedding,
    {
        Self::wrap(
            radio,
            rand,
            use_ble_random_addr,
            stack.context().net_context(),
            stack.context().ble_context(),
        )
    }

    /// Wrap the `EmbassyWifi` type around a Wifi radio and a network context.
    pub const fn wrap(
        radio: T,
        rand: R,
        use_ble_random_addr: bool,
        context: &'a EmbassyNetContext,
        ble_context: &'a BtpGattContext,
    ) -> Self {
        Self {
            radio,
            runner: EnetWifiRunner::new(rand, use_ble_random_addr, context, ble_context),
        }
    }
}

/// What the Wifi drivers have in common, whatever their radio: running a task of
/// the stack over an `embassy-net` driver.
///
/// For implementing the Wifi drivers of the stack for a radio this crate does not know.
pub struct EnetWifiRunner<'a, R> {
    rand: R,
    use_ble_random_addr: bool,
    context: &'a EmbassyNetContext,
    ble_context: &'a BtpGattContext,
}

impl<'a, R> EnetWifiRunner<'a, R> {
    /// Create a new instance of the `EnetWifiRunner` type.
    pub const fn new(
        rand: R,
        use_ble_random_addr: bool,
        context: &'a EmbassyNetContext,
        ble_context: &'a BtpGattContext,
    ) -> Self {
        Self {
            rand,
            use_ble_random_addr,
            context,
            ble_context,
        }
    }
}

impl<'a, R> EnetWifiRunner<'a, R>
where
    R: Rng + Copy,
{
    /// Return the random number generator for the BLE address, if a random one should be used.
    pub fn ble_rand(&self) -> Option<R> {
        self.use_ble_random_addr.then_some(self.rand)
    }

    /// Return the GATT peripheral context.
    pub const fn ble_context(&self) -> &'a BtpGattContext {
        self.ble_context
    }

    /// Run a Wifi task over the given driver and network controller.
    pub async fn run<D, C, A>(&self, driver: D, net_ctl: C, mut task: A) -> Result<(), Error>
    where
        D: embassy_net::driver::Driver,
        C: NetCtl + NetChangeNotif + WirelessDiag + WifiDiag,
        A: WifiTask,
    {
        let mut resources = self.context.resources.lock().await;
        let mut mdns = self.context.mdns.lock().await;

        let resources = &mut *resources;
        let mdns = &mut *mdns;
        let buffers = &self.context.buffers;

        let mut rand = self.rand;
        let mut seed = [0; core::mem::size_of::<u64>()];
        rand.fill_bytes(&mut seed);

        let (stack, mut runner) = create_enet_stack(driver, u64::from_le_bytes(seed), resources);

        let net_stack = EnetStack::new(stack, buffers);
        let netif = EnetNetif::new(stack, InterfaceTypeEnum::WiFi);

        let mut main = pin!(task.run(&net_stack, &netif, &net_ctl, mdns));
        let mut run = pin!(async {
            runner.run().await;
            #[allow(unreachable_code)]
            Ok(())
        });

        select(&mut main, &mut run).coalesce().await
    }

    /// Run a Wifi task with concurrent commissioning over the given driver, network
    /// controller and GATT peripheral.
    pub async fn run_coex<D, C, G, A>(
        &self,
        driver: D,
        net_ctl: C,
        mut gatt: G,
        mut task: A,
    ) -> Result<(), Error>
    where
        D: embassy_net::driver::Driver,
        C: NetCtl + NetChangeNotif + WirelessDiag + WifiDiag,
        G: GattPeripheral,
        A: WifiCoexTask,
    {
        let mut resources = self.context.resources.lock().await;
        let mut mdns = self.context.mdns.lock().await;

        let resources = &mut *resources;
        let mdns = &mut *mdns;
        let buffers = &self.context.buffers;

        let mut rand = self.rand;
        let mut seed = [0; core::mem::size_of::<u64>()];
        rand.fill_bytes(&mut seed);

        let (stack, mut runner) = create_enet_stack(driver, u64::from_le_bytes(seed), resources);

        let net_stack = EnetStack::new(stack, buffers);
        let netif = EnetNetif::new(stack, InterfaceTypeEnum::WiFi);

        let mut main = pin!(task.run(&net_stack, &netif, &net_ctl, mdns, &mut gatt));
        let mut run = pin!(async {
            runner.run().await;
            #[allow(unreachable_code)]
            Ok(())
        });

        select(&mut main, &mut run).coalesce().await
    }
}

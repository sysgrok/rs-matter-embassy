//! Wireless: Type aliases and state structs for an Embassy Matter stack running over a wireless network (Wifi or Thread) and BLE.

use crate::matter::crypto::Rng;
use crate::matter::dm::networks::wireless::WirelessNetwork;
use crate::matter::error::Error;
use crate::matter::utils::init::{init, Init};
use crate::stack::network::{Embedding, Network};
use crate::stack::wireless::{GattTask, WirelessBle};
use crate::stack::MatterStack;

use crate::ble::{BtpGattContext, BtpGattPeripheral, Controller, ControllerRef};
use crate::matter::transport::network::btp::{AdvData, Btp};
use crate::stack::ble::GattPeripheral;

#[cfg(feature = "openthread")]
pub use thread::*;
#[cfg(feature = "embassy-net")]
pub use wifi::*;

// Thread: Type aliases and state structs for an Embassy Matter stack running over a Thread network and BLE.
#[cfg(feature = "openthread")]
mod thread;
// Wifi: Type aliases and state structs for an Embassy Matter stack running over a Wifi network and BLE.
#[cfg(feature = "embassy-net")]
mod wifi;

#[cfg(feature = "esp")]
mod esp_ble;

#[cfg(feature = "esp")]
pub mod esp {
    pub use super::esp_ble::*;
    #[cfg(feature = "openthread")]
    pub use super::thread::esp_thread::*;
    #[cfg(feature = "embassy-net")]
    pub use super::wifi::esp_wifi::*;
}

#[cfg(feature = "rp")]
pub mod rp {
    #[cfg(feature = "embassy-net")]
    pub use super::wifi::rp_wifi::*;
}

/// A type alias for an Embassy Matter stack running over a wireless network (Wifi or Thread) and BLE.
///
/// The difference between this and `WirelessMatterStack` is that all resources necessary for the
/// operation of the BLE controller and pre-allocated inside the stack.
pub type EmbassyWirelessMatterStack<'a, const B: usize, T, N, E = ()> =
    MatterStack<'a, B, EmbassyWirelessBle<T, N, E>>;

/// A type alias for an Embassy implementation of the `Network` trait for a Matter stack running over
/// BLE during commissioning, and then over either WiFi or Thread when operating.
pub type EmbassyWirelessBle<T, N, E = ()> = WirelessBle<T, EmbassyGatt<N, E>>;

#[allow(unused)]
pub(crate) const SLOTS: usize = 20;

/// An embedding of the Trouble Gatt peripheral context for the `WirelessBle` network type from `rs-matter-stack`.
///
/// Allows the memory of this context to be statically allocated and cost-initialized.
///
/// Usage:
/// ```no_run
/// MatterStack<WirelessBle<Wifi, EmbassyGatt<C, E>>>::new(...);
/// ```
///
/// ... where `E` can be a next-level, user-supplied embedding or just `()` if the user does not need to embed anything.
pub struct EmbassyGatt<N, E = ()> {
    btp_gatt_context: BtpGattContext,
    net_context: N,
    embedding: E,
}

impl<N, E> EmbassyGatt<N, E>
where
    N: Embedding,
    E: Embedding,
{
    /// Creates a new instance of the `EspGatt` embedding.
    #[allow(clippy::large_stack_frames)]
    #[inline(always)]
    const fn new() -> Self {
        Self {
            btp_gatt_context: BtpGattContext::new(),
            net_context: N::INIT,
            embedding: E::INIT,
        }
    }

    /// Return an in-place initializer for the `EspGatt` embedding.
    fn init() -> impl Init<Self> {
        init!(Self {
            btp_gatt_context <- BtpGattContext::init(),
            net_context <- N::init(),
            embedding <- E::init(),
        })
    }

    /// Return a reference to the Bluedroid Gatt peripheral context.
    pub fn ble_context(&self) -> &BtpGattContext {
        &self.btp_gatt_context
    }

    pub fn net_context(&self) -> &N {
        &self.net_context
    }

    /// Return a reference to the embedding.
    pub fn embedding(&self) -> &E {
        &self.embedding
    }
}

impl<N, E> Embedding for EmbassyGatt<N, E>
where
    N: Embedding,
    E: Embedding,
{
    const INIT: Self = Self::new();

    fn init() -> impl Init<Self> {
        EmbassyGatt::init()
    }
}

/// A trait representing a task that needs access to the BLE controller to perform its work
pub trait BleDriverTask {
    /// Run the task with the given BLE controller
    async fn run<C>(&mut self, controller: C) -> Result<(), Error>
    where
        C: Controller;
}

impl<T> BleDriverTask for &mut T
where
    T: BleDriverTask,
{
    async fn run<C>(&mut self, controller: C) -> Result<(), Error>
    where
        C: Controller,
    {
        (*self).run(controller).await
    }
}

/// A trait for running a task within a context where the BLE Controller is initialized and operable
/// (e.g. in a commissioning workflow)
pub trait BleDriver {
    /// Setup the BLE controller and run the given task with it
    async fn run<T>(&mut self, task: T) -> Result<(), Error>
    where
        T: BleDriverTask;
}

impl<T> BleDriver for &mut T
where
    T: BleDriver,
{
    async fn run<U>(&mut self, task: U) -> Result<(), Error>
    where
        U: BleDriverTask,
    {
        (*self).run(task).await
    }
}

impl<'a, R, C> BtpGattPeripheral<'a, R, C>
where
    R: Rng + Copy,
    C: Controller,
{
    pub fn new_for_stack<const B: usize, T, E>(
        ble_ctl: C,
        rand: Option<R>,
        stack: &'a crate::wireless::EmbassyWirelessMatterStack<B, T, E>,
    ) -> Self
    where
        T: WirelessNetwork,
        E: Embedding,
    {
        Self::new(ble_ctl, rand, stack.network().embedding().ble_context())
    }
}

#[allow(dead_code)]
struct BleDriverTaskImpl<'a, A, R> {
    task: A,
    rand: Option<R>,
    context: &'a BtpGattContext,
}

impl<A, R> BleDriverTask for BleDriverTaskImpl<'_, A, R>
where
    A: GattTask,
    R: Rng + Copy,
{
    async fn run<C>(&mut self, controller: C) -> Result<(), Error>
    where
        C: Controller,
    {
        let mut peripheral = BtpGattPeripheral::new(controller, self.rand, self.context);

        self.task.run(&mut peripheral).await
    }
}

/// A `BleDriver` over a pre-existing, already created BLE controller: it lends the controller
/// (by reference) to every task it runs, so the controller lives for as long as the driver does.
pub struct PreexistingBleDriver<'a, C>(&'a C);

impl<'a, C> PreexistingBleDriver<'a, C> {
    /// Create a new instance.
    pub const fn new(controller: &'a C) -> Self {
        Self(controller)
    }
}

impl<C> BleDriver for PreexistingBleDriver<'_, C>
where
    C: Controller,
{
    async fn run<T>(&mut self, mut task: T) -> Result<(), Error>
    where
        T: BleDriverTask,
    {
        task.run(ControllerRef::new(self.0)).await
    }
}

/// A `GattPeripheral` that owns a `BleDriver` rather than a BLE controller: the controller is
/// created by the driver only while the peripheral is being run - i.e., with `rs-matter-stack`,
/// only while a commissioning window that has to be advertised over BLE is open - and is dropped
/// (which, for e.g. `esp-radio`, de-initializes the BLE controller) as soon as the run ends.
///
/// This is what makes concurrent commissioning viable for a battery-powered (ICD) device: BLE is
/// up during commissioning only, and the radio is the Thread (or Wifi) one's alone afterwards.
pub struct BleDriverGattPeripheral<'a, B, R> {
    ble: B,
    rand: Option<R>,
    context: &'a BtpGattContext,
}

impl<'a, B, R> BleDriverGattPeripheral<'a, B, R> {
    /// Create a new instance.
    ///
    /// # Arguments
    /// - `ble` - the `BleDriver` creating the BLE controller on demand
    /// - `rand` - a random number generator, if a random BLE address should be used
    /// - `context` - the GATT peripheral context
    pub const fn new(ble: B, rand: Option<R>, context: &'a BtpGattContext) -> Self {
        Self { ble, rand, context }
    }
}

impl<B, R> GattPeripheral for BleDriverGattPeripheral<'_, B, R>
where
    B: BleDriver,
    R: Rng + Copy,
{
    async fn run(
        &mut self,
        btp: &Btp,
        service_name: &str,
        service_adv: &AdvData,
    ) -> Result<(), Error> {
        self.ble
            .run(GattPeripheralRunTask {
                btp,
                service_name,
                service_adv,
                rand: self.rand,
                context: self.context,
            })
            .await
    }
}

/// The `BleDriverTask` behind `BleDriverGattPeripheral`: runs one BTP GATT peripheral on the
/// controller the driver created.
struct GattPeripheralRunTask<'a, R> {
    btp: &'a Btp,
    service_name: &'a str,
    service_adv: &'a AdvData,
    rand: Option<R>,
    context: &'a BtpGattContext,
}

impl<R> BleDriverTask for GattPeripheralRunTask<'_, R>
where
    R: Rng + Copy,
{
    async fn run<C>(&mut self, controller: C) -> Result<(), Error>
    where
        C: Controller,
    {
        let mut peripheral = BtpGattPeripheral::new(controller, self.rand, self.context);

        peripheral
            .run(self.btp, self.service_name, self.service_adv)
            .await
    }
}

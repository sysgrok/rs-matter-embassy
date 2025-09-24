//! Wireless: Type aliases and state structs for an Embassy Matter stack running over a wireless network (Wifi or Thread) and BLE.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use crate::matter::dm::networks::wireless::WirelessNetwork;
use crate::matter::utils::rand::Rand;
use crate::stack::matter::error::Error;
use crate::stack::matter::utils::init::{init, Init};
use crate::stack::network::{Embedding, Network};
use crate::stack::wireless::{GattTask, WirelessBle};
use crate::stack::MatterStack;

use trouble_host::Controller;

use crate::ble::{TroubleBtpGattContext, TroubleBtpGattPeripheral};

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
pub mod esp {
    #[cfg(feature = "openthread")]
    pub use super::thread::esp_thread::*;
    #[cfg(feature = "embassy-net")]
    pub use super::wifi::esp_wifi::*;
}

/// A type alias for an Embassy Matter stack running over a wireless network (Wifi or Thread) and BLE.
///
/// The difference between this and `WirelessMatterStack` is that all resources necessary for the
/// operation of the BLE controller and pre-allocated inside the stack.
pub type EmbassyWirelessMatterStack<'a, const B: usize, T, N, E = ()> =
    MatterStack<'a, B, EmbassyWirelessBle<T, N, E>>;

/// A type alias for an Embassy implementation of the `Network` trait for a Matter stack running over
/// BLE during commissioning, and then over either WiFi or Thread when operating.
pub type EmbassyWirelessBle<T, N, E = ()> =
    WirelessBle<CriticalSectionRawMutex, T, EmbassyGatt<N, E>>;

#[allow(unused)]
pub(crate) const SLOTS: usize = 20;

/// An embedding of the Trouble Gatt peripheral context for the `WirelessBle` network type from `rs-matter-stack`.
///
/// Allows the memory of this context to be statically allocated and cost-initialized.
///
/// Usage:
/// ```no_run
/// MatterStack<WirelessBle<CriticalSectionRawMutex, Wifi, EmbassyGatt<C, E>>>::new(...);
/// ```
///
/// ... where `E` can be a next-level, user-supplied embedding or just `()` if the user does not need to embed anything.
pub struct EmbassyGatt<N, E = ()> {
    btp_gatt_context: TroubleBtpGattContext<CriticalSectionRawMutex>,
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
            btp_gatt_context: TroubleBtpGattContext::new(),
            net_context: N::INIT,
            embedding: E::INIT,
        }
    }

    /// Return an in-place initializer for the `EspGatt` embedding.
    fn init() -> impl Init<Self> {
        init!(Self {
            btp_gatt_context <- TroubleBtpGattContext::init(),
            net_context <- N::init(),
            embedding <- E::init(),
        })
    }

    /// Return a reference to the Bluedroid Gatt peripheral context.
    pub fn ble_context(&self) -> &TroubleBtpGattContext<CriticalSectionRawMutex> {
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

impl<'a, C> TroubleBtpGattPeripheral<'a, CriticalSectionRawMutex, C>
where
    C: Controller,
{
    pub fn new_for_stack<const B: usize, T, E>(
        ble_ctl: C,
        stack: &'a crate::wireless::EmbassyWirelessMatterStack<B, T, E>,
    ) -> Self
    where
        T: WirelessNetwork,
        E: Embedding + 'static,
    {
        Self::new(
            ble_ctl,
            stack.matter().rand(),
            stack.network().embedding().ble_context(),
        )
    }
}

#[allow(dead_code)]
struct BleDriverTaskImpl<'a, A> {
    task: A,
    rand: Rand,
    context: &'a TroubleBtpGattContext<CriticalSectionRawMutex>,
}

impl<A> BleDriverTask for BleDriverTaskImpl<'_, A>
where
    A: GattTask,
{
    async fn run<C>(&mut self, controller: C) -> Result<(), Error>
    where
        C: trouble_host::Controller,
    {
        let peripheral = TroubleBtpGattPeripheral::new(controller, self.rand, self.context);

        self.task.run(&peripheral).await
    }
}

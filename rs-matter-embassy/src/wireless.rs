//! Wireless: Type aliases and state structs for an Embassy Matter stack running over a wireless network (Wifi or Thread) and BLE.

use crate::matter::crypto::Rng;
use crate::matter::dm::networks::wireless::{WirelessNetwork, WirelessNetworks};
use crate::matter::error::Error;
use crate::matter::utils::init::{init, Init};
use crate::stack::network::{Embedding, Network};
use crate::stack::wireless::{WirelessBle, MAX_WIRELESS_NETWORKS};
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
///
/// `S` is the networks store of the Wifi or Thread interface.
pub type EmbassyWirelessMatterStack<
    'a,
    const B: usize,
    T,
    N,
    E = (),
    S = WirelessNetworks<MAX_WIRELESS_NETWORKS, T>,
> = MatterStack<'a, B, EmbassyWirelessBle<T, N, E, S>>;

/// A type alias for an Embassy implementation of the `Network` trait for a Matter stack running over
/// BLE during commissioning, and then over either WiFi or Thread when operating.
pub type EmbassyWirelessBle<T, N, E = (), S = WirelessNetworks<MAX_WIRELESS_NETWORKS, T>> =
    WirelessBle<T, EmbassyGatt<N, E>, S>;

#[allow(unused)]
pub(crate) const SLOTS: usize = 20;

/// An embedding of the Trouble Gatt peripheral context for the `WirelessBle` network type from `rs-matter-stack`.
///
/// Allows the memory of this context to be statically allocated and cost-initialized.
///
/// Usage:
/// ```ignore
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

impl<'a, R, C> BtpGattPeripheral<'a, R, C>
where
    R: Rng + Copy,
    C: Controller,
{
    pub fn new_for_stack<const B: usize, T, N, E, S>(
        ble_ctl: C,
        rand: Option<R>,
        stack: &'a crate::wireless::EmbassyWirelessMatterStack<B, T, N, E, S>,
    ) -> Self
    where
        T: WirelessNetwork,
        N: Embedding,
        E: Embedding,
        S: Embedding,
    {
        Self::new(ble_ctl, rand, stack.network().embedding().ble_context())
    }
}

/// A `GattPeripheral` over a BLE controller which is already up, and which it only borrows.
#[allow(dead_code)]
pub struct ControllerGattPeripheral<'a, C, R> {
    controller: &'a C,
    rand: Option<R>,
    context: &'a BtpGattContext,
}

impl<'a, C, R> ControllerGattPeripheral<'a, C, R> {
    /// Create a new instance.
    ///
    /// # Arguments
    /// - `controller` - the BLE controller
    /// - `rand` - a random number generator, if a random BLE address should be used
    /// - `context` - the GATT peripheral context
    pub const fn new(controller: &'a C, rand: Option<R>, context: &'a BtpGattContext) -> Self {
        Self {
            controller,
            rand,
            context,
        }
    }
}

impl<C, R> GattPeripheral for ControllerGattPeripheral<'_, C, R>
where
    C: Controller,
    R: Rng + Copy,
{
    async fn run(
        &mut self,
        btp: &Btp,
        service_name: &str,
        service_adv: &AdvData,
    ) -> Result<(), Error> {
        let mut peripheral =
            BtpGattPeripheral::new(ControllerRef::new(self.controller), self.rand, self.context);

        peripheral.run(btp, service_name, service_adv).await
    }
}

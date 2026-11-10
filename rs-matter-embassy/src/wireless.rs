//! Wireless: Type aliases and state structs for an Embassy Matter stack running over a wireless network (Wifi or Thread) and BLE.

use crate::matter::crypto::Rng;
use crate::matter::error::Error;
use crate::matter::utils::init::{init, Init};
use crate::stack::network::{Embedding, StackNetwork};
use crate::stack::MatterStack;

use crate::ble::{BtpGattContext, BtpGattPeripheral, Controller, ControllerRef};
use crate::matter::transport::network::btp::{AdvData, Btp};
use crate::stack::ble::GattPeripheral;

#[cfg(all(feature = "openthread", feature = "embassy-net"))]
pub use any::*;
#[cfg(feature = "openthread")]
pub use thread::*;
#[cfg(feature = "embassy-net")]
pub use wifi::*;

// Any: an Embassy Matter stack for a node that runs Ethernet, Wifi or Thread, picked at runtime.
#[cfg(all(feature = "openthread", feature = "embassy-net"))]
mod any;

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

#[allow(unused)]
pub(crate) const SLOTS: usize = 20;

/// The context of a `MatterStack` that runs Wifi or Thread: the context of the BLE GATT
/// peripheral, and the context of the network (`N`), which the stack holds so that they
/// can be statically allocated (see `EmbassyWifiNetwork`, `EmbassyThreadNetwork` and
/// `EmbassyAnyNetwork`).
pub struct EmbassyGatt<N> {
    btp_gatt_context: BtpGattContext,
    net_context: N,
}

impl<N> EmbassyGatt<N>
where
    N: Embedding,
{
    /// Creates a new instance of the `EspGatt` embedding.
    #[allow(clippy::large_stack_frames)]
    #[inline(always)]
    const fn new() -> Self {
        Self {
            btp_gatt_context: BtpGattContext::new(),
            net_context: N::INIT,
        }
    }

    /// Return an in-place initializer for the `EspGatt` embedding.
    fn init() -> impl Init<Self> {
        init!(Self {
            btp_gatt_context <- BtpGattContext::init(),
            net_context <- N::init(),
        })
    }

    /// Return a reference to the Bluedroid Gatt peripheral context.
    pub fn ble_context(&self) -> &BtpGattContext {
        &self.btp_gatt_context
    }

    /// Return a reference to the network context.
    pub fn net_context(&self) -> &N {
        &self.net_context
    }

    /// Return a mutable reference to the network context.
    pub fn net_context_mut(&mut self) -> &mut N {
        &mut self.net_context
    }
}

impl<N> Embedding for EmbassyGatt<N>
where
    N: Embedding,
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
    pub fn new_for_stack<const B: usize, N, S, E>(
        ble_ctl: C,
        rand: Option<R>,
        stack: &'a MatterStack<B, S, E>,
    ) -> Self
    where
        N: Embedding + 'a,
        S: StackNetwork<Context = EmbassyGatt<N>>,
        E: Embedding,
    {
        Self::new(ble_ctl, rand, stack.context().ble_context())
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

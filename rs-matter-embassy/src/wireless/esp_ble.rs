//! A `GattPeripheral` for the ESP32 family of chips.

use bt_hci::controller::ExternalController;

use esp_radio::ble::controller::BleConnector;

use crate::ble::{BtpGattContext, BtpGattPeripheral};
use crate::matter::crypto::Rng;
use crate::matter::error::Error;
use crate::matter::transport::network::btp::{AdvData, Btp};
use crate::stack::ble::GattPeripheral;
use crate::wireless::SLOTS;

/// A `GattPeripheral` for the ESP32 family of chips: initializes the `esp-radio` BLE controller
/// for the duration of each run, and de-initializes it (by dropping the `BleConnector`)
/// afterwards.
///
/// Shared by the Thread and the Wifi radios, and what makes BLE cost nothing outside of the
/// commissioning windows in concurrent commissioning mode.
pub struct EspGattPeripheral<'a, 'd, R> {
    bt_peripheral: esp_hal::peripherals::BT<'d>,
    rand: Option<R>,
    context: &'a BtpGattContext,
}

impl<'a, 'd, R> EspGattPeripheral<'a, 'd, R> {
    /// Create a new instance of the `EspGattPeripheral` type.
    ///
    /// # Arguments
    /// - `bt_peripheral` - The BT peripheral instance.
    /// - `rand` - A random number generator, if a random BLE address should be used.
    /// - `context` - The GATT peripheral context.
    pub fn new(
        bt_peripheral: esp_hal::peripherals::BT<'d>,
        rand: Option<R>,
        context: &'a BtpGattContext,
    ) -> Self {
        Self {
            bt_peripheral,
            rand,
            context,
        }
    }
}

impl<R> GattPeripheral for EspGattPeripheral<'_, '_, R>
where
    R: Rng + Copy,
{
    async fn run(
        &mut self,
        btp: &Btp,
        service_name: &str,
        service_adv: &AdvData,
    ) -> Result<(), Error> {
        let ble_controller = ExternalController::<_, SLOTS>::new(unwrap!(BleConnector::new(
            self.bt_peripheral.reborrow(),
            Default::default(),
        )));

        let mut peripheral = BtpGattPeripheral::new(ble_controller, self.rand, self.context);

        peripheral.run(btp, service_name, service_adv).await
    }
}

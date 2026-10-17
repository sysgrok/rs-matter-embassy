//! A `BleDriver` for the ESP32 family of chips.

use bt_hci::controller::ExternalController;

use esp_radio::ble::controller::BleConnector;

use rs_matter_stack::matter::error::Error;

use crate::wireless::{BleDriver, BleDriverTask, SLOTS};

/// A `BleDriver` for the ESP32 family of chips: initializes the `esp-radio` BLE controller for
/// the duration of each task run, and de-initializes it (by dropping the `BleConnector`)
/// afterwards.
///
/// Shared by the Thread and the Wifi drivers, and what makes BLE cost nothing outside of the
/// commissioning windows in concurrent commissioning mode.
pub struct EspBleDriver<'d> {
    bt_peripheral: esp_hal::peripherals::BT<'d>,
}

impl<'d> EspBleDriver<'d> {
    /// Create a new instance of the `EspBleDriver` type.
    ///
    /// # Arguments
    /// - `bt_peripheral` - The BT peripheral instance.
    pub fn new(bt_peripheral: esp_hal::peripherals::BT<'d>) -> Self {
        Self { bt_peripheral }
    }
}

impl BleDriver for EspBleDriver<'_> {
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: BleDriverTask,
    {
        let ble_controller = ExternalController::<_, SLOTS>::new(unwrap!(BleConnector::new(
            self.bt_peripheral.reborrow(),
            Default::default(),
        )));

        task.run(ble_controller).await
    }
}

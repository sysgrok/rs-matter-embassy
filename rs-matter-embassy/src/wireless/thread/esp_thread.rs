use bt_hci::controller::ExternalController;

use esp_radio::ble::controller::BleConnector;
use openthread::esp::EspRadio;

use rs_matter_stack::matter::error::Error;

use crate::wireless::SLOTS;

/// A `ThreadRadio` implementation for the ESP32 family of chips.
pub struct EspThreadDriver<'a, 'd> {
    controller: &'a esp_radio::Controller<'d>,
    radio_peripheral: esp_hal::peripherals::IEEE802154<'d>,
    bt_peripheral: esp_hal::peripherals::BT<'d>,
}

impl<'a, 'd> EspThreadDriver<'a, 'd> {
    /// Create a new instance of the `EspThreadRadio` type.
    ///
    /// # Arguments
    /// - `peripheral` - The Thread radio peripheral instance.
    pub fn new(
        controller: &'a esp_radio::Controller<'d>,
        radio_peripheral: esp_hal::peripherals::IEEE802154<'d>,
        bt_peripheral: esp_hal::peripherals::BT<'d>,
    ) -> Self {
        Self {
            controller,
            radio_peripheral,
            bt_peripheral,
        }
    }
}

impl super::ThreadDriver for EspThreadDriver<'_, '_> {
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: super::ThreadDriverTask,
    {
        let radio = EspRadio::new(openthread::esp::Ieee802154::new(
            self.radio_peripheral.reborrow(),
        ));

        task.run(radio).await
    }
}

impl super::ThreadCoexDriver for EspThreadDriver<'_, '_> {
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: super::ThreadCoexDriverTask,
    {
        let ble_controller = ExternalController::<_, SLOTS>::new(unwrap!(BleConnector::new(
            self.controller,
            self.bt_peripheral.reborrow(),
            Default::default(),
        )));

        let radio = EspRadio::new(openthread::esp::Ieee802154::new(
            self.radio_peripheral.reborrow(),
        ));

        task.run(radio, ble_controller).await
    }
}

impl super::BleDriver for EspThreadDriver<'_, '_> {
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: super::BleDriverTask,
    {
        let ble_controller = ExternalController::<_, SLOTS>::new(unwrap!(BleConnector::new(
            self.controller,
            self.bt_peripheral.reborrow(),
            Default::default(),
        )));

        task.run(ble_controller).await
    }
}

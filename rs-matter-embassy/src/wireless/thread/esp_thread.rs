use bt_hci::controller::ExternalController;

use esp_radio::ble::controller::BleConnector;

use openthread::esp::EspRadio;

use rs_matter_stack::matter::error::Error;

use crate::wireless::SLOTS;

/// A `ThreadRadio` implementation for the ESP32 family of chips.
pub struct EspThreadDriver<'d> {
    radio_peripheral: esp_hal::peripherals::IEEE802154<'d>,
    bt_peripheral: esp_hal::peripherals::BT<'d>,
    rx_queue_size: Option<usize>,
}

impl<'d> EspThreadDriver<'d> {
    /// Create a new instance of the `EspThreadRadio` type.
    ///
    /// # Arguments
    /// - `peripheral` - The Thread radio peripheral instance.
    pub fn new(
        radio_peripheral: esp_hal::peripherals::IEEE802154<'d>,
        bt_peripheral: esp_hal::peripherals::BT<'d>,
    ) -> Self {
        Self {
            radio_peripheral,
            bt_peripheral,
            rx_queue_size: None,
        }
    }

    /// Override the esp-radio 802.15.4 receive-queue depth (frames buffered
    /// before drops). Defaults to the `openthread` crate's default; raise it
    /// (e.g. 200) for bursty Matter commissioning / SRP load.
    #[must_use]
    pub fn with_rx_queue_size(mut self, rx_queue_size: usize) -> Self {
        self.rx_queue_size = Some(rx_queue_size);
        self
    }
}

impl super::ThreadDriver for EspThreadDriver<'_> {
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: super::ThreadDriverTask,
    {
        let radio = EspRadio::new(openthread::esp::Ieee802154::new(
            self.radio_peripheral.reborrow(),
        ));
        let radio = match self.rx_queue_size {
            Some(rx_queue_size) => radio.with_rx_queue_size(rx_queue_size),
            None => radio,
        };

        task.run(radio).await
    }
}

impl super::ThreadCoexDriver for EspThreadDriver<'_> {
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: super::ThreadCoexDriverTask,
    {
        let ble_controller = ExternalController::<_, SLOTS>::new(unwrap!(BleConnector::new(
            self.bt_peripheral.reborrow(),
            Default::default(),
        )));

        let radio = EspRadio::new(openthread::esp::Ieee802154::new(
            self.radio_peripheral.reborrow(),
        ));
        let radio = match self.rx_queue_size {
            Some(rx_queue_size) => radio.with_rx_queue_size(rx_queue_size),
            None => radio,
        };

        task.run(radio, ble_controller).await
    }
}

impl super::BleDriver for EspThreadDriver<'_> {
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: super::BleDriverTask,
    {
        let ble_controller = ExternalController::<_, SLOTS>::new(unwrap!(BleConnector::new(
            self.bt_peripheral.reborrow(),
            Default::default(),
        )));

        task.run(ble_controller).await
    }
}

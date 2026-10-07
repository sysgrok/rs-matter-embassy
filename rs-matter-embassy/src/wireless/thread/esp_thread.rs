use openthread::esp::EspRadio;

use rs_matter_stack::matter::error::Error;

use crate::wireless::esp::EspBleDriver;

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

    /// The IEEE EUI-64 of the chip's IEEE 802.15.4 radio, derived from the factory eFuse MAC
    /// address the way ESP-IDF does it (`esp_read_mac` with `ESP_MAC_IEEE802154`): the base
    /// MAC's OUI, the two `MAC_EXT` eFuse bytes, then the base MAC's device bytes - e.g.
    /// `60:55:f9` + `ff:fe` + `f7:2c:a2`.
    ///
    /// Stable across reboots, unlike a random one, so the SRP host name derived from it (and
    /// the hardware address the node reports) stay the same - which is what a device that
    /// deep-sleeps, and hence reboots, between wake-ups needs, or else every wake-up registers
    /// a new host with the SRP server.
    pub fn ieee_eui64() -> [u8; 8] {
        let base = esp_hal::efuse::base_mac_address();
        let base = base.as_bytes();
        let ext = esp_hal::efuse::read_field_le::<[u8; 2]>(esp_hal::efuse::MAC_EXT);

        [
            base[0], base[1], base[2], ext[0], ext[1], base[3], base[4], base[5],
        ]
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
        let radio = EspRadio::new(openthread::esp::Ieee802154::new(
            self.radio_peripheral.reborrow(),
        ));
        let radio = match self.rx_queue_size {
            Some(rx_queue_size) => radio.with_rx_queue_size(rx_queue_size),
            None => radio,
        };

        // The BLE controller is created (and the `esp-radio` BLE stack initialized) only while
        // the task actually runs BLE - see `EspBleDriver`.
        task.run(radio, EspBleDriver::new(self.bt_peripheral.reborrow()))
            .await
    }
}

impl super::BleDriver for EspThreadDriver<'_> {
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: super::BleDriverTask,
    {
        EspBleDriver::new(self.bt_peripheral.reborrow())
            .run(task)
            .await
    }
}

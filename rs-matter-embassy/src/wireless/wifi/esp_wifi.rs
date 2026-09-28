use crate::matter::error::Error;
use crate::wifi::esp::EspWifiController;
use crate::wireless::esp::EspBleDriver;

/// A `WifiDriver` implementation for the ESP32 family of chips.
pub struct EspWifiDriver<'d> {
    wifi_peripheral: esp_hal::peripherals::WIFI<'d>,
    bt_peripheral: esp_hal::peripherals::BT<'d>,
}

impl<'d> EspWifiDriver<'d> {
    /// Create a new instance of the `Esp32WifiDriver` type.
    ///
    /// # Arguments
    /// - `controller` - The `esp-radio` controller instance.
    /// - `peripheral` - The Wifi peripheral instance.
    pub fn new(
        wifi_peripheral: esp_hal::peripherals::WIFI<'d>,
        bt_peripheral: esp_hal::peripherals::BT<'d>,
    ) -> Self {
        Self {
            wifi_peripheral,
            bt_peripheral,
        }
    }
}

impl super::WifiDriver for EspWifiDriver<'_> {
    type NetCtl<'a>
        = crate::wifi::esp::EspWifiController<'a>
    where
        Self: 'a;

    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: super::WifiDriverTask,
    {
        let mut controller = unwrap!(esp_radio::wifi::WifiController::new(
            self.wifi_peripheral.reborrow(),
            esp_radio::wifi::ControllerConfig::default(),
        ));

        // esp32c6-specific - need to boost the power to get a good signal
        unwrap!(controller.set_power_saving(esp_radio::wifi::PowerSaveMode::None));

        // Since esp-hal PR #5706, esp-radio sets the max TX power to 20 (5dBm) instead of leaving
        // the ESP-IDF default of 80 (20dBm). With 5dBm, scans find few APs and connecting fails
        // with `AuthenticationExpired`, so restore the previous level.
        unwrap!(controller.set_max_tx_power(80));

        task.run(
            esp_radio::wifi::Interface::station(),
            EspWifiController::new(controller),
        )
        .await
    }
}

impl super::WifiCoexDriver for EspWifiDriver<'_> {
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: super::WifiCoexDriverTask,
    {
        // Wi-Fi power management while the station is disconnected starves the BLE controller:
        // BLE advertisements never go on air while Wi-Fi is up but not (yet) connected, which is
        // exactly the situation during concurrent commissioning. esp-radio hard-coded this to
        // `false` until esp-hal PR #6144, which made it configurable with the ESP-IDF default (`true`).
        let mut controller = unwrap!(esp_radio::wifi::WifiController::new(
            self.wifi_peripheral.reborrow(),
            esp_radio::wifi::ControllerConfig::default().with_sta_disconnected_pm(false),
        ));

        // esp32c6-specific - need to boost the power to get a good signal
        unwrap!(controller.set_power_saving(esp_radio::wifi::PowerSaveMode::None));

        // Since esp-hal PR #5706, esp-radio sets the max TX power to 20 (5dBm) instead of leaving
        // the ESP-IDF default of 80 (20dBm). With 5dBm, scans find few APs and connecting fails
        // with `AuthenticationExpired`, so restore the previous level.
        unwrap!(controller.set_max_tx_power(80));

        // The BLE controller is created (and the `esp-radio` BLE stack initialized) only while
        // the task actually runs BLE - see `EspBleDriver`.
        task.run(
            esp_radio::wifi::Interface::station(),
            EspWifiController::new(controller),
            EspBleDriver::new(self.bt_peripheral.reborrow()),
        )
        .await
    }
}

impl super::BleDriver for EspWifiDriver<'_> {
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: super::BleDriverTask,
    {
        EspBleDriver::new(self.bt_peripheral.reborrow())
            .run(task)
            .await
    }
}

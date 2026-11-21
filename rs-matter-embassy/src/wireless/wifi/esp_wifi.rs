use crate::matter::crypto::Rng;
use crate::matter::error::Error;
use crate::stack::wireless::{
    GattDriver, GattTask, WifiCoexDriver, WifiCoexTask, WifiDriver, WifiTask,
};
use crate::wifi::esp::EspWifiController;
use crate::wireless::esp::EspGattPeripheral;

use super::EmbassyWifi;

pub use crate::wifi::esp::WifiConfig;
pub use crate::wifi::scan::WifiScanResultConfig;
pub use esp_radio::wifi::scan::{ScanConfig, ScanTypeConfig};
pub use esp_radio::wifi::sta::ScanMethod;
pub use esp_radio::wifi::{Protocol, Protocols};

/// The Wifi and BLE radio of the ESP32 family of chips.
pub struct EspWifiRadio<'d> {
    wifi_peripheral: esp_hal::peripherals::WIFI<'d>,
    bt_peripheral: esp_hal::peripherals::BT<'d>,
    protocols: Option<Protocols>,
    wifi_config: Option<WifiConfig>,
}

impl<'d> EspWifiRadio<'d> {
    /// Create a new instance of the `EspWifiRadio` type.
    ///
    /// # Arguments
    /// - `wifi_peripheral` - The Wifi peripheral instance.
    /// - `bt_peripheral` - The BT peripheral instance.
    pub fn new(
        wifi_peripheral: esp_hal::peripherals::WIFI<'d>,
        bt_peripheral: esp_hal::peripherals::BT<'d>,
    ) -> Self {
        Self {
            wifi_peripheral,
            bt_peripheral,
            protocols: None,
            wifi_config: None,
        }
    }

    /// Override the 2.4 GHz 802.11 protocol mask used for station scanning and association.
    pub const fn with_protocols(mut self, protocols: Protocols) -> Self {
        self.protocols = Some(protocols);
        self
    }

    /// Supply the aggregate ESP Wi-Fi configuration for scanning, association, and result handling.
    pub const fn with_wifi_config(mut self, wifi_config: WifiConfig) -> Self {
        self.wifi_config = Some(wifi_config);
        self
    }

    fn configure_controller<'a>(
        controller: esp_radio::wifi::WifiController<'a>,
        protocols: Option<Protocols>,
        wifi_config: Option<WifiConfig>,
    ) -> EspWifiController<'a> {
        let mut wifi = EspWifiController::new(controller);
        if let Some(protocols) = protocols {
            wifi = wifi.with_protocols(protocols);
        }
        if let Some(wifi_config) = wifi_config {
            wifi = wifi.with_wifi_config(wifi_config);
        }
        wifi
    }
}

impl<R> WifiDriver for EmbassyWifi<'_, EspWifiRadio<'_>, R>
where
    R: Rng + Copy,
{
    type NetCtl<'a>
        = crate::wifi::esp::EspWifiController<'a>
    where
        Self: 'a;

    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: WifiTask,
    {
        let mut controller = unwrap!(esp_radio::wifi::WifiController::new(
            self.radio.wifi_peripheral.reborrow(),
            esp_radio::wifi::ControllerConfig::default(),
        ));

        // esp32c6-specific - need to boost the power to get a good signal
        unwrap!(controller.set_power_saving(esp_radio::wifi::PowerSaveMode::None));

        // Since esp-hal PR #5706, esp-radio sets the max TX power to 20 (5dBm) instead of leaving
        // the ESP-IDF default of 80 (20dBm). With 5dBm, scans find few APs and connecting fails
        // with `AuthenticationExpired`, so restore the previous level.
        unwrap!(controller.set_max_tx_power(80));

        let wifi = EspWifiRadio::configure_controller(
            controller,
            self.radio.protocols,
            self.radio.wifi_config,
        );

        self.runner
            .run(esp_radio::wifi::Interface::station(), wifi, task)
            .await
    }
}

impl<R> WifiCoexDriver for EmbassyWifi<'_, EspWifiRadio<'_>, R>
where
    R: Rng + Copy,
{
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: WifiCoexTask,
    {
        // Wi-Fi power management while the station is disconnected starves the BLE controller:
        // BLE advertisements never go on air while Wi-Fi is up but not (yet) connected, which is
        // exactly the situation during concurrent commissioning. esp-radio hard-coded this to
        // `false` until esp-hal PR #6144, which made it configurable with the ESP-IDF default (`true`).
        let mut controller = unwrap!(esp_radio::wifi::WifiController::new(
            self.radio.wifi_peripheral.reborrow(),
            esp_radio::wifi::ControllerConfig::default().with_sta_disconnected_pm(false),
        ));

        // esp32c6-specific - need to boost the power to get a good signal
        unwrap!(controller.set_power_saving(esp_radio::wifi::PowerSaveMode::None));

        // Since esp-hal PR #5706, esp-radio sets the max TX power to 20 (5dBm) instead of leaving
        // the ESP-IDF default of 80 (20dBm). With 5dBm, scans find few APs and connecting fails
        // with `AuthenticationExpired`, so restore the previous level.
        unwrap!(controller.set_max_tx_power(80));

        // The BLE controller is created (and the `esp-radio` BLE stack initialized) only while
        // the task actually runs BLE - see `EspGattPeripheral`.
        let gatt = EspGattPeripheral::new(
            self.radio.bt_peripheral.reborrow(),
            self.runner.ble_rand(),
            self.runner.ble_context(),
        );

        let wifi = EspWifiRadio::configure_controller(
            controller,
            self.radio.protocols,
            self.radio.wifi_config,
        );

        self.runner
            .run_coex(esp_radio::wifi::Interface::station(), wifi, gatt, task)
            .await
    }
}

impl<R> GattDriver for EmbassyWifi<'_, EspWifiRadio<'_>, R>
where
    R: Rng + Copy,
{
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: GattTask,
    {
        task.run(EspGattPeripheral::new(
            self.radio.bt_peripheral.reborrow(),
            self.runner.ble_rand(),
            self.runner.ble_context(),
        ))
        .await
    }
}

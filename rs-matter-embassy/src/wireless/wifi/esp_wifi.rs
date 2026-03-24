use bt_hci::controller::ExternalController;

use esp_radio::ble::controller::BleConnector;

use crate::matter::error::Error;
use crate::wifi::esp::EspWifiController;
use crate::wireless::SLOTS;

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
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: super::WifiDriverTask,
    {
        let (mut controller, wifi_interface) = unwrap!(esp_radio::wifi::new(
            self.wifi_peripheral.reborrow(),
            esp_radio::wifi::ControllerConfig::default(),
        ));

        // esp32c6-specific - need to boost the power to get a good signal
        unwrap!(controller.set_power_saving(esp_radio::wifi::PowerSaveMode::None));

        task.run(wifi_interface.station, EspWifiController::new(controller))
            .await
    }
}

impl super::WifiCoexDriver for EspWifiDriver<'_> {
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: super::WifiCoexDriverTask,
    {
        let ble_ctl = ExternalController::<_, SLOTS>::new(unwrap!(BleConnector::new(
            self.bt_peripheral.reborrow(),
            Default::default(),
        )));

        let (mut controller, wifi_interface) = unwrap!(esp_radio::wifi::new(
            self.wifi_peripheral.reborrow(),
            esp_radio::wifi::ControllerConfig::default(),
        ));

        // esp32c6-specific - need to boost the power to get a good signal
        unwrap!(controller.set_power_saving(esp_radio::wifi::PowerSaveMode::None));

        task.run(
            wifi_interface.station,
            EspWifiController::new(controller),
            ble_ctl,
        )
        .await
    }
}

impl super::BleDriver for EspWifiDriver<'_> {
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

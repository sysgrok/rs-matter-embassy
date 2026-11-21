//! Compile-time API coverage for `WifiConfig` on `EspWifiRadio` and `EspWifiController`.

#![no_std]
#![cfg(all(feature = "esp", feature = "embassy-net"))]

use esp_radio::wifi::WifiController;
use rs_matter_embassy::wifi::esp::EspWifiController;
use rs_matter_embassy::wireless::esp::{
    EspWifiRadio, ScanConfig, ScanMethod, ScanTypeConfig, WifiConfig, WifiScanResultConfig,
};

#[allow(dead_code)]
fn construct_default_wifi_config() -> WifiConfig {
    WifiConfig::default()
}

#[allow(dead_code)]
fn construct_custom_wifi_config() -> WifiConfig {
    WifiConfig::new()
        .with_scan(
            ScanConfig::default().with_scan_type(ScanTypeConfig::Active {
                min: esp_hal::time::Duration::from_millis(30),
                max: esp_hal::time::Duration::from_millis(100),
            }),
        )
        .with_scan_method(ScanMethod::Fast)
        .with_failure_retry_cnt(3)
        .with_scan_results(
            WifiScanResultConfig::new()
                .with_sort_by_rssi(true)
                .with_deduplicate_ssids(true)
                .with_filter_hidden_ssids(true)
                .with_max_results(10),
        )
}

#[allow(dead_code)]
fn apply_to_radio<'d>(
    wifi: esp_hal::peripherals::WIFI<'d>,
    bt: esp_hal::peripherals::BT<'d>,
) -> EspWifiRadio<'d> {
    EspWifiRadio::new(wifi, bt).with_wifi_config(construct_custom_wifi_config())
}

#[allow(dead_code)]
fn apply_to_controller<'a>(controller: WifiController<'a>) -> EspWifiController<'a> {
    EspWifiController::new(controller).with_wifi_config(construct_custom_wifi_config())
}

use core::cell::Cell;
use core::cmp::Reverse;

use esp_radio::wifi::scan::{ScanConfig, ScanTypeConfig};
use esp_radio::wifi::sta::{ScanMethod, StationConfig};
use esp_radio::wifi::{
    AuthenticationMethod, AuthenticationMethodConfig, Config, ConnectionError, Password, Protocols,
    Ssid, WifiController, WifiError,
};

use crate::matter::dm::clusters::net_comm::{
    NetCtl, NetCtlError, NetworkScanInfo, NetworkType, WiFiBandEnum, WiFiSecurityBitmap,
    WirelessCreds,
};
use crate::matter::dm::clusters::wifi_diag::{
    SecurityTypeEnum, WiFiVersionEnum, WifiDiag, WirelessDiag,
};
use crate::matter::dm::networks::NetChangeNotif;
use crate::matter::error::{Error, ErrorCode};
use crate::matter::tlv::Nullable;
use crate::matter::utils::sync::blocking::Mutex;
use crate::matter::utils::sync::IfMutex;
use crate::matter::utils::timers::{Duration, Timer};

const DEFAULT_SCAN_TYPE: ScanTypeConfig = ScanTypeConfig::Active {
    min: esp_hal::time::Duration::from_millis(0),
    max: esp_hal::time::Duration::from_millis(120),
};

/// Configuration for ESP Wi-Fi scanning, 802.11 protocols, and station association.
///
/// `WifiConfig::default()` uses ESP-IDF's 120 ms active scan dwell (`min: 0ms, max: 120ms`)
/// when `scan` is `None`, and leaves protocol and station association overrides unset.
#[derive(Clone, Copy, Default)]
pub struct WifiConfig {
    /// Optional driver configuration for Matter network discovery scans.
    ///
    /// When `None`, active scans use `min: 0ms, max: 120ms` per channel.
    pub scan: Option<ScanConfig>,
    /// Optional 802.11 protocols used for station scanning and association.
    pub protocols: Option<Protocols>,
    /// Optional station association scan method override (independent of `scan`).
    pub scan_method: Option<ScanMethod>,
    /// Optional station association failure retry count override.
    ///
    /// Note: in `esp-radio`, this only takes effect when `scan_method` is `ScanMethod::AllChannels`.
    pub failure_retry_cnt: Option<u8>,
}

impl WifiConfig {
    /// Create a default `WifiConfig` with all overrides unset.
    pub const fn new() -> Self {
        Self {
            scan: None,
            protocols: None,
            scan_method: None,
            failure_retry_cnt: None,
        }
    }

    /// Set the driver `ScanConfig` used for Matter network discovery scans.
    pub const fn with_scan(mut self, scan: ScanConfig) -> Self {
        self.scan = Some(scan);
        self
    }

    /// Override the 802.11 protocols used for station scanning and association.
    pub const fn with_protocols(mut self, protocols: Protocols) -> Self {
        self.protocols = Some(protocols);
        self
    }

    /// Set the station association `ScanMethod` override.
    pub const fn with_scan_method(mut self, scan_method: ScanMethod) -> Self {
        self.scan_method = Some(scan_method);
        self
    }

    /// Set the station association failure retry count override.
    ///
    /// Note: in `esp-radio`, this only takes effect when `scan_method` is `ScanMethod::AllChannels`.
    pub const fn with_failure_retry_cnt(mut self, failure_retry_cnt: u8) -> Self {
        self.failure_retry_cnt = Some(failure_retry_cnt);
        self
    }
}

/// An adaptor from the `esp-wifi` Wifi controller API to the `rs-matter` Wifi controller API
pub struct EspWifiController<'a> {
    controller: IfMutex<WifiController<'a>>,
    connected: Mutex<Cell<bool>>,
    config: WifiConfig,
}

impl<'a> EspWifiController<'a> {
    /// Create a new instance of the `Esp32Controller` type.
    ///
    /// # Arguments
    /// - `controller` - The `esp-wifi` Wifi controller instance.
    pub const fn new(controller: WifiController<'a>) -> Self {
        Self {
            controller: IfMutex::new(controller),
            connected: Mutex::new(Cell::new(false)),
            config: WifiConfig::new(),
        }
    }

    /// Supply the ESP Wi-Fi configuration for scanning, 802.11 protocols, and station association.
    pub const fn with_wifi_config(mut self, config: WifiConfig) -> Self {
        self.config = config;
        self
    }

    /// The AP the station is associated with.
    ///
    /// `None` if not associated, or while a scan or a connect holds the controller.
    fn ap_info(&self) -> Option<ApInfo> {
        // Not returning `AccessPointInfo` as is, because its module differs across `esp-radio` versions
        let ap_info = self.controller.try_lock().ok()?.ap_info().ok()?;

        Some(ApInfo {
            bssid: ap_info.bssid,
            channel: ap_info.channel,
            signal_strength: ap_info.signal_strength,
            auth_method: ap_info.auth_method,
        })
    }
}

/// The parts of the `esp-radio` `AccessPointInfo` the Wi-Fi diagnostics need
struct ApInfo {
    bssid: [u8; 6],
    channel: u8,
    signal_strength: i8,
    auth_method: Option<AuthenticationMethod>,
}

impl NetCtl for EspWifiController<'_> {
    fn net_type(&self) -> NetworkType {
        NetworkType::Wifi
    }

    async fn scan<F>(&self, network: Option<&[u8]>, mut f: F) -> Result<(), NetCtlError>
    where
        F: FnMut(&NetworkScanInfo) -> Result<(), Error>,
    {
        info!("Wifi scan request");

        let mut ctl = self.controller.lock().await;

        // Start Wifi in client mode for scanning
        let mut sta_config = StationConfig::default();
        if let Some(protocols) = self.config.protocols {
            sta_config = sta_config.with_protocols(protocols);
        }
        ctl.set_config(&Config::Station(sta_config))
            .map_err(to_ctl_err)?;
        info!("Wifi configuration updated for scanning");

        let mut scan_config = self
            .config
            .scan
            .unwrap_or_else(|| ScanConfig::default().with_scan_type(DEFAULT_SCAN_TYPE));
        if let Some(network) = network.filter(|n| !n.is_empty()) {
            scan_config = scan_config.with_ssid(Ssid::try_from(network).map_err(to_ctl_err)?);
        }

        let mut aps = ctl.scan_async(&scan_config).await.map_err(to_err)?;
        aps.sort_unstable_by_key(|ap| Reverse(ap.signal_strength));

        info!("Wifi scan complete, reporting {} results", aps.len());

        for ap in aps {
            f(&NetworkScanInfo::Wifi {
                ssid: ap.ssid.as_str().as_bytes(),
                bssid: &ap.bssid,
                channel: ap.channel as _,
                rssi: ap.signal_strength,
                band: WiFiBandEnum::V2G4, // TODO: Once c5 is out we can no longer hard-code this
                security: match ap.auth_method {
                    Some(AuthenticationMethod::None) => WiFiSecurityBitmap::UNENCRYPTED,
                    Some(AuthenticationMethod::Wep) => WiFiSecurityBitmap::WEP,
                    Some(AuthenticationMethod::Wpa) => WiFiSecurityBitmap::WPA_PERSONAL,
                    Some(AuthenticationMethod::Wpa2Personal) => WiFiSecurityBitmap::WPA_2_PERSONAL,
                    Some(AuthenticationMethod::WpaWpa2Personal) => {
                        WiFiSecurityBitmap::WPA_PERSONAL | WiFiSecurityBitmap::WPA_2_PERSONAL
                    }
                    Some(AuthenticationMethod::Wpa2Wpa3Personal) => {
                        WiFiSecurityBitmap::WPA_2_PERSONAL | WiFiSecurityBitmap::WPA_3_PERSONAL
                    }
                    Some(AuthenticationMethod::Wpa3Personal) => WiFiSecurityBitmap::WPA_3_PERSONAL,
                    Some(AuthenticationMethod::Wpa2Enterprise) => {
                        WiFiSecurityBitmap::WPA_2_PERSONAL
                    }
                    _ => WiFiSecurityBitmap::WPA_2_PERSONAL, // Best guess
                },
            })?;
        }

        info!("Wifi scan complete");

        Ok(())
    }

    async fn connect(&self, creds: &WirelessCreds<'_>) -> Result<(), NetCtlError> {
        let WirelessCreds::Wifi { ssid, pass } = creds else {
            return Err(NetCtlError::Other(ErrorCode::InvalidAction.into()));
        };

        let mut ctl = self.controller.lock().await;

        info!(
            "Wifi connect request for SSID {}",
            core::str::from_utf8(ssid).unwrap_or("???")
        );

        // If already connected, disconnect first to ensure a clean state.
        // Calling connect_async() while already connected can hang forever
        // because the ESP WiFi driver may not generate any StationConnected
        // or StationDisconnected events, leaving the event subscriber loop
        // waiting indefinitely.
        if ctl.is_connected() {
            info!("Wifi already connected, disconnecting first");
            let _ = ctl.disconnect_async().await;

            self.connected.lock(|connected| {
                connected.set(false);
            });
        }

        // An empty password means an open network; anything else is taken as WPA2-Personal,
        // which is what the driver used to assume for a configured password
        let authentication = if pass.is_empty() {
            AuthenticationMethodConfig::Open
        } else {
            AuthenticationMethodConfig::Wpa2Personal(Password::try_from(*pass).map_err(to_ctl_err)?)
        };

        let mut sta_config = StationConfig::default()
            .with_ssid(Ssid::try_from(*ssid).map_err(to_ctl_err)?)
            .with_authentication(authentication);
        if let Some(protocols) = self.config.protocols {
            sta_config = sta_config.with_protocols(protocols);
        }
        if let Some(scan_method) = self.config.scan_method {
            sta_config = sta_config.with_scan_method(scan_method);
        }
        if let Some(failure_retry_cnt) = self.config.failure_retry_cnt {
            sta_config = sta_config.with_failure_retry_cnt(failure_retry_cnt);
        }

        ctl.set_config(&Config::Station(sta_config))
            .map_err(to_ctl_err)?;
        info!("Wifi configuration updated");

        ctl.connect_async().await.map_err(to_ctl_err2)?;

        self.connected.lock(|connected| {
            info!("Wifi state updated: {} -> {}", connected.get(), true);
            connected.set(true);
        });

        info!("Wifi connected");

        Ok(())
    }
}

impl NetChangeNotif for EspWifiController<'_> {
    async fn wait_changed(&self) {
        let fetch_connected = || async {
            let ctl = self.controller.lock().await;

            let new_connected = ctl.is_connected();
            self.connected.lock(|connected| {
                if connected.get() != new_connected {
                    warn!(
                        "Wifi state changed: {} -> {}",
                        connected.get(),
                        new_connected
                    );

                    connected.set(new_connected);
                    true
                } else {
                    false
                }
            })
        };

        loop {
            if fetch_connected().await {
                return;
            }

            Timer::after(Duration::from_secs(2)).await;
        }

        // Code below is commented out because it is a bit complex to wait on events in the first place and at the
        // same time - allow `connect` and `scan` to be called in parallel
        // TODO: Fix in future by allowing `scan` and `connect` to signal `wait_changed` when they need to get hold of
        // the ctl, as well as when they are done holding on the ctl

        // {
        //     let mut ctl = self.0.lock().await;
        //     ctl.wait_for_all_events(enumset::EnumSet::all(), true).await;
        // }
    }
}

impl WirelessDiag for EspWifiController<'_> {
    fn connected(&self) -> Result<bool, Error> {
        Ok(self.connected.lock(|connected| connected.get()))
    }
}

impl WifiDiag for EspWifiController<'_> {
    fn bssid(&self, f: &mut dyn FnMut(Option<&[u8]>) -> Result<(), Error>) -> Result<(), Error> {
        let ap_info = self.ap_info();

        f(ap_info.as_ref().map(|ap_info| ap_info.bssid.as_slice()))
    }

    fn security_type(&self) -> Result<Nullable<SecurityTypeEnum>, Error> {
        let Some(ap_info) = self.ap_info() else {
            return Ok(Nullable::none());
        };

        // Mixed modes do not tell which of the two is in use
        let security_type = match ap_info.auth_method {
            Some(AuthenticationMethod::None) => SecurityTypeEnum::None,
            Some(AuthenticationMethod::Wep) => SecurityTypeEnum::WEP,
            Some(AuthenticationMethod::Wpa) => SecurityTypeEnum::WPA,
            Some(AuthenticationMethod::Wpa2Personal) => SecurityTypeEnum::WPA2,
            Some(AuthenticationMethod::Wpa3Personal) => SecurityTypeEnum::WPA3,
            _ => SecurityTypeEnum::Unspecified,
        };

        Ok(Nullable::some(security_type))
    }

    fn wi_fi_version(&self) -> Result<Nullable<WiFiVersionEnum>, Error> {
        // `esp-radio` does not report the negotiated PHY mode
        Ok(Nullable::none())
    }

    fn channel_number(&self) -> Result<Nullable<u16>, Error> {
        Ok(self
            .ap_info()
            .map(|ap_info| Nullable::some(ap_info.channel as _))
            .unwrap_or(Nullable::none()))
    }

    fn rssi(&self) -> Result<Nullable<i8>, Error> {
        Ok(self
            .ap_info()
            .map(|ap_info| Nullable::some(ap_info.signal_strength))
            .unwrap_or(Nullable::none()))
    }
}

fn to_ctl_err(e: WifiError) -> NetCtlError {
    error!("Wifi error: {:?}", e);

    match e {
        WifiError::NotConnected => NetCtlError::OtherConnectionFailure,
        WifiError::Unsupported => NetCtlError::UnsupportedSecurity,
        _ => NetCtlError::Other(ErrorCode::NoNetworkInterface.into()),
    }
}

fn to_ctl_err2(e: ConnectionError) -> NetCtlError {
    match e {
        ConnectionError::Failed(e) => {
            error!("Connection error: {:?}", e);

            NetCtlError::OtherConnectionFailure
        }
        ConnectionError::WifiError(e) => to_ctl_err(e),
        _ => {
            error!("Connection error: {:?}", e);

            NetCtlError::Other(ErrorCode::NoNetworkInterface.into())
        }
    }
}

fn to_err(e: WifiError) -> Error {
    error!("Wifi error: {:?}", e);
    ErrorCode::NoNetworkInterface.into()
}

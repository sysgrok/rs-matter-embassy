use core::cell::Cell;

use cyw43::{Control, JoinOptions, ScanOptions};

use embassy_sync::blocking_mutex;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::mutex::Mutex;

use crate::fmt::Bytes;

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

/// An adaptor from the `cyw43` Wifi controller API to the `rs-matter` Wifi controller API
pub struct Cyw43WifiController<'a, M>(Mutex<M, Control<'a>>, blocking_mutex::Mutex<M, Cell<bool>>)
where
    M: RawMutex;

impl<'a, M> Cyw43WifiController<'a, M>
where
    M: RawMutex,
{
    /// Create a new instance of the `Cyw43WifiController` type.
    ///
    /// # Arguments
    /// - `controller` - The `cyw43` Wifi controller instance.
    pub const fn new(controller: Control<'a>) -> Self {
        Self(
            Mutex::new(controller),
            blocking_mutex::Mutex::new(Cell::new(false)),
        )
    }
}

impl<M> NetCtl for Cyw43WifiController<'_, M>
where
    M: RawMutex,
{
    fn net_type(&self) -> NetworkType {
        NetworkType::Wifi
    }

    async fn scan<F>(&self, network: Option<&[u8]>, mut f: F) -> Result<(), NetCtlError>
    where
        F: FnMut(&NetworkScanInfo) -> Result<(), Error>,
    {
        info!("Wifi scan request");

        let mut ctl = self.0.lock().await;

        let mut scan_options = ScanOptions::default();
        //scan_options.scan_type = ScanType::Active;

        if let Some(network) = network {
            scan_options.ssid = Some(unwrap!(core::str::from_utf8(network)
                .unwrap_or("???")
                .try_into()));
        }

        let mut scanner = ctl.scan(scan_options).await;

        info!("Wifi scan started");

        while let Some(ap) = scanner.next().await {
            if ap.ssid_len > 0 {
                f(&NetworkScanInfo::Wifi {
                    ssid: &ap.ssid[..ap.ssid_len as _],
                    bssid: &ap.bssid,
                    channel: ap.chanspec,
                    rssi: ap.rssi as _,
                    band: WiFiBandEnum::V2G4, // cyw43 only supports 2.4GHz
                    security: WiFiSecurityBitmap::WPA_2_PERSONAL, // TODO
                })?;
            } else {
                info!(
                    "Skipping scan result for a hidden network {}",
                    Bytes(&ap.bssid)
                );
            }
        }

        info!("Wifi scan complete");

        Ok(())
    }

    async fn connect(&self, creds: &WirelessCreds<'_>) -> Result<(), NetCtlError> {
        let WirelessCreds::Wifi { ssid, pass } = creds else {
            return Err(NetCtlError::Other(ErrorCode::InvalidAction.into()));
        };

        let ssid = core::str::from_utf8(ssid).unwrap_or("???");

        info!("Wifi connect request for SSID {}", ssid);

        let mut ctl = self.0.lock().await;

        ctl.leave().await;
        self.1.lock(|connected| connected.set(false));
        info!("Disconnected from current Wifi AP (if any)");

        ctl.join(ssid, JoinOptions::new(pass)) // TODO: Try with something else besides Wpa2Wpa3
            .await
            .map_err(to_ctl_err)?;

        self.1.lock(|connected| {
            info!("Wifi state updated: {} -> {}", connected.get(), true);
            connected.set(true)
        });

        info!("Wifi connected");

        info!("Wifi connect complete");

        Ok(())
    }
}

impl<M> NetChangeNotif for Cyw43WifiController<'_, M>
where
    M: RawMutex,
{
    async fn wait_changed(&self) {
        let fetch_connected = || async {
            let _ctl = self.0.lock().await;

            // TODO: Cyw43 Control does not have a way to check if we are connected or not
            // Need to upstream a way to check that or else we cannot detect Wifi disconnection events
            let new_connected = self.1.lock(|connected| connected.get());
            self.1.lock(|connected| {
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

            // Cyw43 does not have any means to wait on a state change - nor it has any means to detect disconnection -
            // so here we just wait for 2 seconds
            embassy_time::Timer::after(embassy_time::Duration::from_secs(2)).await;
        }
    }
}

impl<M> WirelessDiag for Cyw43WifiController<'_, M>
where
    M: RawMutex,
{
    fn connected(&self) -> Result<bool, Error> {
        Ok(self.1.lock(|connected| connected.get()))
    }
}

impl<M> WifiDiag for Cyw43WifiController<'_, M>
where
    M: RawMutex,
{
    fn bssid(&self, f: &mut dyn FnMut(Option<&[u8]>) -> Result<(), Error>) -> Result<(), Error> {
        f(None)
    }

    fn security_type(&self) -> Result<Nullable<SecurityTypeEnum>, Error> {
        Ok(Nullable::none())
    }

    fn wi_fi_version(&self) -> Result<Nullable<WiFiVersionEnum>, Error> {
        Ok(Nullable::none())
    }

    fn channel_number(&self) -> Result<Nullable<u16>, Error> {
        Ok(Nullable::none())
    }

    fn rssi(&self) -> Result<Nullable<i8>, Error> {
        Ok(Nullable::none())
    }
}

fn to_ctl_err(e: cyw43::ControlError) -> NetCtlError {
    error!("Wifi error: {:?}", debug2format!(e));

    NetCtlError::OtherConnectionFailure
}

//! Network interface: `OtNetif` - a `Netif` trait implementation for `openthread`
//! mDNS impl: `OtMdns` - an mDNS trait implementation for `openthread` using Thread SRP

use core::fmt::Write;
use core::future::poll_fn;

use embassy_futures::select::select;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Instant, Timer};

use openthread::{
    Channels, OpenThread, OtError, OtResources, OtSrpResources, OtUdpResources, RamSettings,
    RamSettingsChange, SettingsKey, SharedRamSettings, SrpConf, SrpService,
};
use rs_matter_stack::matter::crypto::Crypto;
use rs_matter_stack::matter::dm::ChangeNotify;
use rs_matter_stack::matter::transport::network::mdns::Service;
use rs_matter_stack::matter::{Matter, MatterMdnsService};
use rs_matter_stack::mdns::Mdns;
use rs_matter_stack::nal::noop::NoopNet;
use rs_matter_stack::nal::{NetStack, UdpBind};

use crate::fmt::Bytes;

use crate::matter::dm::clusters::gen_diag::{InterfaceTypeEnum, NetifDiag, NetifInfo};
use crate::matter::dm::clusters::net_comm::{
    NetCtl, NetCtlError, NetworkScanInfo, NetworkType, WirelessCreds,
};
use crate::matter::dm::clusters::thread_diag::{
    NeighborTable, NetworkFaultEnum, OperationalDatasetComponents, RouteTable, RoutingRoleEnum,
    SecurityPolicy, ThreadDiag,
};
use crate::matter::dm::clusters::wifi_diag::WirelessDiag;
use crate::matter::dm::networks::NetChangeNotif;
use crate::matter::error::Error;
use crate::matter::error::ErrorCode;
use crate::matter::transport::network::MAX_RX_PACKET_SIZE;
use crate::matter::utils::init::zeroed;
use crate::matter::utils::init::{init, Init};
use crate::matter::utils::storage::Vec;
use crate::stack::persist::{KvBlobStore, SharedKvBlobStore, VENDOR_KEYS_START};

/// Re-export the `openthread` crate
pub mod openthread {
    pub use openthread::*;
}

/// The maximum number of sockets that the Matter stack would use:
/// - One, for the UDP socket used by the Matter protocol
// TODO: Make it configurable with a feature
const OT_MAX_UDP_SOCKETS: usize = 1;

const OT_MAX_SRP_RECORDS: usize = 4;
const OT_SRP_BUF_SZ: usize = 512;

const OT_SETTINGS_BUF_SZ: usize = 1024;

/// A struct that holds all the resources required by the OpenThread stack,
/// as used by Matter.
pub struct OtMatterResources {
    /// The OpenThread main resources
    pub ot: OtResources,
    /// The OpenThread UDP resources
    pub udp: OtUdpResources<OT_MAX_UDP_SOCKETS, MAX_RX_PACKET_SIZE>,
    /// The OpenThread SRP resources
    pub srp: OtSrpResources<OT_MAX_SRP_RECORDS, OT_SRP_BUF_SZ>,
    /// The OpenThread `RamSettings` buffer
    pub settings_buf: [u8; OT_SETTINGS_BUF_SZ],
}

impl OtMatterResources {
    /// Create a new `OtMatterResources` instance
    pub const fn new() -> Self {
        Self {
            ot: OtResources::new(),
            udp: OtUdpResources::new(),
            srp: OtSrpResources::new(),
            settings_buf: [0; OT_SETTINGS_BUF_SZ],
        }
    }

    /// Return an in-place initializer for `OtMatterResources`
    pub fn init() -> impl Init<Self> {
        init!(Self {
            ot: OtResources::new(),
            udp: OtUdpResources::new(),
            srp: OtSrpResources::new(),
            settings_buf <- zeroed(),
        })
    }
}

impl Default for OtMatterResources {
    fn default() -> Self {
        Self::new()
    }
}

/// An implementation of `NetStack` for `openthread`
pub struct OtNetStack<'d>(OpenThread<'d>);

impl<'d> OtNetStack<'d> {
    /// Create a new `OtNetStack` instance
    pub const fn new(ot: OpenThread<'d>) -> Self {
        Self(ot)
    }
}

impl<'d> NetStack for OtNetStack<'d> {
    type UdpBind<'t>
        = &'t OpenThread<'d>
    where
        Self: 't;

    type UdpConnect<'t>
        = NoopNet
    where
        Self: 't;

    type TcpBind<'t>
        = NoopNet
    where
        Self: 't;

    type TcpConnect<'t>
        = NoopNet
    where
        Self: 't;

    type Dns<'t>
        = NoopNet
    where
        Self: 't;

    fn udp_bind(&self) -> Option<Self::UdpBind<'_>> {
        Some(&self.0)
    }

    fn udp_connect(&self) -> Option<Self::UdpConnect<'_>> {
        None
    }

    fn tcp_bind(&self) -> Option<Self::TcpBind<'_>> {
        None
    }

    fn tcp_connect(&self) -> Option<Self::TcpConnect<'_>> {
        None
    }

    fn dns(&self) -> Option<Self::Dns<'_>> {
        None
    }
}

/// A `NetifDiag` anf `NetChangeNotif` traits implementation for `openthread`
pub struct OtNetif<'d>(OpenThread<'d>);

impl<'d> OtNetif<'d> {
    /// Create a new `OtNetif` instance
    pub const fn new(ot: OpenThread<'d>) -> Self {
        Self(ot)
    }
}

impl NetifDiag for OtNetif<'_> {
    fn netifs(&self, f: &mut dyn FnMut(&NetifInfo) -> Result<(), Error>) -> Result<(), Error> {
        let status = self.0.net_status();

        let mut addrs = Vec::<_, 6>::new();

        let _ = self.0.ipv6_addrs(|addr| {
            if let Some((addr, _)) = addr {
                if addrs.len() < addrs.capacity() {
                    unwrap!(addrs.push(addr));
                }
            }

            Ok(())
        }); // TODO

        f(&NetifInfo {
            name: "ot",
            operational: status.ip6_enabled && status.role.is_connected(),
            offprem_svc_reachable_ipv4: None,
            offprem_svc_reachable_ipv6: None,
            hw_addr: &self.0.ieee_eui64(),
            ipv4_addrs: &[],
            ipv6_addrs: &addrs,
            netif_type: InterfaceTypeEnum::Thread,
            netif_index: 0, // Not used with OT
        })?;

        Ok(())
    }
}

impl NetChangeNotif for OtNetif<'_> {
    async fn wait_changed(&self) {
        self.0.wait_changed().await
    }
}

/// A `NetCtl`, `NetChangeNotif`, `WirelessDiag` and `ThreadDiag` traits implementation for `openthread`
pub struct OtNetCtl<'a>(OpenThread<'a>);

impl<'a> OtNetCtl<'a> {
    /// Create a new instance of the `OtNetCtl` type.
    pub fn new(ot: OpenThread<'a>) -> Self {
        Self(ot)
    }
}

impl NetCtl for OtNetCtl<'_> {
    fn net_type(&self) -> NetworkType {
        NetworkType::Thread
    }

    async fn scan<F>(&self, network: Option<&[u8]>, mut f: F) -> Result<(), NetCtlError>
    where
        F: FnMut(&NetworkScanInfo) -> Result<(), Error>,
    {
        const SCAN_DURATION_MILLIS: u16 = 500;

        info!("Thread scan request");

        //let _ = self.0.enable_thread(true);

        self.0
            .scan(Channels::all(), SCAN_DURATION_MILLIS, |scan_result| {
                let Some(scan_result) = scan_result else {
                    debug!("Thread scan reported as complete");
                    return;
                };

                if network
                    .map(|id| id == scan_result.extended_pan_id.to_be_bytes())
                    .unwrap_or(true)
                {
                    let info = NetworkScanInfo::Thread {
                        pan_id: scan_result.pan_id,
                        ext_pan_id: scan_result.extended_pan_id,
                        network_name: scan_result.network_name,
                        channel: scan_result.channel as _,
                        version: scan_result.version,
                        ext_addr: &scan_result.ext_address.to_be_bytes(),
                        rssi: scan_result.rssi,
                        lqi: scan_result.lqi,
                    };

                    info!(
                        "Found Thread network: {:?}",
                        Bytes(&scan_result.extended_pan_id.to_be_bytes())
                    );

                    let _ = f(&info);
                }
            })
            .await
            .map_err(to_net_matter_err)?;

        info!("Thread scan complete");

        Ok(())
    }

    async fn connect(&self, creds: &WirelessCreds<'_>) -> Result<(), NetCtlError> {
        let WirelessCreds::Thread { dataset_tlv } = creds else {
            return Err(NetCtlError::Other(ErrorCode::InvalidAction.into()));
        };

        const TIMEOUT_SECS: u64 = 20;

        let _ = self.0.enable_thread(false);

        // NOTE: Printing the dataset is a security issue, but we do it for now for debugging purposes
        // (i.e. for running some of the pseudo-eth examples the user needs the Thread network dataset)
        info!(
            "Connecting to Thread network, dataset: {:x}",
            Bytes(dataset_tlv)
        );

        self.0
            .set_active_dataset_tlv(dataset_tlv)
            .map_err(to_net_matter_err)?;

        self.0.enable_thread(true).map_err(to_net_matter_err)?;

        let now = Instant::now();

        while !self.0.net_status().role.is_connected() {
            if now.elapsed().as_secs() > TIMEOUT_SECS {
                let _ = self.0.enable_thread(false);

                return Err(NetCtlError::OtherConnectionFailure);
            }

            select(self.0.wait_changed(), Timer::after(Duration::from_secs(1))).await;
        }

        // Enable rx_on_when_idle AFTER Thread attach completes.
        // Root cause: set_link_mode updates radio_conf.rx_when_idle, which
        // changes the radio auto-switching behavior (radio stays in RX after
        // TX/RX instead of going to sleep). If set before attach, the ESP
        // radio driver keeps receiving during the MLE attach handshake,
        // interfering with the attach timing and causing failures.
        if let Err(e) = self.0.set_link_mode(true, false, false) {
            warn!("Failed to set link mode: {:?}", e);
        } else {
            info!("Link mode set: rx_on_when_idle=true");
        }

        Ok(())
    }
}

impl NetChangeNotif for OtNetCtl<'_> {
    async fn wait_changed(&self) {
        self.0.wait_changed().await
    }
}

impl WirelessDiag for OtNetCtl<'_> {
    fn connected(&self) -> Result<bool, Error> {
        let status = self.0.net_status();

        Ok(status.role.is_connected())
    }
}

// TODO
impl ThreadDiag for OtNetCtl<'_> {
    fn channel(&self) -> Result<Option<u16>, Error> {
        Ok(None)
    }

    fn routing_role(&self) -> Result<Option<RoutingRoleEnum>, Error> {
        Ok(None)
    }

    fn network_name(
        &self,
        f: &mut dyn FnMut(Option<&str>) -> Result<(), Error>,
    ) -> Result<(), Error> {
        f(None)
    }

    fn pan_id(&self) -> Result<Option<u16>, Error> {
        Ok(None)
    }

    fn extended_pan_id(&self) -> Result<Option<u64>, Error> {
        let status = self.0.net_status();

        Ok(status.ext_pan_id)
    }

    fn mesh_local_prefix(
        &self,
        f: &mut dyn FnMut(Option<&[u8]>) -> Result<(), Error>,
    ) -> Result<(), Error> {
        f(None)
    }

    fn neighbor_table(
        &self,
        _f: &mut dyn FnMut(&NeighborTable) -> Result<(), Error>,
    ) -> Result<(), Error> {
        Ok(())
    }

    fn route_table(
        &self,
        _f: &mut dyn FnMut(&RouteTable) -> Result<(), Error>,
    ) -> Result<(), Error> {
        Ok(())
    }

    fn partition_id(&self) -> Result<Option<u32>, Error> {
        Ok(None)
    }

    fn weighting(&self) -> Result<Option<u16>, Error> {
        Ok(None)
    }

    fn data_version(&self) -> Result<Option<u16>, Error> {
        Ok(None)
    }

    fn stable_data_version(&self) -> Result<Option<u16>, Error> {
        Ok(None)
    }

    fn leader_router_id(&self) -> Result<Option<u8>, Error> {
        Ok(None)
    }

    fn security_policy(&self) -> Result<Option<SecurityPolicy>, Error> {
        Ok(None)
    }

    fn channel_page0_mask(
        &self,
        f: &mut dyn FnMut(Option<&[u8]>) -> Result<(), Error>,
    ) -> Result<(), Error> {
        f(None)
    }

    fn operational_dataset_components(
        &self,
        f: &mut dyn FnMut(Option<&OperationalDatasetComponents>) -> Result<(), Error>,
    ) -> Result<(), Error> {
        f(None)
    }

    fn active_network_faults_list(
        &self,
        _f: &mut dyn FnMut(NetworkFaultEnum) -> Result<(), Error>,
    ) -> Result<(), Error> {
        Ok(())
    }
}

/// FNV-1a hash implementation for no_std environments.
/// Used to detect changes in mDNS service sets without storing full service data.
struct Fnv1aHasher {
    state: u64,
}

impl Fnv1aHasher {
    const FNV_OFFSET_BASIS: u64 = 0xcbf29ce484222325;
    const FNV_PRIME: u64 = 0x100000001b3;

    const fn new() -> Self {
        Self {
            state: Self::FNV_OFFSET_BASIS,
        }
    }

    fn write(&mut self, bytes: &[u8]) {
        for byte in bytes {
            self.state ^= *byte as u64;
            self.state = self.state.wrapping_mul(Self::FNV_PRIME);
        }
    }

    fn write_u8(&mut self, val: u8) {
        self.write(&[val]);
    }

    fn write_u16(&mut self, val: u16) {
        self.write(&val.to_le_bytes());
    }

    fn write_u64(&mut self, val: u64) {
        self.write(&val.to_le_bytes());
    }

    const fn finish(&self) -> u64 {
        self.state
    }
}

/// Compute a hash of all mDNS services currently published by Matter.
/// Used to detect when services have changed and need to be re-registered with SRP.
fn compute_services_hash(matter: &Matter<'_>) -> u64 {
    let mut hasher = Fnv1aHasher::new();

    // We can't return Result from the callback, so we just hash what we can
    let _ = matter.mdns_services(|service| {
        match service {
            MatterMdnsService::Commissionable { id, discriminator } => {
                hasher.write_u8(0); // Type marker for Commissionable
                hasher.write_u64(id);
                hasher.write_u16(discriminator);
            }
            MatterMdnsService::Commissioned {
                compressed_fabric_id,
                node_id,
            } => {
                hasher.write_u8(1); // Type marker for Commissioned
                hasher.write_u64(compressed_fabric_id);
                hasher.write_u64(node_id);
            }
        }
        Ok(())
    });

    hasher.finish()
}

/// An mDNS trait implementation for `openthread` using Thread SRP
pub struct OtMdns<'d> {
    ot: OpenThread<'d>,
}

impl<'d> OtMdns<'d> {
    /// Create a new `OtMdns` instance
    pub const fn new(ot: OpenThread<'d>) -> Self {
        Self { ot }
    }

    /// Run the `OtMdns` instance by listening to the mDNS services and registering them with the SRP server
    pub async fn run<C: Crypto>(
        &self,
        matter: &Matter<'_>,
        _crypto: C,
        _notify: &dyn ChangeNotify,
    ) -> Result<(), OtError> {
        info!("Running mDNS");

        // Wait for rx_when_idle=true before registering SRP services.
        // Without this, SRP requests are sent but responses cannot be received
        // (causing RESPONSE_TIMEOUT code=28). The netif comes up when device
        // becomes child, but rx_when_idle is set later by OtNetCtl::connect().
        info!("Waiting for rx_when_idle=true...");
        self.ot.wait_rx_when_idle().await;
        info!("rx_when_idle=true, starting SRP registration");

        // On first iteration only: clear stale local SRP state from previous runs.
        // Immediate removal (clear local state without server notification) is used because:
        // 1. The SRP server from a previous session may be unreachable
        // 2. Graceful removal would block waiting for server response during startup
        // 3. The ECDSA key is persisted (via OtPersist), so re-registration under
        //    the same FQDN succeeds — the server validates key ownership
        // 4. Without this, re-adding the device to Apple Home fails: the device
        //    thinks records are registered but the controller sees a timeout
        if !self.ot.srp_is_empty()? {
            if let Err(e) = self.ot.srp_remove_all(true) {
                warn!("SRP startup cleanup failed: {:?}", e);
            } else {
                info!("SRP startup cleanup: cleared stale local state");
            }
        }

        // Track hash of currently registered services to avoid unnecessary re-registration
        let mut current_hash: u64 = 0;

        loop {
            // Compute hash of services Matter wants to publish
            let new_hash = compute_services_hash(matter);

            // Only update SRP registration if services have changed
            if new_hash != current_hash {
                debug!(
                    "SRP services changed: {:016x} -> {:016x}",
                    current_hash, new_hash
                );

                // Clear local SRP state before re-registering changed services.
                // Immediate removal is safe: the persisted ECDSA key ensures the
                // SRP server accepts re-registration under the same FQDN.
                if current_hash != 0 && !self.ot.srp_is_empty()? {
                    if let Err(e) = self.ot.srp_remove_all(true) {
                        warn!("SRP: failed to clear old services: {:?}", e);
                    } else {
                        info!("SRP: cleared old services");
                    }
                    Timer::after(Duration::from_millis(100)).await;
                }

                // Generate hostname from IEEE EUI-64
                let ieee_eui64 = self.ot.ieee_eui64();
                let mut hostname = heapless::String::<16>::new();
                unwrap!(
                    write!(
                        hostname,
                        "{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}",
                        ieee_eui64[0],
                        ieee_eui64[1],
                        ieee_eui64[2],
                        ieee_eui64[3],
                        ieee_eui64[4],
                        ieee_eui64[5],
                        ieee_eui64[6],
                        ieee_eui64[7]
                    ),
                    "Unreachable"
                );

                // Register hostname with SRP
                self.ot.srp_set_conf(&SrpConf {
                    host_name: hostname.as_str(),
                    ..Default::default()
                })?;
                info!("Registered SRP host {}", hostname);

                // Add all current services, tracking failures
                let mut all_ok = true;
                let _ = matter.mdns_services(|matter_service| {
                    Service::call_with(
                        &matter_service,
                        matter.dev_det(),
                        matter.port(),
                        |service| {
                            match self.ot.srp_add_service(&SrpService {
                                name: service.service_protocol,
                                instance_name: service.name,
                                port: service.port,
                                subtype_labels: service.service_subtypes.iter().cloned(),
                                txt_entries: service
                                    .txt_kvs
                                    .iter()
                                    .cloned()
                                    .filter(|(k, _)| !k.is_empty())
                                    .map(|(k, v)| (k, v.as_bytes())),
                                priority: 0,
                                weight: 0,
                                lease_secs: 0,
                                key_lease_secs: 0,
                            }) {
                                Ok(()) => info!("Added service {:?}", matter_service),
                                Err(e) => {
                                    error!(
                                        "Failed to add SRP service {:?}: {:?}",
                                        matter_service, e
                                    );
                                    all_ok = false;
                                }
                            }
                            Ok(())
                        },
                    )
                });

                // Only update hash if all services registered successfully.
                // On partial failure, the next iteration will retry.
                if all_ok {
                    current_hash = new_hash;
                } else {
                    warn!("SRP: partial registration failure, will retry");
                }
            } else {
                debug!("SRP services unchanged, skipping re-registration");
            }

            matter.wait_mdns().await;
        }
    }
}

impl Mdns for OtMdns<'_> {
    async fn run<C, U>(
        &mut self,
        matter: &Matter<'_>,
        crypto: C,
        notify: &dyn ChangeNotify,
        _udp: U,
        _mac: &[u8],
        _ipv4: core::net::Ipv4Addr,
        _ipv6: core::net::Ipv6Addr,
        _interface: u32,
    ) -> Result<(), Error>
    where
        C: Crypto,
        U: UdpBind,
    {
        OtMdns::run(self, matter, crypto, notify)
            .await
            .map_err(to_matter_err)
    }
}

/// The `KvBlobStore` key used to persist OpenThread's SRP ECDSA key
///
/// While persisting other keys is optional, this one _must_ get persisted,
/// or else - upon device restart - it will fail to re-register its SRP services
/// in the SRP server.
const OT_SRP_ECDSA_KEY: u16 = VENDOR_KEYS_START;

/// A struct for implementing persistance of `openthread` settings - volatitle and
/// non-volatile (for selected keys)
pub struct OtPersist<'a, 'd, S> {
    settings: SharedRamSettings<'d, NoopRawMutex, fn(RamSettingsChange) -> bool>,
    store: &'a SharedKvBlobStore<'a, S>,
}

impl<'a, 'd, S> OtPersist<'a, 'd, S>
where
    S: KvBlobStore,
{
    /// Create a new `OtPersist` instance
    ///
    /// # Arguments
    /// - `settings_buf`: A mutable reference to a buffer for storing `openthread` settings before they are persisted
    /// - `store`: A reference to the `KvBlobStore` instance used for persisting a subset of the settings to non-volatile storage
    pub const fn new(settings_buf: &'d mut [u8], store: &'a SharedKvBlobStore<'a, S>) -> Self {
        Self {
            settings: SharedRamSettings::new(RamSettings::new_with_signal_change(
                settings_buf,
                |change| match change {
                    RamSettingsChange::Added { key, .. }
                    | RamSettingsChange::Removed { key, .. }
                        if key == SettingsKey::SrpEcdsaKey as u16 =>
                    {
                        true
                    }
                    RamSettingsChange::Clear => true,
                    _ => false,
                },
            )),
            store,
        }
    }

    /// Return a reference to the `SharedRamSettings` instance to be used with `openthread`
    pub const fn settings(
        &self,
    ) -> &SharedRamSettings<'d, NoopRawMutex, fn(RamSettingsChange) -> bool> {
        &self.settings
    }

    /// Load (a selected subset of) the settings from the `KvBlobStore` non-volatile storage
    pub async fn load(&self) -> Result<(), Error> {
        let (mut kv, mut buf) = self.store.get().await;

        kv.load(OT_SRP_ECDSA_KEY, &mut buf, |data| {
            if let Some(data) = data {
                self.settings.with(|settings| {
                    let mut offset = 0;

                    while offset < data.len() {
                        let key = u16::from_le_bytes([data[offset], data[offset + 1]]);
                        offset += 2;

                        let value = &data[offset..];

                        unwrap!(settings.add(key, value));

                        offset += value.len();
                    }
                })
            }

            Ok(())
        })
        .await
    }

    /// Store (a selected subset of) the settings to the `KvBlobStore` non-volatile storage
    pub async fn store(&self) -> Result<(), Error> {
        let (mut kv, mut buf) = self.store.get().await;

        kv.store(OT_SRP_ECDSA_KEY, &mut buf, |buf| {
            self.settings.with(|settings| {
                let mut offset = 0;

                for (key, value) in settings
                    .iter()
                    .filter(|(key, _)| *key == SettingsKey::SrpEcdsaKey as u16)
                {
                    assert!(value.len() + 2 <= buf.len() - offset);

                    buf[offset..offset + 2].copy_from_slice(&key.to_le_bytes());
                    offset += 2;

                    buf[offset..offset + value.len()].copy_from_slice(value);
                    offset += value.len();
                }

                Ok(offset)
            })
        })
        .await
    }

    /// Run the `OtPersist` instance by waiting for changes in the settings and persisting them
    /// to non-volatile storage
    pub async fn run(&self) -> Result<(), Error> {
        let wait_changed = || poll_fn(|cx| self.settings.poll_changed(cx));

        loop {
            wait_changed().await;

            self.store().await?;
        }
    }
}

pub(crate) fn to_net_matter_err(err: OtError) -> NetCtlError {
    NetCtlError::Other(to_matter_err(err))
}

pub(crate) fn to_matter_err(err: OtError) -> Error {
    error!("OpenThread error: {:?}", err);

    ErrorCode::NoNetworkInterface.into()
}

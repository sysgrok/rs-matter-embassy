//! Network interface: `OtNetif` - a `Netif` trait implementation for `openthread`
//! mDNS impl: `OtMdns` - an mDNS trait implementation for `openthread` using Thread SRP

use core::fmt::Write;
use core::future::poll_fn;
use core::net::IpAddr;

use embassy_futures::select::{select, select3};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Instant, Timer};

use openthread::{
    Channels, DnsResponse, OpenThread, OtError, OtResources, OtSrpResources, OtUdpResources,
    RamSettings, RamSettingsChange, SettingsKey, SharedRamSettings, SrpConf, SrpService,
};

use rs_matter_stack::matter::crypto::Crypto;
use rs_matter_stack::matter::persist::KvBlobStoreAccess;
use rs_matter_stack::matter::Matter;
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
use crate::matter::persist::VENDOR_KEYS_START;
use crate::matter::transport::network::mdns::{DottedName, MdnsRemoteService};
use crate::matter::transport::network::{MatterRemoteService, MAX_RX_PACKET_SIZE};
use crate::matter::utils::init::zeroed;
use crate::matter::utils::init::{init, Init};
use crate::matter::utils::select::Coalesce;
use crate::matter::utils::storage::Vec;
use crate::matter::utils::sync::DynBase;

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
/// Scratch buffer for all three `OtMdns` loops, which run concurrently and so
/// each carve out a disjoint sub-slice (see `OtMdns::run`):
/// - register: [`MDNS_REGISTER_BUF_SZ`] (builds the local service description)
/// - resolve:  host name + TXT
/// - browse:   instance label + host name + TXT
const OT_MDNS_BUF_SZ: usize = MDNS_REGISTER_BUF_SZ + MDNS_RESOLVE_BUF_SZ + MDNS_BROWSE_BUF_SZ;

/// `buf` region for the register (respond) loop.
const MDNS_REGISTER_BUF_SZ: usize = 256;
/// `buf` region for the resolve loop: `host name | TXT`.
const MDNS_RESOLVE_HOST_SZ: usize = 64;
const MDNS_RESOLVE_TXT_SZ: usize = 128;
const MDNS_RESOLVE_BUF_SZ: usize = MDNS_RESOLVE_HOST_SZ + MDNS_RESOLVE_TXT_SZ;
/// `buf` region for the browse loop: `instance label | host name | TXT`.
const MDNS_BROWSE_LABEL_SZ: usize = 64;
const MDNS_BROWSE_HOST_SZ: usize = 64;
const MDNS_BROWSE_TXT_SZ: usize = 128;
const MDNS_BROWSE_BUF_SZ: usize = MDNS_BROWSE_LABEL_SZ + MDNS_BROWSE_HOST_SZ + MDNS_BROWSE_TXT_SZ;

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
    /// A small buffer for mDNS operations
    pub mdns_buf: [u8; OT_MDNS_BUF_SZ],
}

impl OtMatterResources {
    /// Create a new `OtMatterResources` instance
    pub const fn new() -> Self {
        Self {
            ot: OtResources::new(),
            udp: OtUdpResources::new(),
            srp: OtSrpResources::new(),
            settings_buf: [0; OT_SETTINGS_BUF_SZ],
            mdns_buf: [0; OT_MDNS_BUF_SZ],
        }
    }

    /// Return an in-place initializer for `OtMatterResources`
    pub fn init() -> impl Init<Self> {
        init!(Self {
            ot: OtResources::new(),
            udp: OtUdpResources::new(),
            srp: OtSrpResources::new(),
            settings_buf <- zeroed(),
            mdns_buf <- zeroed(),
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

impl DynBase for OtNetif<'_> {}

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

impl DynBase for OtNetCtl<'_> {}

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

/// An mDNS trait implementation for `openthread` using Thread SRP
pub struct OtMdns<'a, 'd> {
    ot: OpenThread<'d>,
    buf: &'a mut [u8],
}

impl<'a, 'd> OtMdns<'a, 'd> {
    /// Create a new `OtMdns` instance
    pub const fn new(ot: OpenThread<'d>, buf: &'a mut [u8]) -> Self {
        Self { ot, buf }
    }

    /// Run the `OtMdns` instance.
    ///
    /// This concurrently drives the three mDNS use cases that Matter needs, all
    /// on top of OpenThread's SRP client + DNS client (which talk to the SRP /
    /// DNS-SD server on the Thread Border Router):
    /// - **respond**: register this node's own Matter service(s) via SRP, so other
    ///   nodes can discover/resolve it;
    /// - **resolve**: resolve a specific remote Matter service (operational or
    ///   commissionable) to an address, on demand;
    /// - **browse**: enumerate commissionable nodes matching a filter, on demand.
    ///
    /// The resolve/browse requests are placed by `rs-matter` into a shared
    /// rendezvous (see `Transport::wait_mdns_resolve_request` /
    /// `wait_mdns_browse_request`); the answers are deposited back via
    /// `try_deposit_mdns_resolve` / `try_deposit_mdns_browse`.
    pub async fn run(&mut self, matter: &Matter<'_>) -> Result<(), OtError> {
        // Split borrows up front so the three loops don't alias `self`: the
        // register loop owns the scratch `buf`, the resolve/browse loops each get
        // a cheap `OpenThread` clone, and each owns a disjoint sub-slice of the
        // single shared `buf` (no per-loop on-stack scratch).
        let (register_buf, rest) = self.buf.split_at_mut(MDNS_REGISTER_BUF_SZ);
        let (resolve_buf, browse_buf) = rest.split_at_mut(MDNS_RESOLVE_BUF_SZ);

        let register = Self::run_register(self.ot.clone(), register_buf, matter);
        let resolve = Self::run_resolve(self.ot.clone(), resolve_buf, matter);
        let browse = Self::run_browse(self.ot.clone(), browse_buf, matter);

        let mut register = core::pin::pin!(register);
        let mut resolve = core::pin::pin!(resolve);
        let mut browse = core::pin::pin!(browse);

        select3(&mut register, &mut resolve, &mut browse)
            .coalesce()
            .await
    }

    /// The **respond** loop: (re-)register this node's own Matter mDNS service(s)
    /// with the SRP server whenever they change.
    async fn run_register(
        ot: OpenThread<'_>,
        buf: &mut [u8],
        matter: &Matter<'_>,
    ) -> Result<(), OtError> {
        loop {
            // TODO: Not very efficient to remove and re-add everything

            let ieee_eui64 = ot.ieee_eui64();

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

            // Reset the SRP client's local registration state before
            // (re-)registering below.
            //
            // We use an *immediate* clear (`otSrpClientClearHostAndServices`)
            // rather than a graceful, server-side removal. A graceful removal
            // (`srp_remove_all(false)`) schedules an unregister update to the SRP
            // server and only completes once the server acks the host into the
            // `REMOVED` state. But right after (re-)attaching, the host we just
            // configured was never actually registered with the server this run,
            // so that ack never arrives and `srp_is_empty()` would spin forever.
            // An immediate clear resets the local client state synchronously
            // (host name + service slots), which is all we need: re-registering
            // with the same (persisted) ECDSA key overwrites any record a prior
            // boot left on the server, and stale server records otherwise expire
            // by lease.
            ot.srp_remove_all(true)?;

            debug_assert!(ot.srp_is_empty()?);

            ot.srp_set_conf(&SrpConf {
                host_name: hostname.as_str(),
                ..Default::default()
            })?;

            info!("Registered SRP host {}", hostname);

            let buf = &mut *buf;

            unwrap!(matter.mdns_services(|matter_service| {
                let (service, _) = matter_service.service(matter.dev_det(), matter.port(), buf)?;
                let service = core::mem::ManuallyDrop::new(service);

                // TODO:
                // Remove `ManuallyDrop` by removing the `'a` lifetime from the signature of the function below:
                // pub fn srp_add_service<'a, SI, TI>(&self, service: &'a SrpService<'a, SI, TI>)
                //                                                      ^- remove this lifetime

                let srp_service = core::mem::ManuallyDrop::new(SrpService {
                    name: service.service_protocol,
                    instance_name: service.name,
                    port: service.port,
                    subtype_labels: service.service_subtypes.clone(),
                    txt_entries: service.txt_kvs.clone().map(|(k, v)| (k, v.as_bytes())),
                    priority: 0,
                    weight: 0,
                    lease_secs: 0,
                    key_lease_secs: 0,
                });

                unwrap!(ot.srp_add_service(&srp_service)); // TODO

                info!("Added service {:?}", matter_service);

                Ok(())
            }));

            matter.transport().wait_mdns().await;
        }
    }

    /// The **resolve** loop: wait for `rs-matter` to request resolution of a
    /// specific remote Matter service, resolve it via the OpenThread DNS client,
    /// and deposit the answer back.
    async fn run_resolve(
        ot: OpenThread<'_>,
        buf: &mut [u8],
        matter: &Matter<'_>,
    ) -> Result<(), OtError> {
        // Partition the resolve region into `host name | TXT` scratch.
        let (host_buf, txt_buf) = buf.split_at_mut(MDNS_RESOLVE_HOST_SZ);

        loop {
            let service = matter.transport().wait_mdns_resolve_request().await;

            // The instance label and DNS-SD service type to query. OpenThread's
            // DNS client is a unicast resolver against the BR's DNS-SD server,
            // which serves the Thread domain `default.service.arpa` (NOT mDNS
            // `local`).
            let mut label = heapless::String::<33>::new();
            let service_name = match service {
                MatterRemoteService::Operational {
                    compressed_fabric_id,
                    node_id,
                } => {
                    let _ = write!(label, "{compressed_fabric_id:016X}-{node_id:016X}");
                    "_matter._tcp.default.service.arpa"
                }
                MatterRemoteService::Commissionable { id } => {
                    let _ = write!(label, "{id:016X}");
                    "_matterc._udp.default.service.arpa"
                }
            };

            // The deposited instance name must match what `rs-matter` expects,
            // i.e. the mDNS `.local` form (see `MatterRemoteService::instance_name`),
            // NOT the `default.service.arpa` name we actually queried.
            let mut instance_name = heapless::String::<128>::new();
            service.instance_name(&mut instance_name);

            let result = ot
                .dns_resolve_service_and_host_address(
                    label.as_str(),
                    service_name,
                    None,
                    |response| {
                        let DnsResponse::Service(response) = response else {
                            return;
                        };

                        let Ok(info) = response.service_info(&mut *host_buf, &mut *txt_buf) else {
                            return;
                        };

                        let Some(addr) = info.host_address else {
                            // No address resolved; nothing useful to deposit.
                            return;
                        };

                        matter
                            .transport()
                            .try_deposit_mdns_resolve(&MdnsRemoteService {
                                instance_name: DottedName(instance_name.as_str()),
                                port: Some(info.port),
                                addrs: core::iter::once(IpAddr::V6(addr)),
                                txt: TxtEntries::new(info.txt_data.unwrap_or(&[])),
                                scope_id: 0,
                            });
                    },
                )
                .await;

            if let Err(e) = result {
                warn!("mDNS resolve query failed: {:?}", e);
            }
        }
    }

    /// The **browse** loop: wait for `rs-matter` to request a commissionable
    /// browse, enumerate matching nodes via the OpenThread DNS client, and
    /// deposit the answers back.
    async fn run_browse(
        ot: OpenThread<'_>,
        buf: &mut [u8],
        matter: &Matter<'_>,
    ) -> Result<(), OtError> {
        // Partition the browse region into `instance label | host name | TXT`.
        let (label_buf, rest) = buf.split_at_mut(MDNS_BROWSE_LABEL_SZ);
        let (host_buf, txt_buf) = rest.split_at_mut(MDNS_BROWSE_HOST_SZ);

        loop {
            let filter = matter.transport().wait_mdns_browse_request().await;

            // Build the (sub)type browse name in OpenThread's DNS-SD domain.
            // `CommissionableFilter::service_type(.., false)` yields e.g.
            // `_matterc._udp` or `_L1234._sub._matterc._udp`; append the domain.
            let mut service_type = heapless::String::<64>::new();
            filter.service_type(&mut service_type, false);
            let mut service_name = heapless::String::<96>::new();
            let _ = write!(service_name, "{service_type}.default.service.arpa");

            let result = ot
                .dns_browse(service_name.as_str(), None, |response| {
                    let DnsResponse::Browse(response) = response else {
                        return;
                    };

                    let mut index = 0;
                    loop {
                        let label = match response.service_instance(index, &mut *label_buf) {
                            Ok(Some(label)) => label,
                            _ => break,
                        };

                        index += 1;

                        let Ok(info) = response.service_info(label, &mut *host_buf, &mut *txt_buf)
                        else {
                            continue;
                        };

                        let Some(addr) = info.host_address else {
                            continue;
                        };

                        // The deposited instance name only needs a valid leading
                        // hex-id label + the commissionable service-type suffix in
                        // the `.local` form (the browse deposit matches on the id
                        // label and the TXT records, not the domain).
                        let mut instance_name = heapless::String::<96>::new();
                        let _ = write!(instance_name, "{label}._matterc._udp.local");

                        matter
                            .transport()
                            .try_deposit_mdns_browse(&MdnsRemoteService {
                                instance_name: DottedName(instance_name.as_str()),
                                port: Some(info.port),
                                addrs: core::iter::once(IpAddr::V6(addr)),
                                txt: TxtEntries::new(info.txt_data.unwrap_or(&[])),
                                scope_id: 0,
                            });
                    }
                })
                .await;

            if let Err(e) = result {
                warn!("mDNS browse query failed: {:?}", e);
            }
        }
    }
}

/// An iterator over DNS-SD TXT record data (a sequence of length-prefixed
/// `key` or `key=value` entries) yielding `(key, value)` string pairs.
///
/// Entries that are not valid UTF-8 are skipped. An entry with no `=` yields an
/// empty value. This is the parse-side counterpart to how Matter TXT records are
/// published, and feeds `rs-matter`'s `MdnsRemoteService` TXT consumers.
#[derive(Clone)]
struct TxtEntries<'a> {
    data: &'a [u8],
}

impl<'a> TxtEntries<'a> {
    const fn new(data: &'a [u8]) -> Self {
        Self { data }
    }
}

impl<'a> Iterator for TxtEntries<'a> {
    type Item = (&'a str, &'a str);

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.data.is_empty() {
                return None;
            }

            let len = self.data[0] as usize;
            let rest = &self.data[1..];

            if len > rest.len() {
                // Malformed: length runs past the buffer. Stop iterating.
                self.data = &[];
                return None;
            }

            let (entry, next) = rest.split_at(len);
            self.data = next;

            let Ok(entry) = core::str::from_utf8(entry) else {
                // Skip non-UTF-8 entries.
                continue;
            };

            let (key, value) = match entry.split_once('=') {
                Some((k, v)) => (k, v),
                None => (entry, ""),
            };

            return Some((key, value));
        }
    }
}

impl Mdns for OtMdns<'_, '_> {
    async fn run<C, U>(
        &mut self,
        matter: &Matter<'_>,
        _crypto: C,
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
        OtMdns::run(self, matter).await.map_err(to_matter_err)
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
pub struct OtPersist<'d, K> {
    settings: SharedRamSettings<'d, NoopRawMutex, fn(RamSettingsChange) -> bool>,
    kv: K,
}

impl<'d, K> OtPersist<'d, K>
where
    K: KvBlobStoreAccess,
{
    /// Create a new `OtPersist` instance
    ///
    /// # Arguments
    /// - `settings_buf`: A mutable reference to a buffer for storing `openthread` settings before they are persisted
    /// - `kv`: A key-value blob store access used for persisting a subset of the settings to non-volatile storage
    pub const fn new(settings_buf: &'d mut [u8], kv: K) -> Self {
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
            kv,
        }
    }

    /// Return a reference to the `SharedRamSettings` instance to be used with `openthread`
    pub const fn settings(
        &self,
    ) -> &SharedRamSettings<'d, NoopRawMutex, fn(RamSettingsChange) -> bool> {
        &self.settings
    }

    /// Load (a selected subset of) the settings from the `KvBlobStore` non-volatile storage
    pub fn load(&self) -> Result<(), Error> {
        self.kv.access(|kv, buf| {
            if let Some(data) = kv.load(OT_SRP_ECDSA_KEY, buf)? {
                self.settings.with(|settings| {
                    let mut offset = 0;

                    while offset < data.len() {
                        let key = u16::from_le_bytes([data[offset], data[offset + 1]]);
                        offset += 2;

                        let value = &data[offset..];

                        unwrap!(settings.add(key, value));

                        offset += value.len();
                    }
                });
            }

            Ok(())
        })
    }

    /// Store (a selected subset of) the settings to the `KvBlobStore` non-volatile storage
    pub fn store(&self) -> Result<(), Error> {
        self.kv.access(|kv, buf| {
            let offset = self.settings.with(|settings| {
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

                offset
            });

            let (data, buf) = buf.split_at_mut(offset);

            kv.store(OT_SRP_ECDSA_KEY, data, buf)
        })
    }

    /// Run the `OtPersist` instance by waiting for changes in the settings and persisting them
    /// to non-volatile storage
    pub async fn run(&self) -> Result<(), Error> {
        let wait_changed = || poll_fn(|cx| self.settings.poll_changed(cx));

        loop {
            wait_changed().await;

            self.store()?;
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

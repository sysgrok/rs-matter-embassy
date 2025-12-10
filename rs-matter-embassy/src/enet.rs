//! Network interface: `EnetNetif` - a `Netif` trait implementation for `embassy-net`
//! UDP: A `UdpBind` trait implementation for `embassy-net`

use core::cell::Cell;
use core::net::Ipv6Addr;

use embassy_futures::select::select;
use embassy_net::driver::{Driver, HardwareAddress as DriverHardwareAddress};
use embassy_net::{
    Config, ConfigV6, HardwareAddress, Ipv6Cidr, Runner, Stack, StackResources, StaticConfigV6,
};
use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};
use embassy_time::{Duration, Timer};

use crate::matter::dm::clusters::gen_diag::{InterfaceTypeEnum, NetifDiag, NetifInfo};
use crate::matter::dm::networks::NetChangeNotif;
use crate::matter::error::Error;
use crate::matter::transport::network::{MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE};
use crate::stack::nal::noop::NoopNet;
use crate::stack::nal::NetStack;

/// Re-export the `edge-nal-embassy` crate
pub use edge_nal_embassy::*;

/// Re-export the `embassy-net` crate
pub mod net {
    pub use ::embassy_net::*;
}

/// The minimum number of sockets that should be configured in the `embassy-net` `StackResources`:
/// The two UDP sockets used by the Matter stack, plus extra 2 for DHCP + DNS
// TODO: Make it configurable with a feature
pub const ENET_MIN_SOCKET_SET: usize = ENET_MAX_UDP_SOCKETS + 2;

/// A type alias for the `UdpBuffers` type configured with the minimum number of UDP socket buffers
/// sufficient for the operation of the Matter stack
pub type EnetMatterUdpBuffers = UdpBuffers<
    ENET_MAX_UDP_SOCKETS,
    MAX_TX_PACKET_SIZE,
    MAX_RX_PACKET_SIZE,
    ENET_MAX_UDP_META_DATA,
>;

/// A type alias for the `StackResources` type configured with the minimum number of sockets
/// sufficient for the operation of the Matter stack
pub type EnetMatterStackResources = StackResources<ENET_MIN_SOCKET_SET>;

/// The maximum number of sockets that the Matter stack would use:
/// - One, for the UDP socket used by the Matter protocol
/// - Another, for the UDP socket used by the mDNS responder
// TODO: Make it configurable with a feature
const ENET_MAX_UDP_SOCKETS: usize = 2;
/// The max number of meta data buffers that the Matter stack would use
const ENET_MAX_UDP_META_DATA: usize = 4;

/// The MAC address used for mDNS multicast queries over IPv4
///
/// Useful with wifi stack implementations (i.e. cyw43) that require explicit
/// allowlisting of the multicast MAC addresses they should be listening on.
pub const MDNS_MULTICAST_MAC_IPV4: [u8; 6] = [0x01, 0x00, 0x5e, 0x00, 0x00, 0xfb];

/// The MAC address used for mDNS multicast queries over IPv6
///
/// Useful with wifi stack implementations (i.e. cyw43) that require explicit
/// allowlisting of the multicast MAC addresses they should be listening on.
pub const MDNS_MULTICAST_MAC_IPV6: [u8; 6] = [0x33, 0x33, 0x00, 0x00, 0x00, 0xfb];

const TIMEOUT_PERIOD_SECS: u8 = 5;

/// A `Netif` trait implementation for `embassy-net`
pub struct EnetNetif<'d> {
    stack: Stack<'d>,
    up: Mutex<CriticalSectionRawMutex, Cell<bool>>,
    netif_type: InterfaceTypeEnum,
}

impl<'d> EnetNetif<'d> {
    /// Create a new `EmbassyNetif` instance
    pub const fn new(stack: Stack<'d>, netif_type: InterfaceTypeEnum) -> Self {
        Self {
            stack,
            up: Mutex::new(Cell::new(false)),
            netif_type,
        }
    }
}

impl NetifDiag for EnetNetif<'_> {
    fn netifs(&self, f: &mut dyn FnMut(&NetifInfo) -> Result<(), Error>) -> Result<(), Error> {
        let v4 = self.stack.config_v4();
        let v6 = self.stack.config_v6();

        let mut hw_addr = [0; 8];
        match self.stack.hardware_address() {
            HardwareAddress::Ethernet(addr) => hw_addr[..6].copy_from_slice(&addr.0),
            #[allow(unreachable_patterns)]
            _ => (),
        }

        f(&NetifInfo {
            name: "enet",
            operational: self.stack.is_link_up(),
            offprem_svc_reachable_ipv4: None,
            offprem_svc_reachable_ipv6: None,
            hw_addr: &hw_addr,
            ipv4_addrs: if let Some(addrs) = v4.map(|v4| [v4.address.address()]).as_ref() {
                addrs.as_slice()
            } else {
                &[]
            },
            ipv6_addrs: if let Some(addrs) = v6.map(|v6| [v6.address.address()]).as_ref() {
                addrs.as_slice()
            } else {
                &[]
            },
            netif_type: self.netif_type,
            netif_index: 0, // Not used on baremetal
        })?;

        Ok(())
    }
}

impl NetChangeNotif for EnetNetif<'_> {
    async fn wait_changed(&self) {
        // Embassy does have a `wait_config_up/down` but no `wait_config_change`
        // Use a timer as a workaround

        if self.up.lock(|up| up.get()) {
            select(
                self.stack.wait_config_down(),
                Timer::after(Duration::from_secs(TIMEOUT_PERIOD_SECS as _)),
            )
            .await;
        } else {
            select(
                self.stack.wait_config_up(),
                Timer::after(Duration::from_secs(TIMEOUT_PERIOD_SECS as _)),
            )
            .await;
        }

        self.up.lock(|up| up.set(self.stack.is_config_up()));
    }
}

/// Create an `embassy-net` stack suitable for the `rs-matter` stack
pub fn create_enet_stack<const N: usize, D: Driver>(
    driver: D,
    seed: u64,
    resources: &mut StackResources<N>,
) -> (Stack<'_>, Runner<'_, D>) {
    let config = create_enet_config(&driver);

    net::new(driver, config, resources, seed)
}

/// Create a `Config` instance suitable for the `rs-matter` stack:
/// - Ipv6 enabled with a static configuration that uses the link-local address derived from the MAC address
/// - Ipv4 enabled with DHCPv4; structly speaking this is not necessary for the Matter stack, but it is
///   useful in that the `rs-matter` mDNS responder would also answer ipv4 queries
pub fn create_enet_config<D: Driver>(driver: &D) -> Config {
    let DriverHardwareAddress::Ethernet(mac) = driver.hardware_address() else {
        unreachable!();
    };

    let mut config = Config::dhcpv4(Default::default());
    config.ipv6 = ConfigV6::Static(StaticConfigV6 {
        address: Ipv6Cidr::new(create_link_local_ipv6(&mac), 10),
        gateway: None,
        dns_servers: heapless::Vec::new(),
    });

    config
}

/// Create a link-local IPv6 address from a MAC address.
pub fn create_link_local_ipv6(mac: &[u8; 6]) -> Ipv6Addr {
    Ipv6Addr::new(
        0xfe80,
        0,
        0,
        0,
        u16::from_be_bytes([mac[0] ^ 0x02, mac[1]]),
        u16::from_be_bytes([mac[2], 0xff]),
        u16::from_be_bytes([0xfe, mac[3]]),
        u16::from_be_bytes([mac[4], mac[5]]),
    )
}

/// Get the multicast MAC address corresponding to the given IPv6 link-local address.
///
/// Useful with Wifi stack implementations (i.e. cyw43) that require explicit
/// allowlisting of the multicast MAC addresses they should be listening on.
///
/// Note that the provided IP should be a link-local IP (fe80::/10) or else this
/// function would return a bogus result.
pub fn multicast_mac_for_link_local_ipv6(ip: &Ipv6Addr) -> [u8; 6] {
    let mut mac = [0x33, 0x33, 0xff, 0, 0, 0];
    mac[3..].copy_from_slice(&ip.octets()[13..]);

    mac
}

/// An implementation of `NetStack` for the `embassy-net` stack
// TODO: Implement optional TCP support with a feature flag
#[derive(Copy, Clone)]
pub struct EnetStack<'a> {
    udp: Udp<'a>,
    dns: Dns<'a>,
}

impl<'a> EnetStack<'a> {
    /// Create a new `EnetStack` instance
    pub fn new(stack: Stack<'a>, buffers: &'a EnetMatterUdpBuffers) -> Self {
        Self {
            udp: Udp::new(stack, buffers),
            dns: Dns::new(stack),
        }
    }
}

impl<'a> NetStack for EnetStack<'a> {
    type UdpBind<'t>
        = Udp<'t>
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
        = Dns<'t>
    where
        Self: 't;

    fn udp_bind(&self) -> Option<Self::UdpBind<'_>> {
        Some(self.udp)
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
        Some(self.dns)
    }
}

#[cfg(test)]
mod test {
    #[test]
    fn test() {
        assert_eq!(
            super::create_link_local_ipv6(&[0x52, 0x74, 0xf2, 0xb1, 0xa8, 0x7f]).octets(),
            [0xfe, 0x80, 0, 0, 0, 0, 0, 0, 0x50, 0x74, 0xf2, 0xff, 0xfe, 0xb1, 0xa8, 0x7f]
        );
    }
}

//! Network interface: `EnetNetif` - a `Netif` trait implementation for `embassy-net`
//! UDP: `EnetStack` - a `NetStack` trait implementation for `embassy-net`

use core::cell::{Cell, UnsafeCell};
use core::future::{ready, Future};
use core::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr};

use cfg_if::cfg_if;

use embassy_net::driver::{Driver, HardwareAddress as DriverHardwareAddress};
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{
    Config, ConfigV6, HardwareAddress, IpEndpoint, IpListenEndpoint, Ipv6Cidr, Runner, Stack,
    StackResources, StaticConfigV6,
};

use crate::error::to_net_error;
use crate::matter::dm::clusters::gen_diag::{InterfaceTypeEnum, NetifDiag, NetifInfo};
use crate::matter::dm::networks::NetChangeNotif;
use crate::matter::error::{Error, ErrorCode};
use crate::matter::transport::network::{
    Address, NetworkMulticast, NetworkReceive, NetworkSend, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE,
};
use crate::matter::utils::futures::select;
use crate::matter::utils::init::{init, init_array_from_fn, zeroed, Init, UnsafeCellInit};
use crate::matter::utils::sync::blocking::Mutex;
use crate::matter::utils::timers::{Duration, Timer};
use crate::stack::net::{NetSocket, NetStack};

/// Re-export the `embassy-net` crate
pub mod net {
    pub use ::embassy_net::*;
}

/// The minimum number of sockets that should be configured in the `embassy-net` `StackResources`:
/// The UDP sockets used by the Matter stack, plus two sockets for DHCP + DNS, plus optional
/// additional application sockets.
pub const ENET_MIN_SOCKET_SET: usize =
    ENET_MAX_UDP_SOCKETS + ENET_REQUIRED_EXTRA_SOCKET_SET + ENET_EXTRA_SOCKET_SET;

const ENET_REQUIRED_EXTRA_SOCKET_SET: usize = 2;

cfg_if! {
    if #[cfg(feature = "enet-extra-sockets-8")] {
        const ENET_EXTRA_SOCKET_SET: usize = 8;
    } else if #[cfg(feature = "enet-extra-sockets-4")] {
        const ENET_EXTRA_SOCKET_SET: usize = 4;
    } else if #[cfg(feature = "enet-extra-sockets-2")] {
        const ENET_EXTRA_SOCKET_SET: usize = 2;
    } else if #[cfg(feature = "enet-extra-sockets-1")] {
        const ENET_EXTRA_SOCKET_SET: usize = 1;
    } else {
        const ENET_EXTRA_SOCKET_SET: usize = 0;
    }
}

/// The buffers of the UDP sockets that the Matter stack binds with `EnetStack`
pub struct EnetMatterUdpBuffers {
    taken: Mutex<Cell<[bool; ENET_MAX_UDP_SOCKETS]>>,
    slots: [UnsafeCell<EnetUdpSlot>; ENET_MAX_UDP_SOCKETS],
}

impl EnetMatterUdpBuffers {
    #[allow(clippy::declare_interior_mutable_const)]
    const SLOT: UnsafeCell<EnetUdpSlot> = UnsafeCell::new(EnetUdpSlot::new());

    /// Create a new `EnetMatterUdpBuffers` instance
    pub const fn new() -> Self {
        Self {
            taken: Mutex::new(Cell::new([false; ENET_MAX_UDP_SOCKETS])),
            slots: [Self::SLOT; ENET_MAX_UDP_SOCKETS],
        }
    }

    /// Return an in-place initializer for `EnetMatterUdpBuffers`
    pub fn init() -> impl Init<Self> {
        init!(Self {
            taken: Mutex::new(Cell::new([false; ENET_MAX_UDP_SOCKETS])),
            slots <- init_array_from_fn(|_| UnsafeCell::init(EnetUdpSlot::init())),
        })
    }

    /// Take a free slot, which stays taken until the returned `EnetUdpSlotRef` is dropped.
    #[allow(clippy::mut_from_ref)]
    fn take(&self) -> Option<(EnetUdpSlotRef<'_>, &mut EnetUdpSlot)> {
        let index = self.taken.lock(|taken| {
            let mut slots = taken.get();
            let index = slots.iter().position(|taken| !taken)?;

            slots[index] = true;
            taken.set(slots);

            Some(index)
        })?;

        // SAFETY: The slot is marked as taken, so no other reference to it exists
        // until `EnetUdpSlotRef` gives it back.
        let slot = unsafe { &mut *self.slots[index].get() };

        Some((
            EnetUdpSlotRef {
                buffers: self,
                index,
            },
            slot,
        ))
    }
}

impl Default for EnetMatterUdpBuffers {
    fn default() -> Self {
        Self::new()
    }
}

/// The buffers of one UDP socket
struct EnetUdpSlot {
    rx_meta: [PacketMetadata; ENET_MAX_UDP_META_DATA],
    rx: [u8; MAX_RX_PACKET_SIZE],
    tx_meta: [PacketMetadata; ENET_MAX_UDP_META_DATA],
    tx: [u8; MAX_TX_PACKET_SIZE],
}

impl EnetUdpSlot {
    const fn new() -> Self {
        Self {
            rx_meta: [PacketMetadata::EMPTY; ENET_MAX_UDP_META_DATA],
            rx: [0; MAX_RX_PACKET_SIZE],
            tx_meta: [PacketMetadata::EMPTY; ENET_MAX_UDP_META_DATA],
            tx: [0; MAX_TX_PACKET_SIZE],
        }
    }

    fn init() -> impl Init<Self> {
        init!(Self {
            rx_meta: [PacketMetadata::EMPTY; ENET_MAX_UDP_META_DATA],
            rx <- zeroed(),
            tx_meta: [PacketMetadata::EMPTY; ENET_MAX_UDP_META_DATA],
            tx <- zeroed(),
        })
    }
}

/// Gives a taken slot of `EnetMatterUdpBuffers` back when dropped.
struct EnetUdpSlotRef<'a> {
    buffers: &'a EnetMatterUdpBuffers,
    index: usize,
}

impl Drop for EnetUdpSlotRef<'_> {
    fn drop(&mut self) {
        self.buffers.taken.lock(|taken| {
            let mut slots = taken.get();
            slots[self.index] = false;
            taken.set(slots);
        });
    }
}

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
    up: Mutex<Cell<bool>>,
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
        #[allow(clippy::single_match)]
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
        dns_servers: Default::default(),
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
#[derive(Copy, Clone)]
pub struct EnetStack<'a> {
    stack: Stack<'a>,
    buffers: &'a EnetMatterUdpBuffers,
}

impl<'a> EnetStack<'a> {
    /// Create a new `EnetStack` instance
    pub const fn new(stack: Stack<'a>, buffers: &'a EnetMatterUdpBuffers) -> Self {
        Self { stack, buffers }
    }

    fn bind_socket(&self, addr: &SocketAddr) -> Result<EnetSocket<'a>, Error> {
        let (slot_ref, slot) = self.buffers.take().ok_or(ErrorCode::ResourceExhausted)?;

        let mut socket = UdpSocket::new(
            self.stack,
            &mut slot.rx_meta,
            &mut slot.rx,
            &mut slot.tx_meta,
            &mut slot.tx,
        );

        let ip = addr.ip();
        socket
            .bind(IpListenEndpoint {
                addr: (!ip.is_unspecified()).then(|| ip.into()),
                port: addr.port(),
            })
            .map_err(to_net_error)?;

        Ok(EnetSocket {
            socket,
            stack: self.stack,
            _slot_ref: slot_ref,
        })
    }
}

impl<'a> NetStack for EnetStack<'a> {
    type Socket<'t>
        = EnetSocket<'a>
    where
        Self: 't;

    fn bind(
        &self,
        addr: SocketAddr,
        _ipv4_interface: Ipv4Addr,
        _ipv6_interface: u32,
    ) -> impl Future<Output = Result<Self::Socket<'_>, Error>> {
        ready(self.bind_socket(&addr))
    }
}

/// The UDP socket of `EnetStack`
pub struct EnetSocket<'a> {
    socket: UdpSocket<'a>,
    stack: Stack<'a>,
    // Dropped after `socket`, so its buffers are given back only once the socket is gone
    _slot_ref: EnetUdpSlotRef<'a>,
}

impl NetSocket for EnetSocket<'_> {
    type Send<'a>
        = &'a Self
    where
        Self: 'a;

    type Receive<'a>
        = &'a Self
    where
        Self: 'a;

    type Multicast<'a>
        = &'a Self
    where
        Self: 'a;

    fn split(&mut self) -> (Self::Send<'_>, Self::Receive<'_>, Self::Multicast<'_>) {
        (&*self, &*self, &*self)
    }
}

impl NetworkSend for &EnetSocket<'_> {
    async fn send_to(&mut self, data: &[u8], addr: Address) -> Result<(), Error> {
        let addr = addr.udp().ok_or(ErrorCode::NoNetworkInterface)?;

        self.socket
            .send_to(data, IpEndpoint::from(addr))
            .await
            .map_err(to_net_error)
    }
}

impl NetworkReceive for &EnetSocket<'_> {
    async fn wait_available(&mut self) -> Result<(), Error> {
        self.socket.wait_recv_ready().await;

        Ok(())
    }

    async fn recv_from(&mut self, buffer: &mut [u8]) -> Result<(usize, Address), Error> {
        let (len, meta) = self.socket.recv_from(buffer).await.map_err(to_net_error)?;

        Ok((len, Address::Udp(meta.endpoint.into())))
    }
}

impl NetworkMulticast for &EnetSocket<'_> {
    fn join(&mut self, addr: IpAddr) -> impl Future<Output = Result<(), Error>> {
        ready(self.stack.join_multicast_group(addr).map_err(to_net_error))
    }

    fn leave(&mut self, addr: IpAddr) -> impl Future<Output = Result<(), Error>> {
        ready(self.stack.leave_multicast_group(addr).map_err(to_net_error))
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

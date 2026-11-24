//! Network interface: `EnetNetif` - a `Netif` trait implementation for `embassy-net`
//! UDP: `EnetStack` - a `NetStack` trait implementation for `embassy-net`

use core::cell::Cell;
#[cfg(feature = "tcp")]
use core::cell::UnsafeCell;
use core::future::{ready, Future};
use core::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr};
use core::task::{Context, Waker};

use embassy_net::config::IFACE_ADDR_COUNT;
use embassy_net::driver::{
    Capabilities, Driver, DriverBuf, HardwareAddress as DriverHardwareAddress, LinkState,
    NotSupported, RxWanted,
};
use embassy_net::iface::Iface;
#[cfg(feature = "tcp")]
use embassy_net::tcp::{TcpListener, TcpReader, TcpSocket, TcpWriter};
use embassy_net::udp::UdpSocket;
use embassy_net::wire::{HardwareAddress, IpAddr as EnetIpAddr, ListenSocketAddr};
use embassy_net::{Pool, Runner, Stack, StackStorage};

use crate::error::to_net_error;
use crate::matter::dm::clusters::gen_diag::{InterfaceTypeEnum, NetifDiag, NetifInfo};
use crate::matter::dm::networks::NetChangeNotif;
use crate::matter::error::{Error, ErrorCode};
#[cfg(feature = "tcp")]
use crate::matter::transport::network::tcp::FRAME_HDR_LEN;
use crate::matter::transport::network::{Address, NetworkMulticast, NetworkReceive, NetworkSend};
#[cfg(feature = "tcp")]
use crate::matter::transport::MAX_RX_BUF_SIZE;
use crate::matter::utils::futures::select;
#[cfg(feature = "tcp")]
use crate::matter::utils::init::{init, init_array_from_fn, zeroed, Init, UnsafeCellInit};
use crate::matter::utils::storage::Vec;
use crate::matter::utils::sync::blocking::Mutex;
#[cfg(feature = "tcp")]
use crate::matter::utils::sync::IfMutex;
use crate::matter::utils::timers::{Duration, Timer};
#[cfg(feature = "tcp")]
use crate::stack::net::NetTcpStack;
use crate::stack::net::{NetSocket, NetStack};
#[cfg(feature = "tcp")]
use crate::stack::tcp::{
    TcpConfig, TcpKeepAlive, TcpNetwork, TcpNetworkBuffers, TcpNetworkRead, TcpNetworkSocket,
    TcpNetworkSockets, TcpNetworkWrite,
};

/// Re-export the `embassy-net` crate
pub mod net {
    pub use ::embassy_net::*;
}

/// The maximum number of Matter-over-TCP sockets (`SLOTS = 1` active + `1` accepted while the
/// active one is still there, so that a new connection can evict it).
#[cfg(feature = "tcp")]
const ENET_MAX_TCP_SOCKETS: usize = 2;

/// The storage of the `embassy-net` stack that `EnetStack` runs over.
///
/// The stack is created again over it every time the Matter stack runs the network.
pub struct EnetStackStorage(StackStorage<'static>);

impl EnetStackStorage {
    /// Create a new `EnetStackStorage` instance
    pub const fn new() -> Self {
        Self(StackStorage::new())
    }

    fn get(&mut self) -> &mut StackStorage<'_> {
        // SAFETY: `StackStorage` is a `MaybeUninit` which `Stack::new` writes without reading
        // what was there, and never drops. So the stack of an earlier run, whose borrows have
        // ended, is never touched again, and the storage can hold one of any lifetime.
        unsafe {
            core::mem::transmute::<&mut StackStorage<'static>, &mut StackStorage<'_>>(&mut self.0)
        }
    }
}

impl Default for EnetStackStorage {
    fn default() -> Self {
        Self::new()
    }
}

/// The packet pool of the `embassy-net` stacks that `EnetStack` runs over: every packet buffer
/// of the stack, its sockets and its driver comes from it.
///
/// `P` is usually a `StaticPool` in a `ConstStaticCell`. Its buffer size has to fit the largest
/// frame of the driver (1514 bytes for Ethernet and Wifi) plus the headroom the driver needs,
/// and some drivers (e.g. `cyw43`) also need a particular size and alignment.
pub struct EnetPool<P: 'static>(&'static mut P);

impl<P> EnetPool<P>
where
    P: Pool,
{
    /// Create a new `EnetPool` instance
    pub const fn new(pool: &'static mut P) -> Self {
        Self(pool)
    }

    fn get(&mut self) -> &'static mut P {
        // SAFETY: `embassy-net` takes the pool by `&'static mut` so that no two stacks allocate
        // from it at the same time, and the packet buffers it hands out carry no lifetime.
        // `create_enet_stack` borrows the `EnetPool` for as long as the stack it creates lives,
        // so these stacks exist one after the other, and the pool itself does live forever.
        unsafe { &mut *core::ptr::from_mut(self.0) }
    }
}

const TIMEOUT_PERIOD_SECS: u8 = 5;

/// Create an `embassy-net` stack suitable for the `rs-matter` stack, with `driver` as its only
/// interface:
/// - IPv6 with the link-local address derived from the MAC address, which `embassy-net`
///   assigns by itself
/// - IPv4 with DHCPv4; strictly speaking this is not necessary for the Matter stack, but it is
///   useful in that the `rs-matter` mDNS responder would also answer IPv4 queries
///
/// Returns the interface, and the runner of the stack.
pub fn create_enet_stack<'d, P>(
    driver: &'d mut dyn Driver,
    pool: &'d mut EnetPool<P>,
    storage: &'d mut EnetStackStorage,
    seed: u64,
) -> Result<(Iface<'d>, Runner<'d>), Error>
where
    P: Pool,
{
    let (stack, runner) = Stack::new(storage.get(), pool.get(), seed);

    let iface = stack.add_iface_borrowed(driver).map_err(to_net_error)?;
    iface
        .set_dhcpv4(Some(Default::default()))
        .map_err(to_net_error)?;

    Ok((iface, runner))
}

/// An `embassy-net` driver over a driver of the `embassy-net-driver` 0.2 crate, for drivers which
/// do not implement the `xarxa-driver` trait that `embassy-net` takes now, such as the Wifi
/// driver of `esp-radio`.
///
/// Every frame is copied between the buffers of the driver and those of the packet pool.
pub struct EnetLegacyDriver<D> {
    driver: D,
    waker: Option<Waker>,
    rx_buf: Option<DriverBuf>,
}

impl<D> EnetLegacyDriver<D> {
    /// Create a new `EnetLegacyDriver` instance
    pub const fn new(driver: D) -> Self {
        Self {
            driver,
            waker: None,
            rx_buf: None,
        }
    }
}

impl<D> EnetLegacyDriver<D>
where
    D: embassy_net_driver::Driver,
{
    /// Call `f` with the driver and a context which wakes the waker `embassy-net` registered
    fn with<R>(&mut self, f: impl FnOnce(&mut D, &mut Option<DriverBuf>, &mut Context) -> R) -> R {
        let mut cx = Context::from_waker(self.waker.as_ref().unwrap_or(Waker::noop()));

        f(&mut self.driver, &mut self.rx_buf, &mut cx)
    }
}

impl<D> Driver for EnetLegacyDriver<D>
where
    D: embassy_net_driver::Driver,
{
    fn capabilities(&self) -> Capabilities {
        let mut caps = Capabilities::default();
        caps.medium = self.hardware_address().medium();
        caps.max_transmission_unit = self.driver.capabilities().max_transmission_unit;

        caps
    }

    fn hardware_address(&self) -> DriverHardwareAddress {
        match self.driver.hardware_address() {
            embassy_net_driver::HardwareAddress::Ethernet(addr) => {
                DriverHardwareAddress::Ethernet(addr)
            }
            embassy_net_driver::HardwareAddress::Ieee802154(addr) => {
                DriverHardwareAddress::Ieee802154(addr)
            }
            _ => DriverHardwareAddress::Ip,
        }
    }

    fn link_state(&mut self) -> LinkState {
        match self.with(|driver, _, cx| driver.link_state(cx)) {
            embassy_net_driver::LinkState::Up => LinkState::Up,
            embassy_net_driver::LinkState::Down => LinkState::Down,
        }
    }

    fn register_waker(&mut self, waker: &Waker) -> Result<(), NotSupported> {
        self.waker = Some(waker.clone());

        Ok(())
    }

    fn rx_wanted(&mut self) -> RxWanted {
        // One buffer is enough, as the driver keeps the frames in its own buffers until then
        let starved = self.rx_buf.is_none();

        RxWanted {
            count: starved as usize,
            starved,
        }
    }

    fn rx_give(&mut self, buf: DriverBuf) {
        self.rx_buf = Some(buf);
    }

    fn receive(&mut self) -> Option<DriverBuf> {
        use embassy_net_driver::RxToken;

        self.with(|driver, rx_buf, cx| {
            let buf = rx_buf.as_mut()?;

            while let Some((token, _)) = driver.receive(cx) {
                let len = token.consume(|frame| {
                    let storage = buf.storage_mut();
                    let fits = frame.len() <= storage.len();

                    if fits {
                        storage[..frame.len()].copy_from_slice(frame);
                    }

                    fits.then_some(frame.len())
                });

                // A frame which does not fit in a buffer of the pool is dropped
                if let Some(len) = len {
                    let mut buf = unwrap!(rx_buf.take());
                    buf.set_len(len);

                    return Some(buf);
                }
            }

            None
        })
    }

    fn can_transmit(&mut self) -> bool {
        self.with(|driver, _, cx| driver.transmit(cx).is_some())
    }

    fn transmit(&mut self, buf: DriverBuf) -> Result<(), DriverBuf> {
        use embassy_net_driver::TxToken;

        self.with(|driver, _, cx| {
            let Some(token) = driver.transmit(cx) else {
                return Err(buf);
            };

            token.consume(buf.len(), |frame| frame.copy_from_slice(&buf));

            Ok(())
        })
    }
}

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

/// A `Netif` trait implementation for `embassy-net`
pub struct EnetNetif<'d> {
    iface: Iface<'d>,
    up: Mutex<Cell<bool>>,
    netif_type: InterfaceTypeEnum,
}

impl<'d> EnetNetif<'d> {
    /// Create a new `EmbassyNetif` instance
    pub const fn new(iface: Iface<'d>, netif_type: InterfaceTypeEnum) -> Self {
        Self {
            iface,
            up: Mutex::new(Cell::new(false)),
            netif_type,
        }
    }
}

impl NetifDiag for EnetNetif<'_> {
    fn netifs(&self, f: &mut dyn FnMut(&NetifInfo) -> Result<(), Error>) -> Result<(), Error> {
        let mut ipv4_addrs = Vec::<Ipv4Addr, IFACE_ADDR_COUNT>::new();
        let mut ipv6_addrs = Vec::<Ipv6Addr, IFACE_ADDR_COUNT>::new();

        for addr in self.iface.ip_addrs() {
            // Both fit, as there are no more addresses than `IFACE_ADDR_COUNT`
            match addr.cidr.address() {
                EnetIpAddr::V4(addr) => unwrap!(ipv4_addrs.push(addr).ok()),
                EnetIpAddr::V6(addr) => unwrap!(ipv6_addrs.push(addr).ok()),
            }
        }

        let mut hw_addr = [0; 8];
        #[allow(clippy::single_match)]
        match self.iface.hardware_addr() {
            HardwareAddress::Ethernet(addr) => hw_addr[..6].copy_from_slice(&addr.0),
            #[allow(unreachable_patterns)]
            _ => (),
        }

        f(&NetifInfo {
            name: "enet",
            operational: self.iface.is_link_up(),
            offprem_svc_reachable_ipv4: None,
            offprem_svc_reachable_ipv6: None,
            hw_addr: &hw_addr,
            ipv4_addrs: &ipv4_addrs,
            ipv6_addrs: &ipv6_addrs,
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
                self.iface.wait_config_down(),
                Timer::after(Duration::from_secs(TIMEOUT_PERIOD_SECS as _)),
            )
            .await;
        } else {
            select(
                self.iface.wait_config_up(),
                Timer::after(Duration::from_secs(TIMEOUT_PERIOD_SECS as _)),
            )
            .await;
        }

        self.up.lock(|up| up.set(self.iface.is_config_up()));
    }
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
    iface: Iface<'a>,
    #[cfg(feature = "tcp")]
    tcp: Option<EnetTcpStack<'a>>,
}

impl<'a> EnetStack<'a> {
    /// Create a new `EnetStack` instance over the given interface
    pub const fn new(iface: Iface<'a>) -> Self {
        Self {
            iface,
            #[cfg(feature = "tcp")]
            tcp: None,
        }
    }

    /// Create a new `EnetStack` instance with Matter-over-TCP enabled.
    #[cfg(feature = "tcp")]
    pub fn new_with_tcp(
        iface: Iface<'a>,
        tcp_buffers: &'a EnetMatterTcpBuffers,
        matter_tcp_buffers: &'a EnetMatterTcpTransportBuffers,
        tcp_config: &'a TcpConfig,
    ) -> Self {
        Self {
            iface,
            tcp: Some(EnetTcpStack {
                stack: iface.stack(),
                tcp_buffers,
                matter_tcp_buffers,
                tcp_config,
            }),
        }
    }

    fn bind_socket(&self, addr: &SocketAddr) -> Result<EnetSocket<'a>, Error> {
        let mut socket =
            UdpSocket::new(self.iface.stack()).map_err(|_| ErrorCode::ResourceExhausted)?;

        socket
            .bind(listen_addr(addr), ListenSocketAddr::UNSPECIFIED)
            .map_err(to_net_error)?;

        Ok(EnetSocket {
            socket,
            iface: self.iface,
        })
    }
}

/// The local address `embassy-net` should bind to for `addr`: an unspecified address binds to
/// both IPv4 and IPv6, as it does with the BSD sockets.
fn listen_addr(addr: &SocketAddr) -> ListenSocketAddr {
    let ip = addr.ip();

    ListenSocketAddr {
        addr: (!ip.is_unspecified()).then(|| ip.into()),
        port: addr.port(),
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

    #[cfg(feature = "tcp")]
    fn tcp(&self) -> Option<impl NetTcpStack + '_> {
        self.tcp
    }
}

/// RX buffer size per Matter-over-TCP socket in `EnetMatterTcpBuffers`.
#[cfg(feature = "tcp")]
pub const ENET_MATTER_TCP_RX_BUF_SIZE: usize = 1024;
/// TX buffer size per Matter-over-TCP socket in `EnetMatterTcpBuffers`.
#[cfg(feature = "tcp")]
pub const ENET_MATTER_TCP_TX_BUF_SIZE: usize = 1024;

/// Static TCP socket buffers reserved for Matter-over-TCP.
#[cfg(feature = "tcp")]
pub struct EnetMatterTcpBuffers {
    taken: Mutex<Cell<[bool; ENET_MAX_TCP_SOCKETS]>>,
    slots: [UnsafeCell<EnetTcpSlot>; ENET_MAX_TCP_SOCKETS],
}

#[cfg(feature = "tcp")]
impl EnetMatterTcpBuffers {
    #[allow(clippy::declare_interior_mutable_const)]
    const SLOT: UnsafeCell<EnetTcpSlot> = UnsafeCell::new(EnetTcpSlot::new());

    /// Create a new `EnetMatterTcpBuffers` instance.
    pub const fn new() -> Self {
        Self {
            taken: Mutex::new(Cell::new([false; ENET_MAX_TCP_SOCKETS])),
            slots: [Self::SLOT; ENET_MAX_TCP_SOCKETS],
        }
    }

    /// Return an in-place initializer for `EnetMatterTcpBuffers`.
    pub fn init() -> impl Init<Self> {
        init!(Self {
            taken: Mutex::new(Cell::new([false; ENET_MAX_TCP_SOCKETS])),
            slots <- init_array_from_fn(|_| UnsafeCell::init(EnetTcpSlot::init())),
        })
    }

    /// Take a free TCP slot, which stays taken until the returned `EnetTcpSlotRef` is dropped.
    #[allow(clippy::mut_from_ref)]
    fn take(&self) -> Option<(EnetTcpSlotRef<'_>, &mut EnetTcpSlot)> {
        let index = self.taken.lock(|taken| {
            let mut slots = taken.get();
            let index = slots.iter().position(|taken| !taken)?;

            slots[index] = true;
            taken.set(slots);

            Some(index)
        })?;

        // SAFETY: The slot is marked as taken, so no other reference to it exists
        // until `EnetTcpSlotRef` gives it back.
        let slot = unsafe { &mut *self.slots[index].get() };

        Some((
            EnetTcpSlotRef {
                buffers: self,
                index,
            },
            slot,
        ))
    }
}

#[cfg(feature = "tcp")]
impl Default for EnetMatterTcpBuffers {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "tcp")]
struct EnetTcpSlot {
    rx: [u8; ENET_MATTER_TCP_RX_BUF_SIZE],
    tx: [u8; ENET_MATTER_TCP_TX_BUF_SIZE],
}

#[cfg(feature = "tcp")]
impl EnetTcpSlot {
    const fn new() -> Self {
        Self {
            rx: [0; ENET_MATTER_TCP_RX_BUF_SIZE],
            tx: [0; ENET_MATTER_TCP_TX_BUF_SIZE],
        }
    }

    fn init() -> impl Init<Self> {
        init!(Self {
            rx <- zeroed(),
            tx <- zeroed(),
        })
    }
}

/// Gives a taken slot of `EnetMatterTcpBuffers` back when dropped.
#[cfg(feature = "tcp")]
pub struct EnetTcpSlotRef<'a> {
    buffers: &'a EnetMatterTcpBuffers,
    index: usize,
}

#[cfg(feature = "tcp")]
impl Drop for EnetTcpSlotRef<'_> {
    fn drop(&mut self) {
        self.buffers.taken.lock(|taken| {
            let mut slots = taken.get();
            slots[self.index] = false;
            taken.set(slots);
        });
    }
}

/// Caller-owned Matter TCP frame and slot buffers for [`EnetStack`].
#[cfg(feature = "tcp")]
pub type EnetMatterTcpTransportBuffers =
    TcpNetworkBuffers<{ MAX_RX_BUF_SIZE }, 1, { MAX_RX_BUF_SIZE + FRAME_HDR_LEN }>;

/// An implementation of `NetTcpStack` for the `embassy-net` stack.
#[cfg(feature = "tcp")]
#[derive(Copy, Clone)]
pub struct EnetTcpStack<'a> {
    stack: Stack<'a>,
    tcp_buffers: &'a EnetMatterTcpBuffers,
    matter_tcp_buffers: &'a EnetMatterTcpTransportBuffers,
    tcp_config: &'a TcpConfig,
}

#[cfg(feature = "tcp")]
impl<'a> EnetTcpStack<'a> {
    /// Create a listener bound to `addr` using this stack's TCP buffers.
    pub fn listener(&self, addr: SocketAddr) -> Result<EnetTcpListener<'a>, Error> {
        EnetTcpListener::new(self.stack, self.tcp_buffers, addr)
    }
}

#[cfg(feature = "tcp")]
impl<'a> NetTcpStack for EnetTcpStack<'a> {
    type Tcp<'t>
        = TcpNetwork<'a, EnetTcpListener<'a>, { MAX_RX_BUF_SIZE }, 1>
    where
        Self: 't;

    fn bind(&self, addr: SocketAddr) -> impl Future<Output = Result<Self::Tcp<'_>, Error>> {
        ready(self.listener(addr).and_then(|listener| {
            TcpNetwork::new(listener, self.matter_tcp_buffers, self.tcp_config)
        }))
    }
}

/// Owned Matter-over-TCP listener for `embassy-net`.
#[cfg(feature = "tcp")]
pub struct EnetTcpListener<'a> {
    stack: Stack<'a>,
    buffers: &'a EnetMatterTcpBuffers,
    listener: IfMutex<TcpListener<'a>>,
}

#[cfg(feature = "tcp")]
impl<'a> EnetTcpListener<'a> {
    /// Create a new Matter-over-TCP listener bound to `addr`.
    pub fn new(
        stack: Stack<'a>,
        buffers: &'a EnetMatterTcpBuffers,
        addr: SocketAddr,
    ) -> Result<Self, Error> {
        let mut listener = TcpListener::new(stack).map_err(|_| ErrorCode::ResourceExhausted)?;
        listener.listen(listen_addr(&addr)).map_err(to_net_error)?;

        Ok(Self {
            stack,
            buffers,
            listener: IfMutex::new(listener),
        })
    }
}

#[cfg(feature = "tcp")]
impl<'a> TcpNetworkSockets for EnetTcpListener<'a> {
    type Socket = EnetTcpSocket<'a>;

    async fn accept(&self) -> Result<(SocketAddr, Self::Socket), Error> {
        // The connection attempt is not answered until a socket accepts it, so the socket and
        // its buffers are taken only once there is one
        let token = self
            .listener
            .lock()
            .await
            .accept()
            .await
            .map_err(to_net_error)?;

        let (slot_ref, slot) = self.buffers.take().ok_or(ErrorCode::ResourceExhausted)?;

        let mut socket = TcpSocket::new(self.stack, &mut slot.rx, &mut slot.tx)
            .map_err(|_| ErrorCode::ResourceExhausted)?;
        socket.set_nagle_enabled(false);
        socket.accept(token).await.map_err(to_net_error)?;

        let addr = socket.remote_addr().ok_or(ErrorCode::StdIoError)?.into();

        Ok((
            addr,
            EnetTcpSocket {
                socket,
                timeout: None,
                _slot_ref: slot_ref,
            },
        ))
    }
}

/// The TCP stream socket of `EnetStack`.
#[cfg(feature = "tcp")]
pub struct EnetTcpSocket<'a> {
    socket: TcpSocket<'a, 'a>,
    timeout: Option<Duration>,
    // Dropped after `socket`, so its buffers are given back only once the socket is gone.
    _slot_ref: EnetTcpSlotRef<'a>,
}

#[cfg(feature = "tcp")]
impl EnetTcpSocket<'_> {
    /// Return the configured inactivity timeout of the underlying `embassy-net` socket.
    #[doc(hidden)]
    pub fn timeout(&self) -> Option<Duration> {
        self.timeout
    }
}

#[cfg(feature = "tcp")]
impl<'d> TcpNetworkSocket for EnetTcpSocket<'d> {
    type Read<'a>
        = EnetTcpReader<'a, 'd>
    where
        Self: 'a;

    type Write<'a>
        = EnetTcpWriter<'a, 'd>
    where
        Self: 'a;

    fn split(&mut self) -> (Self::Read<'_>, Self::Write<'_>) {
        let (reader, writer) = self.socket.split();
        (EnetTcpReader(reader), EnetTcpWriter(writer))
    }

    async fn close(&mut self) -> Result<(), Error> {
        self.socket.close();
        self.socket.flush().await.map_err(to_net_error)
    }

    async fn abort(&mut self) -> Result<(), Error> {
        // The RST goes out only when the stack runs next, so wait for that before the
        // socket is dropped
        self.socket.abort();
        self.socket.flush().await.map_err(to_net_error)
    }

    fn set_keep_alive(&mut self, keep_alive: Option<TcpKeepAlive>) -> Result<(), Error> {
        let duration = keep_alive.map(|k| Duration::from_millis(k.interval.as_millis()));
        self.socket.set_keep_alive(duration);

        let timeout = keep_alive.and_then(|k| {
            let interval_ms = k.interval.as_millis();
            let idle_ms = k.idle.as_millis();
            let total_ms = idle_ms.checked_add(interval_ms.checked_mul(u64::from(k.retries))?)?;
            Some(Duration::from_millis(total_ms))
        });
        self.socket.set_timeout(timeout);
        self.timeout = timeout;

        Ok(())
    }
}

/// Readable half of `EnetTcpSocket`.
#[cfg(feature = "tcp")]
pub struct EnetTcpReader<'a, 'd>(TcpReader<'a, 'd>);

#[cfg(feature = "tcp")]
impl TcpNetworkRead for EnetTcpReader<'_, '_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        self.0.read(buf).await.map_err(to_net_error)
    }
}

/// Writable half of `EnetTcpSocket`.
#[cfg(feature = "tcp")]
pub struct EnetTcpWriter<'a, 'd>(TcpWriter<'a, 'd>);

#[cfg(feature = "tcp")]
impl TcpNetworkWrite for EnetTcpWriter<'_, '_> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Error> {
        self.0.write(buf).await.map_err(to_net_error)
    }
}

/// The UDP socket of `EnetStack`
pub struct EnetSocket<'a> {
    socket: UdpSocket<'a>,
    iface: Iface<'a>,
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

        self.socket.send_to(data, addr).await.map_err(to_net_error)
    }
}

impl NetworkReceive for &EnetSocket<'_> {
    async fn wait_available(&mut self) -> Result<(), Error> {
        self.socket.wait_recv_ready().await;

        Ok(())
    }

    async fn recv_from(&mut self, buffer: &mut [u8]) -> Result<(usize, Address), Error> {
        let (len, meta) = self.socket.recv_from(buffer).await.map_err(to_net_error)?;

        Ok((len, Address::Udp(meta.remote_addr.into())))
    }
}

impl NetworkMulticast for &EnetSocket<'_> {
    fn join(&mut self, addr: IpAddr) -> impl Future<Output = Result<(), Error>> {
        ready(self.iface.join_multicast_group(addr).map_err(to_net_error))
    }

    fn leave(&mut self, addr: IpAddr) -> impl Future<Output = Result<(), Error>> {
        ready(self.iface.leave_multicast_group(addr).map_err(to_net_error))
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

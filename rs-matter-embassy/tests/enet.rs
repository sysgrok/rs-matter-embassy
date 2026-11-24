//! Tests for the UDP sockets of `EnetStack`, over two `embassy-net` stacks on a link in RAM

#![cfg(feature = "embassy-net")]
#![allow(clippy::large_unopt_stack_frames)]

#[cfg(feature = "tcp")]
use core::mem::MaybeUninit;
use core::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr, SocketAddrV6};
use core::task::Waker;

use std::collections::VecDeque;
use std::sync::Mutex;

use rs_matter_embassy::enet::net::driver::{
    Capabilities, Driver, DriverBuf, HardwareAddress, LinkState, NotSupported, RxWanted,
};
use rs_matter_embassy::enet::net::iface::Iface;
#[cfg(feature = "tcp")]
use rs_matter_embassy::enet::net::tcp::TcpSocket;
use rs_matter_embassy::enet::net::{Runner, StaticPool};
use rs_matter_embassy::enet::{
    create_enet_stack, create_link_local_ipv6, EnetPool, EnetStack, EnetStackStorage,
};
#[cfg(feature = "tcp")]
use rs_matter_embassy::enet::{EnetMatterTcpBuffers, EnetMatterTcpTransportBuffers};
use rs_matter_embassy::matter::error::Error;
#[cfg(feature = "tcp")]
use rs_matter_embassy::matter::error::ErrorCode;
use rs_matter_embassy::matter::transport::network::{
    Address, NetworkMulticast, NetworkReceive, NetworkSend,
};
#[cfg(feature = "tcp")]
use rs_matter_embassy::matter::utils::futures::select;
use rs_matter_embassy::matter::utils::futures::{block_on, select3, Either3};
#[cfg(feature = "tcp")]
use rs_matter_embassy::matter::utils::init::InitMaybeUninit;
use rs_matter_embassy::matter::utils::timers::{with_timeout, Duration};
use rs_matter_embassy::stack::net::{NetSocket, NetStack};
#[cfg(feature = "tcp")]
use rs_matter_embassy::stack::net::{NetTcp, NetTcpStack};

const A_MAC: [u8; 6] = [0x02, 0, 0, 0, 0, 0x01];
const B_MAC: [u8; 6] = [0x02, 0, 0, 0, 0, 0x02];

const PORT: u16 = 5540;

/// The empty buffers a `Link` keeps to receive frames into
const LINK_RX_BUFS: usize = 4;

/// The packet pool of one stack in the tests
type TestPool = StaticPool<1514, 16>;

/// One end of the link in RAM, with the stack running over it
struct Node<'a> {
    link: Link<'a>,
    pool: EnetPool<TestPool>,
    storage: EnetStackStorage,
}

impl<'a> Node<'a> {
    fn new(mac: [u8; 6], tx: &'a Mutex<Frames>, rx: &'a Mutex<Frames>) -> Self {
        Self {
            link: Link::new(mac, tx, rx),
            pool: EnetPool::new(Box::leak(Box::new(TestPool::new()))),
            storage: EnetStackStorage::new(),
        }
    }

    fn stack(&mut self, seed: u64) -> (Iface<'_>, Runner<'_>) {
        create_enet_stack(&mut self.link, &mut self.pool, &mut self.storage, seed).unwrap()
    }
}

#[test]
fn sockets_exchange_unicast_and_multicast_packets() {
    let a_to_b = Mutex::new(Frames::default());
    let b_to_a = Mutex::new(Frames::default());

    let mut a_node = Node::new(A_MAC, &a_to_b, &b_to_a);
    let mut b_node = Node::new(B_MAC, &b_to_a, &a_to_b);

    let (a_iface, mut a_runner) = a_node.stack(1);
    let (b_iface, mut b_runner) = b_node.stack(2);

    let a = EnetStack::new(a_iface);
    let b = EnetStack::new(b_iface);

    let result = block_on(select3(
        with_timeout(Duration::from_secs(10), exchange(&a, &b)),
        a_runner.run(),
        b_runner.run(),
    ));

    assert!(matches!(result, Either3::First(Ok(Ok(())))));
}

async fn exchange(a: &EnetStack<'_>, b: &EnetStack<'_>) -> Result<(), Error> {
    let mut a_socket = a.bind(any(PORT), Ipv4Addr::UNSPECIFIED, 0).await?;
    let mut b_socket = b.bind(any(PORT), Ipv4Addr::UNSPECIFIED, 0).await?;

    // A second socket on the same port is refused, and a dropped socket gives its port back
    assert!(b.bind(any(PORT), Ipv4Addr::UNSPECIFIED, 0).await.is_err());
    let mdns_socket = b.bind(any(5353), Ipv4Addr::UNSPECIFIED, 0).await?;
    drop(mdns_socket);
    drop(b.bind(any(5353), Ipv4Addr::UNSPECIFIED, 0).await?);

    let (mut a_send, _, _) = a_socket.split();
    let (_, mut b_recv, mut b_multicast) = b_socket.split();

    let mut buf = [0; 8];

    a_send.send_to(b"ping", udp(link_local(&B_MAC))).await?;

    b_recv.wait_available().await?;
    let (len, addr) = b_recv.recv_from(&mut buf).await?;

    assert_eq!(&buf[..len], b"ping");
    assert_eq!(addr.udp(), udp(link_local(&A_MAC)).udp());

    let group = Ipv6Addr::new(0xff02, 0, 0, 0, 0, 0, 0, 0xfb);
    b_multicast.join(IpAddr::V6(group)).await?;

    a_send.send_to(b"group", udp(group)).await?;

    let (len, _) = b_recv.recv_from(&mut buf).await?;
    assert_eq!(&buf[..len], b"group");

    b_multicast.leave(IpAddr::V6(group)).await?;

    // A new socket on the port of one that carried traffic works; more packets than a socket
    // queues, so that the packet buffers are given back to the pool
    drop(b_socket);
    let mut b_socket = b.bind(any(PORT), Ipv4Addr::UNSPECIFIED, 0).await?;
    let (_, mut b_recv, _) = b_socket.split();

    for _ in 0..8 {
        a_send.send_to(b"again", udp(link_local(&B_MAC))).await?;

        let (len, _) = b_recv.recv_from(&mut buf).await?;
        assert_eq!(&buf[..len], b"again");
    }

    Ok(())
}

#[cfg(feature = "tcp")]
#[test]
fn tcp_sockets_exchange_framed_messages() {
    let a_to_b = Mutex::new(Frames::default());
    let b_to_a = Mutex::new(Frames::default());

    let mut a_node = Node::new(A_MAC, &a_to_b, &b_to_a);
    let mut b_node = Node::new(B_MAC, &b_to_a, &a_to_b);

    let (a_iface, mut a_runner) = a_node.stack(1);
    let (b_iface, mut b_runner) = b_node.stack(2);

    let mut a_tcp_buffers = MaybeUninit::uninit();
    let a_tcp_buffers = a_tcp_buffers.init_with(EnetMatterTcpBuffers::init());
    let mut a_matter_tcp_buffers = MaybeUninit::<EnetMatterTcpTransportBuffers>::uninit();
    let a_matter_tcp_buffers =
        a_matter_tcp_buffers.init_with(EnetMatterTcpTransportBuffers::init());

    let a_tcp_config = rs_matter_embassy::stack::tcp::TcpConfig::new();
    let a = EnetStack::new_with_tcp(a_iface, a_tcp_buffers, a_matter_tcp_buffers, &a_tcp_config);

    let result = block_on(select3(
        with_timeout(Duration::from_secs(10), exchange_tcp(&a, b_iface.stack())),
        a_runner.run(),
        b_runner.run(),
    ));

    assert!(matches!(result, Either3::First(Ok(Ok(())))));
}

#[cfg(feature = "tcp")]
#[test]
fn tcp_socket_keep_alive_configures_timeout() {
    use core::net::{Ipv6Addr, SocketAddr};
    use rs_matter_embassy::enet::EnetTcpListener;
    use rs_matter_embassy::stack::tcp::{TcpKeepAlive, TcpNetworkSocket, TcpNetworkSockets};

    let a_to_b = Mutex::new(Frames::default());
    let b_to_a = Mutex::new(Frames::default());

    let mut a_node = Node::new(A_MAC, &a_to_b, &b_to_a);
    let mut b_node = Node::new(B_MAC, &b_to_a, &a_to_b);

    let (a_iface, mut a_runner) = a_node.stack(1);
    let (b_iface, mut b_runner) = b_node.stack(2);
    let (a_stack, b_stack) = (a_iface.stack(), b_iface.stack());

    let mut a_tcp_buffers = MaybeUninit::uninit();
    let a_tcp_buffers = a_tcp_buffers.init_with(EnetMatterTcpBuffers::init());

    let work = async {
        let listener = EnetTcpListener::new(
            a_stack,
            a_tcp_buffers,
            SocketAddr::new(IpAddr::V6(Ipv6Addr::UNSPECIFIED), 5540),
        )?;

        let mut rx_buf = [0u8; 256];
        let mut tx_buf = [0u8; 256];
        let mut b_sock = TcpSocket::new(b_stack, &mut rx_buf, &mut tx_buf).unwrap();
        let addr_a = link_local(&A_MAC);

        let (accepted_res, connect_res) = rs_matter_embassy::matter::utils::futures::join(
            listener.accept(),
            b_sock.connect((addr_a, 5540)),
        )
        .await;

        connect_res.unwrap();
        let (_, mut accepted) = accepted_res.unwrap();

        let keep_alive = TcpKeepAlive {
            idle: Duration::from_secs(25),
            interval: Duration::from_secs(25),
            retries: 5,
        };
        accepted.set_keep_alive(Some(keep_alive)).unwrap();
        assert_eq!(accepted.timeout(), Some(Duration::from_secs(150)));

        accepted.set_keep_alive(None).unwrap();
        assert_eq!(accepted.timeout(), None);

        Ok::<(), Error>(())
    };

    let result = block_on(select3(
        with_timeout(Duration::from_secs(10), work),
        a_runner.run(),
        b_runner.run(),
    ));

    assert!(matches!(result, Either3::First(Ok(Ok(())))));
}

#[cfg(feature = "tcp")]
#[test]
fn tcp_socket_abort_resets_unresponsive_peer() {
    use core::net::{Ipv6Addr, SocketAddr};
    use rs_matter_embassy::enet::EnetTcpListener;
    use rs_matter_embassy::matter::utils::futures::{select, Either};
    use rs_matter_embassy::stack::tcp::{TcpNetworkSocket, TcpNetworkSockets};

    let a_to_b = Mutex::new(Frames::default());
    let b_to_a = Mutex::new(Frames::default());

    let mut a_node = Node::new(A_MAC, &a_to_b, &b_to_a);
    let mut b_node = Node::new(B_MAC, &b_to_a, &a_to_b);

    let (a_iface, mut a_runner) = a_node.stack(1);
    let (b_iface, mut b_runner) = b_node.stack(2);
    let (a_stack, b_stack) = (a_iface.stack(), b_iface.stack());

    let mut a_tcp_buffers = MaybeUninit::uninit();
    let a_tcp_buffers = a_tcp_buffers.init_with(EnetMatterTcpBuffers::init());

    let listener = EnetTcpListener::new(
        a_stack,
        a_tcp_buffers,
        SocketAddr::new(IpAddr::V6(Ipv6Addr::UNSPECIFIED), 5540),
    )
    .unwrap();

    let mut rx_buf = [0u8; 256];
    let mut tx_buf = [0u8; 256];
    let mut b_sock = TcpSocket::new(b_stack, &mut rx_buf, &mut tx_buf).unwrap();
    let addr_a = link_local(&A_MAC);

    let connect = rs_matter_embassy::matter::utils::futures::join(
        listener.accept(),
        b_sock.connect((addr_a, 5540)),
    );

    let mut accepted = match block_on(select3(
        with_timeout(Duration::from_secs(5), connect),
        a_runner.run(),
        b_runner.run(),
    )) {
        Either3::First(Ok((accepted_res, connect_res))) => {
            connect_res.unwrap();
            accepted_res.unwrap().1
        }
        _ => panic!("connect timed out"),
    };

    // `b_runner` is not polled any more, so the peer never acknowledges the FIN, and
    // `close()` does not finish
    let result = block_on(select(
        with_timeout(Duration::from_millis(500), accepted.close()),
        a_runner.run(),
    ));

    assert!(matches!(result, Either::First(Err(_))));

    // `abort()` waits only for the RST to go out
    let result = block_on(select(
        with_timeout(Duration::from_secs(5), accepted.abort()),
        a_runner.run(),
    ));

    assert!(matches!(result, Either::First(Ok(Ok(())))));

    // The RST is on the link, although the socket is dropped right after `abort()`
    drop(accepted);
    assert!(a_to_b.lock().unwrap().frames.iter().any(|f| is_tcp_rst(f)));
}

#[cfg(feature = "tcp")]
async fn exchange_tcp(
    a: &EnetStack<'_>,
    b_stack: rs_matter_embassy::enet::net::Stack<'_>,
) -> Result<(), Error> {
    let tcp = a.tcp().expect("tcp enabled");
    let a_tcp = tcp.bind(any(PORT)).await?;
    let mut a_send = a_tcp.send();
    let mut a_recv = a_tcp.recv();

    let mut rx_buf = [0u8; 1024];
    let mut tx_buf = [0u8; 1024];
    let mut b_sock = TcpSocket::new(b_stack, &mut rx_buf, &mut tx_buf).unwrap();

    let server = async {
        let mut buf = [0u8; 16];
        a_recv.wait_available().await?;
        let (len, addr) = a_recv.recv_from(&mut buf).await?;
        assert_eq!(&buf[..len], b"ping");
        assert!(matches!(addr, Address::Tcp(_)));

        a_send.send_to(b"pong", addr).await?;

        // Wait for peer close notification
        let closed = a_recv.wait_available_or_closed().await?;
        assert_eq!(closed, Some(addr));
        Ok::<(), Error>(())
    };

    let client = async {
        b_sock
            .connect((link_local(&A_MAC), PORT))
            .await
            .map_err(|_| ErrorCode::StdIoError)?;

        let mut frame = [0u8; 8];
        frame[..4].copy_from_slice(&4u32.to_le_bytes());
        frame[4..8].copy_from_slice(b"ping");
        b_sock
            .write(&frame)
            .await
            .map_err(|_| ErrorCode::StdIoError)?;
        b_sock.flush().await.map_err(|_| ErrorCode::StdIoError)?;

        let mut resp = [0u8; 8];
        let mut read = 0;
        while read < resp.len() {
            let n = b_sock
                .read(&mut resp[read..])
                .await
                .map_err(|_| ErrorCode::StdIoError)?;
            assert!(n > 0);
            read += n;
        }
        assert_eq!(u32::from_le_bytes(resp[..4].try_into().unwrap()), 4);
        assert_eq!(&resp[4..8], b"pong");

        b_sock.close();
        b_sock.flush().await.map_err(|_| ErrorCode::StdIoError)?;
        core::future::pending::<Result<(), Error>>().await
    };

    let exchange = select(server, client);
    match select(a_tcp.run(), exchange).await {
        rs_matter_embassy::matter::utils::futures::Either::First(r) => r?,
        rs_matter_embassy::matter::utils::futures::Either::Second(
            rs_matter_embassy::matter::utils::futures::Either::First(r),
        ) => r?,
        rs_matter_embassy::matter::utils::futures::Either::Second(
            rs_matter_embassy::matter::utils::futures::Either::Second(r),
        ) => r?,
    }

    Ok(())
}

fn any(port: u16) -> SocketAddr {
    SocketAddr::V6(SocketAddrV6::new(Ipv6Addr::UNSPECIFIED, port, 0, 0))
}

fn udp(addr: Ipv6Addr) -> Address {
    Address::Udp(SocketAddr::V6(SocketAddrV6::new(addr, PORT, 0, 0)))
}

fn link_local(mac: &[u8; 6]) -> Ipv6Addr {
    create_link_local_ipv6(mac)
}

/// Whether `frame` carries a TCP segment over IPv6 with the RST flag set
#[cfg(feature = "tcp")]
fn is_tcp_rst(frame: &[u8]) -> bool {
    const TCP_HDR: usize = 14 + 40;

    frame.len() >= TCP_HDR + 20
        && frame[12..14] == [0x86, 0xdd]
        && frame[14 + 6] == 6
        && frame[TCP_HDR + 13] & 0x04 != 0
}

/// The frames going one way over a `Link`
#[derive(Default)]
struct Frames {
    frames: VecDeque<Vec<u8>>,
    waker: Option<Waker>,
}

/// One end of an Ethernet link in RAM
struct Link<'a> {
    mac: [u8; 6],
    tx: &'a Mutex<Frames>,
    rx: &'a Mutex<Frames>,
    rx_bufs: VecDeque<DriverBuf>,
}

impl<'a> Link<'a> {
    const fn new(mac: [u8; 6], tx: &'a Mutex<Frames>, rx: &'a Mutex<Frames>) -> Self {
        Self {
            mac,
            tx,
            rx,
            rx_bufs: VecDeque::new(),
        }
    }
}

impl Driver for Link<'_> {
    fn capabilities(&self) -> Capabilities {
        let mut caps = Capabilities::default();
        caps.max_transmission_unit = 1514;

        caps
    }

    fn hardware_address(&self) -> HardwareAddress {
        HardwareAddress::Ethernet(self.mac)
    }

    fn link_state(&mut self) -> LinkState {
        LinkState::Up
    }

    fn register_waker(&mut self, waker: &Waker) -> Result<(), NotSupported> {
        self.rx.lock().unwrap().waker = Some(waker.clone());

        Ok(())
    }

    fn rx_wanted(&mut self) -> RxWanted {
        RxWanted {
            count: LINK_RX_BUFS - self.rx_bufs.len(),
            starved: self.rx_bufs.is_empty(),
        }
    }

    fn rx_give(&mut self, buf: DriverBuf) {
        self.rx_bufs.push_back(buf);
    }

    fn receive(&mut self) -> Option<DriverBuf> {
        if self.rx_bufs.is_empty() {
            return None;
        }

        let frame = self.rx.lock().unwrap().frames.pop_front()?;

        let mut buf = self.rx_bufs.pop_front()?;
        buf.storage_mut()[..frame.len()].copy_from_slice(&frame);
        buf.set_len(frame.len());

        Some(buf)
    }

    fn can_transmit(&mut self) -> bool {
        true
    }

    fn transmit(&mut self, buf: DriverBuf) -> Result<(), DriverBuf> {
        let mut tx = self.tx.lock().unwrap();
        tx.frames.push_back(buf.to_vec());

        if let Some(waker) = tx.waker.take() {
            waker.wake();
        }

        Ok(())
    }
}

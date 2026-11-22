//! Tests for the UDP sockets of `EnetStack`, over two `embassy-net` stacks on a link in RAM

#![cfg(feature = "embassy-net")]
#![allow(clippy::large_unopt_stack_frames)]

use core::mem::MaybeUninit;
use core::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr, SocketAddrV6};
use core::task::{Context, Waker};

use std::collections::VecDeque;
use std::sync::Mutex;

use rs_matter_embassy::enet::net::driver::{
    Capabilities, Driver, HardwareAddress, LinkState, RxToken, TxToken,
};
#[cfg(feature = "tcp")]
use rs_matter_embassy::enet::net::tcp::TcpSocket;
use rs_matter_embassy::enet::net::StackResources;
#[cfg(feature = "mld-v1-compat")]
use rs_matter_embassy::enet::Mldv1ConvertingDriver;
use rs_matter_embassy::enet::{
    create_enet_stack, create_link_local_ipv6, EnetMatterUdpBuffers, EnetStack,
};
#[cfg(feature = "tcp")]
use rs_matter_embassy::enet::{EnetMatterTcpBuffers, EnetMatterTcpTransportBuffers};
use rs_matter_embassy::matter::error::{Error, ErrorCode};
use rs_matter_embassy::matter::transport::network::{
    Address, NetworkMulticast, NetworkReceive, NetworkSend,
};
#[cfg(feature = "tcp")]
use rs_matter_embassy::matter::utils::futures::select;
use rs_matter_embassy::matter::utils::futures::{block_on, select3, Either3};
use rs_matter_embassy::matter::utils::init::InitMaybeUninit;
use rs_matter_embassy::matter::utils::timers::{with_timeout, Duration};
use rs_matter_embassy::stack::net::{NetSocket, NetStack};
#[cfg(feature = "tcp")]
use rs_matter_embassy::stack::net::{NetTcp, NetTcpStack};

const A_MAC: [u8; 6] = [0x02, 0, 0, 0, 0, 0x01];
const B_MAC: [u8; 6] = [0x02, 0, 0, 0, 0, 0x02];

const PORT: u16 = 5540;

#[test]
fn sockets_exchange_unicast_and_multicast_packets() {
    let a_to_b = Mutex::new(Frames::default());
    let b_to_a = Mutex::new(Frames::default());

    let mut a_resources = StackResources::<4>::new();
    let mut b_resources = StackResources::<4>::new();

    let (a_stack, mut a_runner) =
        create_enet_stack(Link::new(A_MAC, &a_to_b, &b_to_a), 1, &mut a_resources);
    let (b_stack, mut b_runner) =
        create_enet_stack(Link::new(B_MAC, &b_to_a, &a_to_b), 2, &mut b_resources);

    // One side built with `new`, the other in place
    let a_buffers = EnetMatterUdpBuffers::new();
    let mut b_buffers = MaybeUninit::uninit();
    let b_buffers = b_buffers.init_with(EnetMatterUdpBuffers::init());

    let a = EnetStack::new(a_stack, &a_buffers);
    let b = EnetStack::new(b_stack, b_buffers);

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

    // The buffers have room for two sockets, and a dropped socket gives its room back
    let mdns_socket = b.bind(any(5353), Ipv4Addr::UNSPECIFIED, 0).await?;
    assert!(matches!(
        b.bind(any(5354), Ipv4Addr::UNSPECIFIED, 0).await,
        Err(e) if e.code() == ErrorCode::ResourceExhausted
    ));
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

    // A new socket in the slot that carried traffic works; more packets than a socket
    // has metadata entries, so that its rings wrap
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

    let mut a_resources = StackResources::<6>::new();
    let mut b_resources = StackResources::<4>::new();

    let (a_stack, mut a_runner) =
        create_enet_stack(Link::new(A_MAC, &a_to_b, &b_to_a), 1, &mut a_resources);
    let (b_stack, mut b_runner) =
        create_enet_stack(Link::new(B_MAC, &b_to_a, &a_to_b), 2, &mut b_resources);

    let a_udp_buffers = EnetMatterUdpBuffers::new();
    let mut a_tcp_buffers = MaybeUninit::uninit();
    let a_tcp_buffers = a_tcp_buffers.init_with(EnetMatterTcpBuffers::init());
    let mut a_matter_tcp_buffers = MaybeUninit::<EnetMatterTcpTransportBuffers>::uninit();
    let a_matter_tcp_buffers =
        a_matter_tcp_buffers.init_with(EnetMatterTcpTransportBuffers::init());

    let a_tcp_config = rs_matter_embassy::stack::tcp::TcpConfig::new();
    let a = EnetStack::new_with_tcp(
        a_stack,
        &a_udp_buffers,
        a_tcp_buffers,
        a_matter_tcp_buffers,
        &a_tcp_config,
    );

    let result = block_on(select3(
        with_timeout(Duration::from_secs(10), exchange_tcp(&a, b_stack)),
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

    let mut a_resources = StackResources::<6>::new();
    let mut b_resources = StackResources::<4>::new();

    let (a_stack, mut a_runner) =
        create_enet_stack(Link::new(A_MAC, &a_to_b, &b_to_a), 1, &mut a_resources);
    let (b_stack, mut b_runner) =
        create_enet_stack(Link::new(B_MAC, &b_to_a, &a_to_b), 2, &mut b_resources);

    let mut a_tcp_buffers = MaybeUninit::uninit();
    let a_tcp_buffers = a_tcp_buffers.init_with(EnetMatterTcpBuffers::init());

    let work = async {
        let listener = EnetTcpListener::new(
            a_stack,
            a_tcp_buffers,
            SocketAddr::new(IpAddr::V6(Ipv6Addr::UNSPECIFIED), 5540),
        );

        let mut rx_buf = [0u8; 256];
        let mut tx_buf = [0u8; 256];
        let mut b_sock = TcpSocket::new(b_stack, &mut rx_buf, &mut tx_buf);
        let addr_a = a_stack.config_v6().unwrap().address.address();

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

    let mut a_resources = StackResources::<6>::new();
    let mut b_resources = StackResources::<4>::new();

    let (a_stack, mut a_runner) =
        create_enet_stack(Link::new(A_MAC, &a_to_b, &b_to_a), 1, &mut a_resources);
    let (b_stack, mut b_runner) =
        create_enet_stack(Link::new(B_MAC, &b_to_a, &a_to_b), 2, &mut b_resources);

    let mut a_tcp_buffers = MaybeUninit::uninit();
    let a_tcp_buffers = a_tcp_buffers.init_with(EnetMatterTcpBuffers::init());

    let listener = EnetTcpListener::new(
        a_stack,
        a_tcp_buffers,
        SocketAddr::new(IpAddr::V6(Ipv6Addr::UNSPECIFIED), 5540),
    );

    let mut rx_buf = [0u8; 256];
    let mut tx_buf = [0u8; 256];
    let mut b_sock = TcpSocket::new(b_stack, &mut rx_buf, &mut tx_buf);
    let addr_a = a_stack.config_v6().unwrap().address.address();

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
    let mut b_sock = TcpSocket::new(b_stack, &mut rx_buf, &mut tx_buf);

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
}

impl<'a> Link<'a> {
    const fn new(mac: [u8; 6], tx: &'a Mutex<Frames>, rx: &'a Mutex<Frames>) -> Self {
        Self { mac, tx, rx }
    }
}

impl Driver for Link<'_> {
    type RxToken<'a>
        = Rx
    where
        Self: 'a;

    type TxToken<'a>
        = Tx<'a>
    where
        Self: 'a;

    fn receive(&mut self, cx: &mut Context) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        let mut rx = self.rx.lock().unwrap();

        if let Some(frame) = rx.frames.pop_front() {
            Some((Rx(frame), Tx(self.tx)))
        } else {
            rx.waker = Some(cx.waker().clone());
            None
        }
    }

    fn transmit(&mut self, _cx: &mut Context) -> Option<Self::TxToken<'_>> {
        Some(Tx(self.tx))
    }

    fn link_state(&mut self, _cx: &mut Context) -> LinkState {
        LinkState::Up
    }

    fn capabilities(&self) -> Capabilities {
        let mut caps = Capabilities::default();
        caps.max_transmission_unit = 1514;

        caps
    }

    fn hardware_address(&self) -> HardwareAddress {
        HardwareAddress::Ethernet(self.mac)
    }
}

struct Rx(Vec<u8>);

impl RxToken for Rx {
    fn consume<R, F>(mut self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        f(&mut self.0)
    }
}

struct Tx<'a>(&'a Mutex<Frames>);

impl TxToken for Tx<'_> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let mut frame = vec![0; len];
        let result = f(&mut frame);

        let mut tx = self.0.lock().unwrap();
        tx.frames.push_back(frame);

        if let Some(waker) = tx.waker.take() {
            waker.wake();
        }

        result
    }
}

#[cfg(feature = "mld-v1-compat")]
fn noop_waker() -> Waker {
    use core::task::{RawWaker, RawWakerVTable};
    unsafe fn clone(_: *const ()) -> RawWaker {
        RawWaker::new(core::ptr::null(), &VTABLE)
    }
    unsafe fn wake(_: *const ()) {}
    unsafe fn wake_by_ref(_: *const ()) {}
    unsafe fn drop(_: *const ()) {}
    static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, wake, wake_by_ref, drop);
    unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) }
}

#[cfg(feature = "mld-v1-compat")]
fn icmpv6_checksum_over_ipv6_frame(frame: &[u8], icmp_offset: usize, icmp_len: usize) -> u16 {
    let mut sum: u32 = 0;
    let mut i = 22;
    while i < 54 {
        sum += u16::from_be_bytes([frame[i], frame[i + 1]]) as u32;
        i += 2;
    }
    sum += icmp_len as u32;
    sum += 58u32; // Next Header = ICMPv6
    let icmp = &frame[icmp_offset..icmp_offset + icmp_len];
    let mut j = 0;
    while j < icmp.len() {
        if j != 2 {
            let hi = icmp[j];
            let lo = if j + 1 < icmp.len() { icmp[j + 1] } else { 0 };
            sum += u16::from_be_bytes([hi, lo]) as u32;
        }
        j += 2;
    }
    while (sum >> 16) != 0 {
        sum = (sum & 0xffff) + (sum >> 16);
    }
    !(sum as u16)
}

#[cfg(feature = "mld-v1-compat")]
mod mldv1_compat_tests {
    use super::*;

    fn build_mldv2_join_frame(src_mac: [u8; 6], src_ip: [u8; 16], group: [u8; 16]) -> [u8; 90] {
        let mut f = [0u8; 90];
        // dst MAC = 33:33:00:00:00:16 (all-MLDv2-routers)
        f[0..6].copy_from_slice(&[0x33, 0x33, 0x00, 0x00, 0x00, 0x16]);
        f[6..12].copy_from_slice(&src_mac);
        f[12..14].copy_from_slice(&[0x86, 0xDD]);
        // IPv6 header
        f[14] = 0x60;
        f[18..20].copy_from_slice(&36u16.to_be_bytes()); // HbH(8) + MLDv2(28)
        f[20] = 0; // Hop-by-Hop
        f[21] = 1; // Hop Limit
        f[22..38].copy_from_slice(&src_ip);
        f[38..54].copy_from_slice(&[0xff, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x16]);
        // Hop-by-Hop Router Alert
        f[54] = 58; // ICMPv6
        f[55] = 0;
        f[56..62].copy_from_slice(&[5, 2, 0, 0, 1, 0]);
        // ICMPv6 MLDv2 Membership Report (type 143)
        f[62] = 143;
        f[68..70].copy_from_slice(&1u16.to_be_bytes()); // nr_records = 1
        f[70] = 4; // CHANGE_TO_EXCLUDE_MODE
        f[74..90].copy_from_slice(&group);
        f
    }

    #[test]
    fn mldv1_driver_rewrites_mldv2_join_with_valid_checksum() {
        let tx = Mutex::new(Frames::default());
        let rx = Mutex::new(Frames::default());
        let mut driver = Mldv1ConvertingDriver::new(Link::new(A_MAC, &tx, &rx));

        let src_ip = create_link_local_ipv6(&A_MAC).octets();
        let group = [
            0xff, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0xff, 0xa6, 0x78, 0x44,
        ];
        let input = build_mldv2_join_frame(A_MAC, src_ip, group);

        let waker = noop_waker();
        let mut cx = Context::from_waker(&waker);
        let tok = driver.transmit(&mut cx).expect("tx token");
        tok.consume(input.len(), |buf| buf.copy_from_slice(&input));

        let out = tx.lock().unwrap().frames.pop_front().expect("tx frame");
        assert_eq!(out[0..6], [0x33, 0x33, 0xff, 0xa6, 0x78, 0x44]);
        assert_eq!(u16::from_be_bytes([out[18], out[19]]), 32); // HbH(8) + MLDv1(24)
        assert_eq!(out[38..54], group);
        assert_eq!(out[62], 131); // MLDv1 Membership Report
        assert_eq!(out[70..86], group);

        let written_cksum = u16::from_be_bytes([out[64], out[65]]);
        let expected_cksum = icmpv6_checksum_over_ipv6_frame(&out, 62, 24);
        assert_eq!(written_cksum, expected_cksum);
        assert_ne!(written_cksum, 0);
    }

    #[test]
    fn mldv1_driver_passes_non_mld_frames_untouched() {
        let tx = Mutex::new(Frames::default());
        let rx = Mutex::new(Frames::default());
        let mut driver = Mldv1ConvertingDriver::new(Link::new(A_MAC, &tx, &rx));

        let src_ip = create_link_local_ipv6(&A_MAC).octets();
        let group = [
            0xff, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0xff, 0xa6, 0x78, 0x44,
        ];
        let base = build_mldv2_join_frame(A_MAC, src_ip, group);

        let waker = noop_waker();
        let mut cx = Context::from_waker(&waker);

        // Exercise passthrough Driver trait methods (link_state, capabilities, hardware_address, receive).
        assert!(driver.link_state(&mut cx) == LinkState::Up);
        assert_eq!(driver.capabilities().max_transmission_unit, 1514);
        assert_eq!(driver.hardware_address(), HardwareAddress::Ethernet(A_MAC));
        rx.lock().unwrap().frames.push_back(vec![1, 2, 3]);
        let (rx_tok, tx_tok) = driver.receive(&mut cx).expect("rx+tx pair");
        rx_tok.consume(|b| assert_eq!(b, &[1, 2, 3]));
        tx_tok.consume(10, |b| b.fill(0xaa)); // len < 90 branch
        assert_eq!(
            tx.lock().unwrap().frames.pop_front().unwrap(),
            vec![0xaa; 10]
        );

        // Also rewrite MODE_IS_EXCLUDE (record type = 2).
        let mut mode_is_exclude = base;
        mode_is_exclude[70] = 2;
        driver
            .transmit(&mut cx)
            .unwrap()
            .consume(90, |b| b.copy_from_slice(&mode_is_exclude));
        let out = tx.lock().unwrap().frames.pop_front().unwrap();
        assert_eq!(out[62], 131);

        // Every guard branch in convert_mldv2_to_mldv1 must leave the frame untouched.
        let mut cases: Vec<[u8; 90]> = Vec::new();

        // 1. buf[12] != 0x86 (IPv4 0x0800)
        let mut c = base;
        c[12] = 0x08;
        c[13] = 0x00;
        cases.push(c);

        // 2. buf[12] == 0x86 && buf[13] != 0xDD
        let mut c = base;
        c[13] = 0x00;
        cases.push(c);

        // 3. buf[20] != 0 (Next Header != Hop-by-Hop)
        let mut c = base;
        c[20] = 17;
        cases.push(c);

        // 4. buf[54] != 58 (HbH Next Header != ICMPv6)
        let mut c = base;
        c[54] = 17;
        cases.push(c);

        // 5. buf[62] != 143 (ICMPv6 type != MLDv2 Report)
        let mut c = base;
        c[62] = 130;
        cases.push(c);

        // 6. nr_records != 1
        let mut c = base;
        c[68..70].copy_from_slice(&2u16.to_be_bytes());
        cases.push(c);

        // 7. num_srcs != 0
        let mut c = base;
        c[72..74].copy_from_slice(&1u16.to_be_bytes());
        cases.push(c);

        // 8. record_type != 2 && record_type != 4 (e.g. CHANGE_TO_INCLUDE = 3)
        let mut c = base;
        c[70] = 3;
        cases.push(c);

        for input in cases {
            let tok = driver.transmit(&mut cx).expect("tx token");
            tok.consume(input.len(), |buf| buf.copy_from_slice(&input));
            let out = tx.lock().unwrap().frames.pop_front().expect("tx frame");
            assert_eq!(out.as_slice(), &input);
        }
    }
}

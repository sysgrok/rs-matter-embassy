//! Tests for the UDP sockets of `EnetStack`, over two `embassy-net` stacks on a link in RAM

#![cfg(feature = "embassy-net")]

use core::mem::MaybeUninit;
use core::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr, SocketAddrV6};
use core::task::{Context, Waker};

use std::collections::VecDeque;
use std::sync::Mutex;

use rs_matter_embassy::enet::net::driver::{
    Capabilities, Driver, HardwareAddress, LinkState, RxToken, TxToken,
};
use rs_matter_embassy::enet::net::StackResources;
use rs_matter_embassy::enet::{
    create_enet_stack, create_link_local_ipv6, EnetMatterUdpBuffers, EnetStack,
};
use rs_matter_embassy::matter::error::{Error, ErrorCode};
use rs_matter_embassy::matter::transport::network::{
    Address, NetworkMulticast, NetworkReceive, NetworkSend,
};
use rs_matter_embassy::matter::utils::futures::{block_on, select3, Either3};
use rs_matter_embassy::matter::utils::init::InitMaybeUninit;
use rs_matter_embassy::matter::utils::timers::{with_timeout, Duration};
use rs_matter_embassy::stack::net::{NetSocket, NetStack};

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

    b_multicast.leave(IpAddr::V6(group)).await
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

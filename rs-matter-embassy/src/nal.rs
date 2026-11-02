//! A `NetSocket` over the `edge-nal` traits, which the UDP sockets of `embassy-net` and
//! `openthread` implement.

use core::fmt::Debug;
use core::net::{IpAddr, Ipv4Addr};

use edge_nal::{
    MulticastV4, MulticastV6, Readable, UdpReceive, UdpSend, UdpSplit, UdpSplitMulticast,
};

use crate::matter::error::{Error, ErrorCode};
use crate::matter::transport::network::{Address, NetworkMulticast, NetworkReceive, NetworkSend};
use crate::stack::net::NetSocket;

/// A `NetSocket` over a bound `edge-nal` UDP socket.
pub struct NalSocket<S> {
    socket: S,
    ipv4_interface: Ipv4Addr,
    ipv6_interface: u32,
}

impl<S> NalSocket<S> {
    /// Create a new instance of the `NalSocket` type.
    pub const fn new(socket: S, ipv4_interface: Ipv4Addr, ipv6_interface: u32) -> Self {
        Self {
            socket,
            ipv4_interface,
            ipv6_interface,
        }
    }
}

impl<S> NetSocket for NalSocket<S>
where
    S: UdpSplitMulticast,
{
    type Send<'a>
        = NalHalf<<S as UdpSplit>::Send<'a>>
    where
        Self: 'a;

    type Receive<'a>
        = NalHalf<<S as UdpSplit>::Receive<'a>>
    where
        Self: 'a;

    type Multicast<'a>
        = NalMulticast<S::MulticastV4<'a>, S::MulticastV6<'a>>
    where
        Self: 'a;

    fn split(&mut self) -> (Self::Send<'_>, Self::Receive<'_>, Self::Multicast<'_>) {
        let (recv, send, m4, m6) = self.socket.split_multicast();

        (
            NalHalf(send),
            NalHalf(recv),
            NalMulticast {
                m4,
                ipv4_interface: self.ipv4_interface,
                m6,
                ipv6_interface: self.ipv6_interface,
            },
        )
    }
}

/// One half of a `NalSocket`.
pub struct NalHalf<T>(T);

impl<T> NetworkSend for NalHalf<T>
where
    T: UdpSend,
{
    async fn send_to(&mut self, data: &[u8], addr: Address) -> Result<(), Error> {
        let addr = addr.udp().ok_or(ErrorCode::NoNetworkInterface)?;

        self.0.send(addr, data).await.map_err(map_err)
    }
}

impl<T> NetworkReceive for NalHalf<T>
where
    T: UdpReceive + Readable,
{
    async fn wait_available(&mut self) -> Result<(), Error> {
        self.0.readable().await.map_err(map_err)
    }

    async fn recv_from(&mut self, buffer: &mut [u8]) -> Result<(usize, Address), Error> {
        let (size, addr) = self.0.receive(buffer).await.map_err(map_err)?;

        Ok((size, Address::Udp(addr)))
    }
}

/// The multicast group membership of a `NalSocket`.
pub struct NalMulticast<M4, M6> {
    m4: M4,
    ipv4_interface: Ipv4Addr,
    m6: M6,
    ipv6_interface: u32,
}

impl<M4, M6> NetworkMulticast for NalMulticast<M4, M6>
where
    M4: MulticastV4,
    M6: MulticastV6,
{
    async fn join(&mut self, addr: IpAddr) -> Result<(), Error> {
        match addr {
            IpAddr::V4(addr) => self
                .m4
                .join_v4(addr, self.ipv4_interface)
                .await
                .map_err(map_err),
            IpAddr::V6(addr) => self
                .m6
                .join_v6(addr, self.ipv6_interface)
                .await
                .map_err(map_err),
        }
    }

    async fn leave(&mut self, addr: IpAddr) -> Result<(), Error> {
        match addr {
            IpAddr::V4(addr) => self
                .m4
                .leave_v4(addr, self.ipv4_interface)
                .await
                .map_err(map_err),
            IpAddr::V6(addr) => self
                .m6
                .leave_v6(addr, self.ipv6_interface)
                .await
                .map_err(map_err),
        }
    }
}

pub(crate) fn map_err<E: Debug>(e: E) -> Error {
    warn!("Network error: {:?}", debug2format!(e));

    ErrorCode::StdIoError.into()
}

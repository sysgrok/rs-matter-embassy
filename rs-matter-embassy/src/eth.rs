//! Ethernet: an Embassy Matter stack running over an Ethernet network.

use core::cell::Cell;
use core::marker::PhantomData;
use core::pin::pin;

use embassy_net::driver::{Capabilities, Driver, DriverBuf, HardwareAddress, LinkState, RxWanted};
use embassy_net::iface::Iface;
use embassy_net::Pool;

use crate::enet::{create_enet_stack, EnetNetif, EnetPool, EnetStack, EnetStackStorage};
#[cfg(feature = "tcp")]
use crate::enet::{EnetMatterTcpBuffers, EnetMatterTcpTransportBuffers};
use crate::matter::crypto::Rng;
use crate::matter::dm::clusters::eth_diag::EthDiag;
use crate::matter::dm::clusters::gen_diag::InterfaceTypeEnum;
use crate::matter::dm::networks::eth::EthNetwork;
use crate::matter::error::Error;
use crate::matter::transport::network::mdns::builtin::BuiltinMdns;
#[cfg(feature = "tcp")]
use crate::matter::transport::network::tcp::TcpConfig;
use crate::matter::utils::futures::{select, Coalesce};
use crate::matter::utils::init::{init, Init};
use crate::matter::utils::sync::blocking::Mutex;
use crate::matter::utils::sync::IfMutex;
use crate::stack::eth::{EthernetDriver, EthernetTask};
use crate::stack::network::{Embedding, EthStackNetwork, StackNetwork};
use crate::stack::MatterStack;

/// The network of a `MatterStack` that runs Ethernet with [`EmbassyEthernet`]: the networks
/// store `S`, and the `embassy-net` context, which the stack holds so that it can be
/// statically allocated.
pub struct EmbassyEthNetwork<S = EthNetwork<'static>>(PhantomData<fn() -> S>);

impl<S> StackNetwork for EmbassyEthNetwork<S>
where
    S: EthStackNetwork<Context = ()>,
{
    type Store = S::Store;
    type Context = EmbassyNetContext;
}

impl<S> EthStackNetwork for EmbassyEthNetwork<S> where S: EthStackNetwork<Context = ()> {}

/// An Ethernet driver of the stack over `embassy-net` and an `embassy-net` driver `D`, with
/// the packet buffers of the `embassy-net` stack coming from the pool `P`.
pub struct EmbassyEthernet<'a, D, R, P: 'static> {
    device: D,
    pool: EnetPool<P>,
    rand: R,
    #[cfg(feature = "tcp")]
    tcp_config: &'a TcpConfig,
    context: &'a EmbassyNetContext,
}

impl<'a, D, R, P> EmbassyEthernet<'a, D, R, P>
where
    P: Pool,
{
    #[cfg(feature = "tcp")]
    const DEFAULT_TCP_CONFIG: TcpConfig = TcpConfig::new();

    /// Create a new instance of the `EmbassyEthernet` type.
    ///
    /// # Arguments
    /// - `device` - The `embassy-net` driver
    /// - `pool` - The packet pool of the `embassy-net` stack; see [`EnetPool`]
    /// - `rand` - The random number generator
    /// - `stack` - The Matter stack
    pub fn new<const B: usize, S, E>(
        device: D,
        pool: &'static mut P,
        rand: R,
        stack: &'a MatterStack<'a, B, S, E>,
    ) -> Self
    where
        S: StackNetwork<Context = EmbassyNetContext>,
        E: Embedding,
    {
        Self::wrap(device, pool, rand, stack.context())
    }

    /// Wrap an `embassy-net` driver with the `EmbassyEthernet` type.
    pub fn wrap(device: D, pool: &'static mut P, rand: R, context: &'a EmbassyNetContext) -> Self {
        Self {
            device,
            pool: EnetPool::new(pool),
            rand,
            #[cfg(feature = "tcp")]
            tcp_config: &Self::DEFAULT_TCP_CONFIG,
            context,
        }
    }

    /// Configure the Matter-over-TCP timeouts and keep-alive settings for this interface.
    #[cfg(feature = "tcp")]
    pub const fn with_tcp_config(mut self, tcp_config: &'a TcpConfig) -> Self {
        self.tcp_config = tcp_config;
        self
    }
}

impl<D, R, P> EthernetDriver for EmbassyEthernet<'_, D, R, P>
where
    D: Driver,
    R: Rng,
    P: Pool,
{
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: EthernetTask,
    {
        let mut storage = self.context.storage.lock().await;
        let mut mdns = self.context.mdns.lock().await;

        let mdns = &mut *mdns;

        let mut seed = [0; core::mem::size_of::<u64>()];
        self.rand.fill_bytes(&mut seed);

        let counters = EnetCounters::new();
        let mut driver = EnetCountingDriver::new(&mut self.device, &counters);

        let (iface, mut runner) = create_enet_stack(
            &mut driver,
            &mut self.pool,
            &mut storage,
            u64::from_le_bytes(seed),
        )?;
        let _iface_guard = scopeguard::guard(iface, remove_iface);

        #[cfg(feature = "tcp")]
        let net_stack = EnetStack::new_with_tcp(
            iface,
            &self.context.tcp.tcp_buffers,
            &self.context.tcp.matter_tcp_buffers,
            self.tcp_config,
        );
        #[cfg(not(feature = "tcp"))]
        let net_stack = EnetStack::new(iface);
        let netif = EnetNetif::new(iface, InterfaceTypeEnum::Ethernet);
        let eth_diag = EnetEthDiag::new(iface, &counters);

        let mut main = pin!(task.run(&net_stack, &netif, &eth_diag, mdns));
        let mut run = pin!(async {
            runner.run().await;
            #[allow(unreachable_code)]
            Ok(())
        });

        select(&mut main, &mut run).coalesce().await
    }
}

/// Remove the interface of a run from its stack, once the Matter stack is done with it: the
/// stack is never dropped, so this is what gives back the packet buffers the interface still
/// holds.
pub(crate) fn remove_iface(iface: Iface<'_>) {
    iface.stack().remove_iface(iface.handle());
}

#[cfg(feature = "tcp")]
pub(crate) struct EnetTcpContext {
    pub(crate) tcp_buffers: EnetMatterTcpBuffers,
    pub(crate) matter_tcp_buffers: EnetMatterTcpTransportBuffers,
}

#[cfg(feature = "tcp")]
impl EnetTcpContext {
    const fn new() -> Self {
        Self {
            tcp_buffers: EnetMatterTcpBuffers::new(),
            matter_tcp_buffers: EnetMatterTcpTransportBuffers::new(),
        }
    }

    fn init() -> impl Init<Self> {
        init!(Self {
            tcp_buffers <- EnetMatterTcpBuffers::init(),
            matter_tcp_buffers <- EnetMatterTcpTransportBuffers::init(),
        })
    }
}

#[cfg(not(feature = "tcp"))]
pub(crate) struct EnetTcpContext {}

#[cfg(not(feature = "tcp"))]
impl EnetTcpContext {
    const fn new() -> Self {
        Self {}
    }

    fn init() -> impl Init<Self> {
        init!(Self {})
    }
}

/// A context (storage) for the network layer of the Matter stack.
pub struct EmbassyNetContext {
    pub(crate) tcp: EnetTcpContext,
    pub(crate) storage: IfMutex<EnetStackStorage>,
    pub(crate) mdns: IfMutex<BuiltinMdns>,
}

impl EmbassyNetContext {
    /// Create a new instance of the `EmbassyNetContext` type.
    pub const fn new() -> Self {
        Self {
            tcp: EnetTcpContext::new(),
            storage: IfMutex::new(EnetStackStorage::new()),
            mdns: IfMutex::new(BuiltinMdns::new()),
        }
    }

    /// Return an in-place initializer for the `EmbassyNetContext` type.
    pub fn init() -> impl Init<Self> {
        init!(Self {
            tcp <- EnetTcpContext::init(),
            // `EnetStackStorage` is a `MaybeUninit`, so there is nothing to initialize in place
            storage <- IfMutex::init(EnetStackStorage::new()),
            mdns <- IfMutex::init(BuiltinMdns::init()),
        })
    }
}

impl Default for EmbassyNetContext {
    fn default() -> Self {
        Self::new()
    }
}

impl Embedding for EmbassyNetContext {
    const INIT: Self = Self::new();

    fn init() -> impl Init<Self> {
        EmbassyNetContext::init()
    }
}

/// An `EthDiag` implementation for `embassy-net`.
///
/// Advertise what it reports with `eth_diag(carrier_detect, packet_counts)` on the root endpoint.
pub struct EnetEthDiag<'a, 'd> {
    iface: Iface<'d>,
    counters: &'a EnetCounters,
}

impl<'a, 'd> EnetEthDiag<'a, 'd> {
    /// Create a new instance of the `EnetEthDiag` type.
    pub const fn new(iface: Iface<'d>, counters: &'a EnetCounters) -> Self {
        Self { iface, counters }
    }
}

impl EthDiag for EnetEthDiag<'_, '_> {
    fn carrier_detect(&self) -> Result<Option<bool>, Error> {
        Ok(Some(self.iface.is_link_up()))
    }

    fn packet_rx_count(&self) -> Result<u64, Error> {
        Ok(self.counters.rx.lock(Cell::get))
    }

    fn packet_tx_count(&self) -> Result<u64, Error> {
        Ok(self.counters.tx.lock(Cell::get))
    }

    fn reset_counts(&self) -> Result<(), Error> {
        self.counters.rx.lock(|count| count.set(0));
        self.counters.tx.lock(|count| count.set(0));

        Ok(())
    }
}

/// The packet counts of an `EnetCountingDriver`.
pub struct EnetCounters {
    rx: Mutex<Cell<u64>>,
    tx: Mutex<Cell<u64>>,
}

impl EnetCounters {
    /// Create a new instance of the `EnetCounters` type.
    pub const fn new() -> Self {
        Self {
            rx: Mutex::new(Cell::new(0)),
            tx: Mutex::new(Cell::new(0)),
        }
    }

    fn inc(count: &Mutex<Cell<u64>>) {
        count.lock(|count| count.set(count.get().wrapping_add(1)));
    }
}

impl Default for EnetCounters {
    fn default() -> Self {
        Self::new()
    }
}

/// An `embassy-net` driver wrapper which counts the packets received and transmitted by the stack.
pub struct EnetCountingDriver<'a, D> {
    driver: D,
    counters: &'a EnetCounters,
}

impl<'a, D> EnetCountingDriver<'a, D> {
    /// Create a new instance of the `EnetCountingDriver` type.
    pub const fn new(driver: D, counters: &'a EnetCounters) -> Self {
        Self { driver, counters }
    }
}

impl<D> Driver for EnetCountingDriver<'_, D>
where
    D: Driver,
{
    fn capabilities(&self) -> Capabilities {
        self.driver.capabilities()
    }

    fn hardware_address(&self) -> HardwareAddress {
        self.driver.hardware_address()
    }

    fn link_state(&mut self) -> LinkState {
        self.driver.link_state()
    }

    fn register_waker(
        &mut self,
        waker: &core::task::Waker,
    ) -> Result<(), embassy_net::driver::NotSupported> {
        self.driver.register_waker(waker)
    }

    fn rx_wanted(&mut self) -> RxWanted {
        self.driver.rx_wanted()
    }

    fn rx_give(&mut self, buf: DriverBuf) {
        self.driver.rx_give(buf)
    }

    fn receive(&mut self) -> Option<DriverBuf> {
        let buf = self.driver.receive();

        if buf.is_some() {
            EnetCounters::inc(&self.counters.rx);
        }

        buf
    }

    fn can_transmit(&mut self) -> bool {
        self.driver.can_transmit()
    }

    fn transmit(&mut self, buf: DriverBuf) -> Result<(), DriverBuf> {
        self.driver.transmit(buf)?;

        EnetCounters::inc(&self.counters.tx);

        Ok(())
    }

    fn set_multicast_filter(&mut self, addrs: &[[u8; 6]]) {
        self.driver.set_multicast_filter(addrs)
    }
}

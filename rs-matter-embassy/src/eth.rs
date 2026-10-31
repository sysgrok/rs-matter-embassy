//! Eternet: Type aliases for an Embassy Matter stack running over an Ethernet network.

use core::cell::Cell;
use core::mem::MaybeUninit;
use core::pin::pin;
use core::task::Context;

use embassy_net::driver::{Capabilities, Driver, HardwareAddress, LinkState, RxToken, TxToken};

use crate::enet::{
    create_enet_stack, EnetMatterStackResources, EnetMatterUdpBuffers, EnetNetif, EnetStack,
};
use crate::matter::crypto::Rng;
use crate::matter::dm::clusters::eth_diag::EthDiag;
use crate::matter::dm::clusters::gen_diag::InterfaceTypeEnum;
use crate::matter::error::Error;
use crate::matter::transport::network::mdns::builtin::BuiltinMdns;
use crate::matter::utils::futures::{select, Coalesce};
use crate::matter::utils::init::{init, Init};
use crate::matter::utils::sync::blocking::Mutex;
use crate::matter::utils::sync::IfMutex;
use crate::stack::eth::{Eth, Ethernet, EthernetTask};
use crate::stack::network::{Embedding, Network};
use crate::stack::MatterStack;

/// A type alias for an Embassy Matter stack running over an Ethernet network.
///
/// The difference between this and `EthMatterStack` is that all resources necessary for the
/// operation of `embassy-net` are pre-allocated inside the stack.
pub type EmbassyEthMatterStack<'a, const B: usize, E = ()> = MatterStack<'a, B, EmbassyEth<E>>;

/// A type alias for an Embassy implementation of the `Network` trait for a Matter stack running over
/// Ethernet.
pub type EmbassyEth<E = ()> = Eth<EmbassyNet<E>>;

/// An embedding of the `embassy-net` context for the `Eth` network type from `rs-matter-stack`.
///
/// Allows the memory of this context to be statically allocated and cost-initialized.
pub struct EmbassyNet<E = ()> {
    net_context: EmbassyNetContext,
    embedding: E,
}

impl<E> EmbassyNet<E>
where
    E: Embedding,
{
    /// Creates a new instance of the `EspGatt` embedding.
    #[allow(clippy::large_stack_frames)]
    #[inline(always)]
    const fn new() -> Self {
        Self {
            net_context: EmbassyNetContext::new(),
            embedding: E::INIT,
        }
    }

    /// Return an in-place initializer for the `EspGatt` embedding.
    fn init() -> impl Init<Self> {
        init!(Self {
            net_context <- EmbassyNetContext::init(),
            embedding <- E::init(),
        })
    }

    pub fn net_context(&self) -> &EmbassyNetContext {
        &self.net_context
    }

    /// Return a reference to the embedding.
    pub fn embedding(&self) -> &E {
        &self.embedding
    }
}

impl<E> Embedding for EmbassyNet<E>
where
    E: Embedding,
{
    const INIT: Self = Self::new();

    fn init() -> impl Init<Self> {
        EmbassyNet::init()
    }
}

/// A trait representing a task that needs access to the Ethernet driver to perform its work
pub trait EthernetDriverTask {
    /// Run the task with the given Ethernet driver
    async fn run<D>(&mut self, driver: D) -> Result<(), Error>
    where
        D: embassy_net::driver::Driver;
}

impl<T> EthernetDriverTask for &mut T
where
    T: EthernetDriverTask,
{
    async fn run<D>(&mut self, driver: D) -> Result<(), Error>
    where
        D: embassy_net::driver::Driver,
    {
        (*self).run(driver).await
    }
}

/// A trait for running a task within a context where the Ethernet driver is initialized and operable
pub trait EthernetDriver {
    /// Setup the Ethernet driver and run the given task with it
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: EthernetDriverTask;
}

impl<T> EthernetDriver for &mut T
where
    T: EthernetDriver,
{
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: EthernetDriverTask,
    {
        (*self).run(task).await
    }
}

/// An Ethernet driver provider that uses a pre-existing, already created Ethernet driver,
/// rather than creating it when the Matter stack needs it.
pub struct PreexistingEthDriver<D>(D);

impl<D> PreexistingEthDriver<D> {
    /// Create a new instance of the `PreexistingEthDriver` type.
    pub const fn new(enet_driver: D) -> Self {
        Self(enet_driver)
    }
}

impl<D> EthernetDriver for PreexistingEthDriver<D>
where
    D: embassy_net::driver::Driver,
{
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: EthernetDriverTask,
    {
        task.run(&mut self.0).await
    }
}

/// An `Ethernet` trait implementation for `embassy-net`.
pub struct EmbassyEthernet<'a, T, R> {
    driver: T,
    rand: R,
    context: &'a EmbassyNetContext,
}

impl<'a, T, R> EmbassyEthernet<'a, T, R>
where
    T: EthernetDriver,
{
    /// Create a new instance of the `EmbassyEthernet` type.
    pub fn new<const B: usize, E>(
        driver: T,
        rand: R,
        stack: &'a EmbassyEthMatterStack<'a, B, E>,
    ) -> Self
    where
        E: Embedding,
    {
        Self::wrap(driver, rand, stack.network().embedding().net_context())
    }

    /// Wrap an existing `Ethernet` driver with the `EmbassyEthernet` type.
    pub fn wrap(driver: T, rand: R, context: &'a EmbassyNetContext) -> Self {
        Self {
            driver,
            rand,
            context,
        }
    }
}

impl<T, R> Ethernet for EmbassyEthernet<'_, T, R>
where
    T: EthernetDriver,
    R: Rng,
{
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: EthernetTask,
    {
        struct EthernetDriverTaskImpl<'a, 'b, A, R> {
            context: &'a EmbassyNetContext,
            task: A,
            rand: &'b mut R,
        }

        impl<A, R> EthernetDriverTask for EthernetDriverTaskImpl<'_, '_, A, R>
        where
            A: EthernetTask,
            R: Rng,
        {
            async fn run<D>(&mut self, driver: D) -> Result<(), Error>
            where
                D: embassy_net::driver::Driver,
            {
                let mut resources = self.context.resources.lock().await;
                let mut mdns = self.context.mdns.lock().await;

                let resources = &mut *resources;
                let mdns = &mut *mdns;
                let buffers = &self.context.buffers;

                let mut seed = [0; core::mem::size_of::<u64>()];
                self.rand.fill_bytes(&mut seed);

                let counters = EnetCounters::new();

                let (stack, mut runner) = create_enet_stack(
                    EnetCountingDriver::new(driver, &counters),
                    u64::from_le_bytes(seed),
                    resources,
                );

                let net_stack = EnetStack::new(stack, buffers);
                let netif = EnetNetif::new(stack, InterfaceTypeEnum::Ethernet);
                let eth_diag = EnetEthDiag::new(stack, &counters);

                let mut main = pin!(self.task.run(&net_stack, &netif, &eth_diag, mdns));
                let mut run = pin!(async {
                    runner.run().await;
                    #[allow(unreachable_code)]
                    Ok(())
                });

                select(&mut main, &mut run).coalesce().await
            }
        }

        self.driver
            .run(EthernetDriverTaskImpl {
                context: self.context,
                task,
                rand: &mut self.rand,
            })
            .await
    }
}

/// A context (storage) for the network layer of the Matter stack.
pub struct EmbassyNetContext {
    pub(crate) buffers: EnetMatterUdpBuffers,
    pub(crate) resources: IfMutex<EnetMatterStackResources>,
    pub(crate) mdns: IfMutex<BuiltinMdns>,
}

impl EmbassyNetContext {
    /// Create a new instance of the `EmbassyNetContext` type.
    pub const fn new() -> Self {
        Self {
            buffers: EnetMatterUdpBuffers::new(),
            resources: IfMutex::new(EnetMatterStackResources::new()),
            mdns: IfMutex::new(BuiltinMdns::new()),
        }
    }

    /// Return an in-place initializer for the `EmbassyNetContext` type.
    pub fn init() -> impl Init<Self> {
        init!(Self {
            // TODO: Implement init constructor for `UdpBuffers`
            buffers: EnetMatterUdpBuffers::new(),
            // Note: below will break if `HostResources` stops being a bunch of `MaybeUninit`s
            resources <- IfMutex::init(unsafe { MaybeUninit::<EnetMatterStackResources>::uninit().assume_init() }),
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
    stack: embassy_net::Stack<'d>,
    counters: &'a EnetCounters,
}

impl<'a, 'd> EnetEthDiag<'a, 'd> {
    /// Create a new instance of the `EnetEthDiag` type.
    pub const fn new(stack: embassy_net::Stack<'d>, counters: &'a EnetCounters) -> Self {
        Self { stack, counters }
    }
}

impl EthDiag for EnetEthDiag<'_, '_> {
    fn carrier_detect(&self) -> Result<Option<bool>, Error> {
        Ok(Some(self.stack.is_link_up()))
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
    type RxToken<'a>
        = CountingRxToken<'a, D::RxToken<'a>>
    where
        Self: 'a;

    type TxToken<'a>
        = CountingTxToken<'a, D::TxToken<'a>>
    where
        Self: 'a;

    fn receive(&mut self, cx: &mut Context) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        let counters = self.counters;

        self.driver.receive(cx).map(|(rx, tx)| {
            (
                CountingRxToken {
                    token: rx,
                    counters,
                },
                CountingTxToken {
                    token: tx,
                    counters,
                },
            )
        })
    }

    fn transmit(&mut self, cx: &mut Context) -> Option<Self::TxToken<'_>> {
        let counters = self.counters;

        self.driver
            .transmit(cx)
            .map(|token| CountingTxToken { token, counters })
    }

    fn link_state(&mut self, cx: &mut Context) -> LinkState {
        self.driver.link_state(cx)
    }

    fn capabilities(&self) -> Capabilities {
        self.driver.capabilities()
    }

    fn hardware_address(&self) -> HardwareAddress {
        self.driver.hardware_address()
    }
}

/// The RX token of `EnetCountingDriver`
pub struct CountingRxToken<'a, T> {
    token: T,
    counters: &'a EnetCounters,
}

impl<T> RxToken for CountingRxToken<'_, T>
where
    T: RxToken,
{
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        EnetCounters::inc(&self.counters.rx);

        self.token.consume(f)
    }
}

/// The TX token of `EnetCountingDriver`
pub struct CountingTxToken<'a, T> {
    token: T,
    counters: &'a EnetCounters,
}

impl<T> TxToken for CountingTxToken<'_, T>
where
    T: TxToken,
{
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        EnetCounters::inc(&self.counters.tx);

        self.token.consume(len, f)
    }
}

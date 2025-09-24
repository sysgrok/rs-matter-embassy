//! Eternet: Type aliases for an Embassy Matter stack running over an Ethernet network.

use core::mem::MaybeUninit;
use core::pin::pin;

use embassy_futures::select::select;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use rs_matter_stack::mdns::BuiltinMdns;

use crate::enet::{
    create_enet_stack, EnetMatterStackResources, EnetMatterUdpBuffers, EnetNetif, EnetStack,
};
use crate::matter::dm::clusters::gen_diag::InterfaceTypeEnum;
use crate::matter::error::Error;
use crate::matter::utils::init::{init, Init};
use crate::matter::utils::rand::Rand;
use crate::matter::utils::select::Coalesce;
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
pub struct EmbassyEthernet<'a, T> {
    driver: T,
    context: &'a EmbassyNetContext,
    rand: Rand,
}

impl<'a, T> EmbassyEthernet<'a, T>
where
    T: EthernetDriver,
{
    /// Create a new instance of the `EmbassyEthernet` type.
    pub fn new<const B: usize, E>(driver: T, stack: &'a EmbassyEthMatterStack<'a, B, E>) -> Self
    where
        E: Embedding + 'static,
    {
        Self::wrap(
            driver,
            stack.network().embedding().net_context(),
            stack.matter().rand(),
        )
    }

    /// Wrap an existing `Ethernet` driver with the `EmbassyEthernet` type.
    pub fn wrap(driver: T, context: &'a EmbassyNetContext, rand: Rand) -> Self {
        Self {
            driver,
            context,
            rand,
        }
    }
}

impl<T> Ethernet for EmbassyEthernet<'_, T>
where
    T: EthernetDriver,
{
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: EthernetTask,
    {
        struct EthernetDriverTaskImpl<'a, A> {
            rand: Rand,
            context: &'a EmbassyNetContext,
            task: A,
        }

        impl<A> EthernetDriverTask for EthernetDriverTaskImpl<'_, A>
        where
            A: EthernetTask,
        {
            async fn run<D>(&mut self, driver: D) -> Result<(), Error>
            where
                D: embassy_net::driver::Driver,
            {
                let mut resources = self.context.resources.lock().await;
                let resources = &mut *resources;
                let buffers = &self.context.buffers;

                let mut seed = [0; core::mem::size_of::<u64>()];
                (self.rand)(&mut seed);

                let (stack, mut runner) =
                    create_enet_stack(driver, u64::from_le_bytes(seed), resources);

                let net_stack = EnetStack::new(stack, buffers);
                let netif = EnetNetif::new(stack, InterfaceTypeEnum::Ethernet);

                let mut main = pin!(self.task.run(&net_stack, &netif, BuiltinMdns));
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
                rand: self.rand,
                context: self.context,
                task,
            })
            .await
    }
}

/// A context (storage) for the network layer of the Matter stack.
pub struct EmbassyNetContext {
    pub(crate) buffers: EnetMatterUdpBuffers,
    pub(crate) resources: IfMutex<CriticalSectionRawMutex, EnetMatterStackResources>,
}

impl EmbassyNetContext {
    /// Create a new instance of the `EmbassyNetContext` type.
    pub const fn new() -> Self {
        Self {
            buffers: EnetMatterUdpBuffers::new(),
            resources: IfMutex::new(EnetMatterStackResources::new()),
        }
    }

    /// Return an in-place initializer for the `EmbassyNetContext` type.
    pub fn init() -> impl Init<Self> {
        init!(Self {
            // TODO: Implement init constructor for `UdpBuffers`
            buffers: EnetMatterUdpBuffers::new(),
            // Note: below will break if `HostResources` stops being a bunch of `MaybeUninit`s
            resources <- IfMutex::init(unsafe { MaybeUninit::<EnetMatterStackResources>::uninit().assume_init() }),
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

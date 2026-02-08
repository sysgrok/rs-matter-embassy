use core::pin::pin;

use embassy_futures::select::select4;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_time::{Duration, Timer};

use openthread::sys::otRadioCaps;
use openthread::{OpenThread, Radio};

use crate::ble::{ControllerRef, TroubleBtpGattContext, TroubleBtpGattPeripheral};
use crate::matter::crypto::{CryptoRngCore, RngCore};
use crate::matter::dm::networks::wireless::Thread;
use crate::matter::error::Error;
use crate::matter::utils::init::{init, Init};
use crate::matter::utils::select::Coalesce;
use crate::matter::utils::sync::IfMutex;
use crate::ot::{to_matter_err, OtNetCtl, OtNetStack, OtPersist};
use crate::ot::{OtMatterResources, OtMdns, OtNetif};
use crate::stack::network::{Embedding, Network};
use crate::stack::persist::{KvBlobStore, SharedKvBlobStore};
use crate::stack::rand::RngAdaptor;
use crate::stack::wireless::{self, Gatt, GattTask};

use super::{BleDriver, BleDriverTask, BleDriverTaskImpl, EmbassyWirelessMatterStack};

#[cfg(feature = "esp")]
pub mod esp_thread;
#[cfg(feature = "nrf")]
pub mod nrf;

/// A type alias for an Embassy Matter stack running over Thread (and BLE, during commissioning).
///
/// The difference between this and the `ThreadMatterStack` is that all resources necessary for the
/// operation of `openthread` as well as the BLE controller and pre-allocated inside the stack.
pub type EmbassyThreadMatterStack<'a, const B: usize, E = ()> =
    EmbassyWirelessMatterStack<'a, B, Thread, OtNetContext, E>;

/// A trait representing a task that needs access to the Thread radio to perform its work
pub trait ThreadDriverTask {
    /// Run the task with the given Thread radio
    async fn run<R>(&mut self, radio: R) -> Result<(), Error>
    where
        R: Radio;
}

impl<T> ThreadDriverTask for &mut T
where
    T: ThreadDriverTask,
{
    async fn run<R>(&mut self, radio: R) -> Result<(), Error>
    where
        R: Radio,
    {
        (*self).run(radio).await
    }
}

/// A trait representing a task that needs access to the Thread radio,
/// as well as to the BLE controller to perform its work
pub trait ThreadCoexDriverTask {
    /// Run the task with the given Thread radio and BLE controller
    async fn run<R, B>(&mut self, radio: R, ble_ctl: B) -> Result<(), Error>
    where
        R: Radio,
        B: trouble_host::Controller;
}

impl<T> ThreadCoexDriverTask for &mut T
where
    T: ThreadCoexDriverTask,
{
    async fn run<R, B>(&mut self, radio: R, ble_ctl: B) -> Result<(), Error>
    where
        R: Radio,
        B: trouble_host::Controller,
    {
        (*self).run(radio, ble_ctl).await
    }
}

/// A trait for running a task within a context where the Thread radio is initialized and operable
pub trait ThreadDriver {
    /// Setup the Thread radio and run the given task with it
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: ThreadDriverTask;
}

impl<T> ThreadDriver for &mut T
where
    T: ThreadDriver,
{
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: ThreadDriverTask,
    {
        (*self).run(task).await
    }
}

/// A trait for running a task within a context where the Thread radio - as well as the BLE controller - are initialized and operable
pub trait ThreadCoexDriver {
    /// Setup the Thread radio and the BLE controller and run the given task with these
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: ThreadCoexDriverTask;
}

impl<T> ThreadCoexDriver for &mut T
where
    T: ThreadCoexDriver,
{
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: ThreadCoexDriverTask,
    {
        (*self).run(task).await
    }
}

/// A Thread radio provider that uses a pre-existing, already created Thread radio
/// as well as an already created BLE controller rather than creating these when the Matter stack needs them.
pub struct PreexistingThreadDriver<R, B>(R, B);

impl<R, B> PreexistingThreadDriver<R, B> {
    /// Create a new instance of the `PreexistingThreadRadio` type.
    pub const fn new(radio: R, ble: B) -> Self {
        Self(radio, ble)
    }
}

impl<R, B> ThreadDriver for PreexistingThreadDriver<R, B>
where
    R: Radio,
{
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: ThreadDriverTask,
    {
        task.run(&mut self.0).await
    }
}

impl<R, B> ThreadCoexDriver for PreexistingThreadDriver<R, B>
where
    R: Radio,
    B: trouble_host::Controller,
{
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: ThreadCoexDriverTask,
    {
        task.run(&mut self.0, ControllerRef::new(&self.1)).await
    }
}

impl<R, B> BleDriver for PreexistingThreadDriver<R, B>
where
    B: trouble_host::Controller,
{
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: BleDriverTask,
    {
        task.run(ControllerRef::new(&self.1)).await
    }
}

/// A `Wireless` trait implementation for `openthread`'s Thread stack.
pub struct EmbassyThread<'a, T, S, R> {
    driver: T,
    ieee_eui64: [u8; 8],
    store: &'a SharedKvBlobStore<'a, S>,
    context: &'a OtNetContext,
    ble_context: &'a TroubleBtpGattContext<CriticalSectionRawMutex>,
    use_ble_random_addr: bool,
    rand: R,
    radio_caps: Option<otRadioCaps>,
}

impl<'a, T, S, R> EmbassyThread<'a, T, S, R>
where
    T: ThreadDriver,
    S: KvBlobStore,
    R: CryptoRngCore + Copy,
{
    /// Create a new instance of the `EmbassyThread` type.
    pub fn new<const B: usize, E>(
        driver: T,
        rand: R,
        ieee_eui64: [u8; 8],
        store: &'a SharedKvBlobStore<'a, S>,
        stack: &'a EmbassyThreadMatterStack<'a, B, E>,
        use_ble_random_addr: bool,
    ) -> Self
    where
        E: Embedding + 'static,
    {
        Self::wrap(
            driver,
            rand,
            ieee_eui64,
            store,
            stack.network().embedding().net_context(),
            stack.network().embedding().ble_context(),
            use_ble_random_addr,
        )
    }

    /// Wrap an existing `ThreadDriver` with the given parameters.
    pub fn wrap(
        driver: T,
        rand: R,
        ieee_eui64: [u8; 8],
        store: &'a SharedKvBlobStore<'a, S>,
        context: &'a OtNetContext,
        ble_context: &'a TroubleBtpGattContext<CriticalSectionRawMutex>,
        use_ble_random_addr: bool,
    ) -> Self {
        Self {
            driver,
            ieee_eui64,
            store,
            context,
            ble_context,
            rand,
            use_ble_random_addr,
            radio_caps: None,
        }
    }

    /// Set hardware-specific radio capabilities.
    ///
    /// If not set, OpenThread uses the default (ACK_TIMEOUT only).
    /// For ESP32, use `OT_RADIO_CAPS_ACK_TIMEOUT | OT_RADIO_CAPS_CSMA_BACKOFF`.
    pub fn with_radio_caps(mut self, caps: otRadioCaps) -> Self {
        self.radio_caps = Some(caps);
        self
    }
}

impl<T, S, R> wireless::Thread for EmbassyThread<'_, T, S, R>
where
    T: ThreadDriver,
    S: KvBlobStore,
    R: CryptoRngCore + Copy,
{
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: wireless::ThreadTask,
    {
        self.driver
            .run(ThreadDriverTaskImpl {
                ieee_eui64: self.ieee_eui64,
                rand: self.rand,
                store: self.store,
                context: self.context,
                task,
                radio_caps: self.radio_caps,
            })
            .await
    }
}

impl<T, S, R> wireless::ThreadCoex for EmbassyThread<'_, T, S, R>
where
    T: ThreadCoexDriver,
    S: KvBlobStore,
    R: CryptoRngCore + Copy,
{
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: wireless::ThreadCoexTask,
    {
        self.driver
            .run(ThreadCoexDriverTaskImpl {
                ieee_eui64: self.ieee_eui64,
                rand: self.rand,
                store: self.store,
                context: self.context,
                ble_context: self.ble_context,
                use_ble_random_addr: self.use_ble_random_addr,
                task,
                radio_caps: self.radio_caps,
            })
            .await
    }
}

impl<T, S, R> Gatt for EmbassyThread<'_, T, S, R>
where
    T: BleDriver,
    S: KvBlobStore,
    R: RngCore + Copy,
{
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: GattTask,
    {
        self.driver
            .run(BleDriverTaskImpl {
                task,
                rand: self.use_ble_random_addr.then_some(self.rand),
                context: self.ble_context,
            })
            .await
    }
}

/// A network context for the `EmbassyThread` type.
pub struct OtNetContext {
    resources: IfMutex<NoopRawMutex, OtMatterResources>,
}

impl OtNetContext {
    /// Create a new instance of the `OtNetContext` type.
    pub const fn new() -> Self {
        Self {
            resources: IfMutex::new(OtMatterResources::new()),
        }
    }

    /// Return an in-place initializer for the `OtNetContext` type.
    pub fn init() -> impl Init<Self> {
        init!(Self {
            resources <- IfMutex::init(OtMatterResources::init()),
        })
    }
}

impl Default for OtNetContext {
    fn default() -> Self {
        Self::new()
    }
}

impl Embedding for OtNetContext {
    const INIT: Self = Self::new();

    fn init() -> impl Init<Self> {
        OtNetContext::init()
    }
}

struct ThreadDriverTaskImpl<'a, A, S, C> {
    ieee_eui64: [u8; 8],
    rand: C,
    store: &'a SharedKvBlobStore<'a, S>,
    context: &'a OtNetContext,
    task: A,
    radio_caps: Option<otRadioCaps>,
}

impl<A, S, C> ThreadDriverTask for ThreadDriverTaskImpl<'_, A, S, C>
where
    A: wireless::ThreadTask,
    S: KvBlobStore,
    C: CryptoRngCore + Copy,
{
    async fn run<R>(&mut self, radio: R) -> Result<(), Error>
    where
        R: Radio,
    {
        let mut resources = self.context.resources.lock().await;
        let resources = &mut *resources;

        let persister = OtPersist::new(&mut resources.settings_buf, self.store);
        persister.load().await?;

        let mut settings = persister.settings();
        let mut rand = RngAdaptor::new(self.rand);

        let ot = OpenThread::new_with_udp_srp(
            self.ieee_eui64,
            &mut rand,
            &mut settings,
            &mut resources.ot,
            &mut resources.udp,
            &mut resources.srp,
        )
        .map_err(to_matter_err)?;

        if let Some(caps) = self.radio_caps {
            ot.set_radio_caps(caps);
        }

        let net_ctl = OtNetCtl::new(ot.clone());
        let net_stack = OtNetStack::new(ot.clone());
        let netif = OtNetif::new(ot.clone());
        let mut mdns = OtMdns::new(ot.clone());

        let mut main = pin!(self.task.run(&net_stack, &netif, &net_ctl, &mut mdns));
        let mut radio = pin!(async {
            ot.run(radio).await;
            #[allow(unreachable_code)]
            Ok(())
        });
        let mut persist = pin!(persister.run());
        ot.enable_ipv6(true).map_err(to_matter_err)?;
        ot.srp_autostart().map_err(to_matter_err)?;
        info!("SRP autostart enabled");

        // Note: rx_when_idle is set by OtNetCtl::connect() after device attaches.
        // OtMdns::run() waits for rx_when_idle=true before registering SRP services.

        let mut srp_diag = pin!(log_srp_state(&ot));

        let result = select4(&mut main, &mut radio, &mut persist, &mut srp_diag)
            .coalesce()
            .await;

        let _ = ot.enable_thread(false);
        let _ = ot.srp_stop();
        let _ = ot.enable_ipv6(false);

        result
    }
}

struct ThreadCoexDriverTaskImpl<'a, A, S, C> {
    ieee_eui64: [u8; 8],
    rand: C,
    store: &'a SharedKvBlobStore<'a, S>,
    context: &'a OtNetContext,
    ble_context: &'a TroubleBtpGattContext<CriticalSectionRawMutex>,
    task: A,
    use_ble_random_addr: bool,
    radio_caps: Option<otRadioCaps>,
}

impl<A, S, C> ThreadCoexDriverTask for ThreadCoexDriverTaskImpl<'_, A, S, C>
where
    A: wireless::ThreadCoexTask,
    S: KvBlobStore,
    C: CryptoRngCore + Copy,
{
    async fn run<R, B>(&mut self, radio: R, ble_ctl: B) -> Result<(), Error>
    where
        R: Radio,
        B: trouble_host::Controller,
    {
        let mut resources = self.context.resources.lock().await;
        let resources = &mut *resources;

        let persister = OtPersist::new(&mut resources.settings_buf, self.store);
        persister.load().await?;

        let mut settings = persister.settings();
        let mut rand = RngAdaptor::new(self.rand);

        let ot = OpenThread::new_with_udp_srp(
            self.ieee_eui64,
            &mut rand,
            &mut settings,
            &mut resources.ot,
            &mut resources.udp,
            &mut resources.srp,
        )
        .map_err(to_matter_err)?;

        if let Some(caps) = self.radio_caps {
            ot.set_radio_caps(caps);
        }

        let net_ctl = OtNetCtl::new(ot.clone());
        let net_stack = OtNetStack::new(ot.clone());
        let netif = OtNetif::new(ot.clone());
        let mut mdns = OtMdns::new(ot.clone());
        let mut peripheral = TroubleBtpGattPeripheral::new(
            ble_ctl,
            self.use_ble_random_addr.then_some(self.rand),
            self.ble_context,
        );

        let mut main =
            pin!(self
                .task
                .run(&net_stack, &netif, &net_ctl, &mut mdns, &mut peripheral));
        let mut radio = pin!(async {
            ot.run(radio).await;
            #[allow(unreachable_code)]
            Ok(())
        });
        let mut persist = pin!(persister.run());
        ot.enable_ipv6(true).map_err(to_matter_err)?;
        ot.srp_autostart().map_err(to_matter_err)?;
        info!("SRP autostart enabled");

        // Note: rx_when_idle is set by OtNetCtl::connect() after device attaches.
        // OtMdns::run() waits for rx_when_idle=true before registering SRP services.

        let mut srp_diag = pin!(log_srp_state(&ot));

        let result = select4(&mut main, &mut radio, &mut persist, &mut srp_diag)
            .coalesce()
            .await;

        let _ = ot.enable_thread(false);
        let _ = ot.srp_stop();
        let _ = ot.enable_ipv6(false);

        result
    }
}

/// Log SRP client diagnostics (server address, service counts and states).
/// Runs in a loop, logging every 10 seconds until cancelled.
async fn log_srp_state(ot: &OpenThread<'_>) -> Result<(), Error> {
    let mut tick = 0u32;
    loop {
        Timer::after(Duration::from_secs(10)).await;
        tick += 10;

        let server_addr = ot.srp_server_addr().ok().flatten();

        let mut total = 0u8;
        let mut registered = 0u8;
        let mut adding = 0u8;
        let mut removing = 0u8;
        let _ = ot.srp_services(|svc| {
            if let Some((_, state, _)) = svc {
                total += 1;
                match state {
                    openthread::SrpState::Registered => registered += 1,
                    openthread::SrpState::Adding | openthread::SrpState::ToAdd => adding += 1,
                    openthread::SrpState::Removing | openthread::SrpState::ToRemove => {
                        removing += 1
                    }
                    _ => {}
                }
            }
        });

        if let Some(addr) = server_addr {
            info!(
                "SRP[{}s]: srv={}, reg={}/{}, add={}, rm={}",
                tick, addr, registered, total, adding, removing
            );
        } else {
            warn!("SRP[{}s]: NO SERVER, svc={}", tick, total);
        }
    }
}

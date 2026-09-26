use core::pin::pin;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_futures::select::{select, select3, select4, Either, Either3, Either4};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};

use openthread::{DeviceRole, OpenThread, Radio};

use rs_matter_stack::matter::persist::KvBlobStoreAccess;

use crate::ble::{BtpGattContext, BtpGattPeripheral, Controller, ControllerRef};
use crate::matter::crypto::{CryptoRng, Rng};
use crate::matter::dm::networks::wireless::Thread;
use crate::matter::error::Error;
use crate::matter::utils::init::{init, Init};
use crate::matter::utils::select::Coalesce;
use crate::matter::utils::sync::IfMutex;
use crate::ot::{to_matter_err, OtNetCtl, OtNetStack, OtPersist};
use crate::ot::{OtMatterResources, OtMdns, OtNetif};
use crate::stack::network::{Embedding, Network};
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
        B: Controller;
}

impl<T> ThreadCoexDriverTask for &mut T
where
    T: ThreadCoexDriverTask,
{
    async fn run<R, B>(&mut self, radio: R, ble_ctl: B) -> Result<(), Error>
    where
        R: Radio,
        B: Controller,
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
    B: Controller,
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
    B: Controller,
{
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: BleDriverTask,
    {
        task.run(ControllerRef::new(&self.1)).await
    }
}

/// Sleepy End Device (SED) configuration for the Thread stack.
///
/// Setting `rx_on_when_idle = false` makes OpenThread park the receiver and
/// wake only to data-poll its parent, so both the radio and (with esp-rtos
/// auto light-sleep) the CPU can sleep. The poll period is the primary
/// radio-duty-cycle / battery-life knob.
///
/// The device runs on two periods: a short `active_poll_period_ms` for
/// `active_hold` after boot or a [`SedHandle::request_active`] nudge
/// (commissioning, button presses, ICD stay-active), decaying to the *idle*
/// period once quiet. The idle period is not fixed here - it is owned by the
/// [`SedHandle`], so the application can change it at runtime (e.g. to follow
/// the device's ICD operating mode).
///
/// A third, much shorter burst of `fast_poll_period_ms` for `fast_hold` follows
/// a [`SedHandle::request_fast_polls`] nudge, sent right after the device
/// transmits a message. A reply to that message waits at the parent until the
/// next poll, and the parent's MAC ACK cannot announce it (the reply does not
/// exist yet when the ACK is sent), so without the burst the reply only arrives
/// with a retransmission or the next regular poll. This is the Matter ICD
/// "active mode" after network activity.
#[derive(Clone, Copy, Debug)]
pub struct ThreadSedConfig {
    /// Poll period while active.
    pub active_poll_period_ms: u32,
    /// How long the active window stays open after the last nudge.
    pub active_hold: Duration,
    /// Poll period during a fast-poll burst.
    pub fast_poll_period_ms: u32,
    /// How long a fast-poll burst lasts.
    pub fast_hold: Duration,
    /// Child timeout in seconds, or `None` to leave OpenThread's default.
    pub child_timeout_s: Option<u32>,
}

impl ThreadSedConfig {
    fn active_child_supervision_check_timeout_s(&self) -> u16 {
        child_supervision_check_timeout_s(self.active_poll_period_ms)
    }

    fn fast_child_supervision_check_timeout_s(&self) -> u16 {
        child_supervision_check_timeout_s(self.fast_poll_period_ms)
    }
}

/// OpenThread's default child-supervision interval: the child asks its parent
/// for a supervision message at least this often
/// (`OPENTHREAD_CONFIG_CHILD_SUPERVISION_INTERVAL`). Left unchanged here.
const CHILD_SUPERVISION_INTERVAL_S: u32 = 129;

/// OpenThread's default child-supervision check timeout
/// (`OPENTHREAD_CONFIG_CHILD_SUPERVISION_CHECK_TIMEOUT`).
const DEFAULT_CHILD_SUPERVISION_CHECK_TIMEOUT_S: u32 = 190;

/// The child-supervision check timeout that matches a poll period, never below
/// OpenThread's default.
///
/// A sleepy child only gets the parent's queued supervision message when it
/// polls, so the longest normal gap between frames from the parent is the
/// supervision interval plus one poll period. With a check timeout below that
/// gap (e.g. the 190 s default against a 900 s poll period), every idle period
/// ends in a false "Supervision timeout" and a needless Child Update Request.
/// One more poll period is added as margin for a missed poll.
fn child_supervision_check_timeout_s(poll_period_ms: u32) -> u16 {
    CHILD_SUPERVISION_INTERVAL_S
        .saturating_add(poll_period_ms.div_ceil(1000).saturating_mul(2))
        .clamp(
            DEFAULT_CHILD_SUPERVISION_CHECK_TIMEOUT_S,
            u32::from(u16::MAX),
        ) as u16
}

/// An application-owned handle for the SED duty cycle: it owns the current idle
/// poll period and can (re)open the "active" window.
///
/// It is `const`-constructible, `Sync` and static-friendly - declare it as a
/// `static`, hand a `&` to [`EmbassyThread::with_sed`], then call its methods
/// wherever the application learns about state changes (a button handler, an
/// ICD stay-active request, or a change to the ICD operating mode).
///
/// ```ignore
/// static SED: SedHandle = SedHandle::new(30_000);
/// // ... .with_sed(config, &SED)
/// SED.request_active();          // be responsive for a while
/// SED.request_fast_polls();      // fetch the reply to a message just sent
/// SED.set_idle_period(900_000);  // then sleep longer between polls
/// ```
pub struct SedHandle {
    active: Signal<CriticalSectionRawMutex, ()>,
    fast: Signal<CriticalSectionRawMutex, ()>,
    idle_changed: Signal<CriticalSectionRawMutex, ()>,
    idle_poll_period_ms: AtomicU32,
}

impl SedHandle {
    /// Create a new handle with the given initial idle poll period.
    pub const fn new(idle_poll_period_ms: u32) -> Self {
        Self {
            active: Signal::new(),
            fast: Signal::new(),
            idle_changed: Signal::new(),
            idle_poll_period_ms: AtomicU32::new(idle_poll_period_ms),
        }
    }

    /// (Re)open the active window. Non-blocking and coalescing, so it is safe
    /// to call from anywhere.
    pub fn request_active(&self) {
        self.active.signal(());
    }

    /// Start a short fast-poll burst, right after the device sent a message
    /// that expects a reply. Non-blocking and coalescing, so it is safe to call
    /// from anywhere.
    pub fn request_fast_polls(&self) {
        self.fast.signal(());
    }

    /// Change the idle poll period. Applied immediately if the device is
    /// currently idle, and at the next decay otherwise.
    pub fn set_idle_period(&self, poll_period_ms: u32) {
        self.idle_poll_period_ms
            .store(poll_period_ms, Ordering::Relaxed);
        self.idle_changed.signal(());
    }

    fn idle_poll_period_ms(&self) -> u32 {
        self.idle_poll_period_ms.load(Ordering::Relaxed)
    }

    fn idle_child_supervision_check_timeout_s(&self) -> u16 {
        child_supervision_check_timeout_s(self.idle_poll_period_ms())
    }

    async fn wait_active(&self) {
        self.active.wait().await
    }

    async fn wait_fast(&self) {
        self.fast.wait().await
    }

    async fn wait_idle_changed(&self) {
        self.idle_changed.wait().await
    }
}

/// The per-run SED wiring: the app's handle plus the timings. `Copy`.
#[derive(Clone, Copy)]
struct SedRuntime<'a> {
    handle: &'a SedHandle,
    config: ThreadSedConfig,
}

/// A `Wireless` trait implementation for `openthread`'s Thread stack.
pub struct EmbassyThread<'a, T, K, R> {
    driver: T,
    ieee_eui64: [u8; 8],
    kv: K,
    context: &'a OtNetContext,
    ble_context: &'a BtpGattContext,
    use_ble_random_addr: bool,
    rand: R,
    sed: Option<SedRuntime<'a>>,
}

impl<'a, T, K, R> EmbassyThread<'a, T, K, R>
where
    T: ThreadDriver,
    K: KvBlobStoreAccess,
    R: CryptoRng + Copy,
{
    /// Create a new instance of the `EmbassyThread` type.
    pub fn new<const B: usize, E>(
        driver: T,
        rand: R,
        ieee_eui64: [u8; 8],
        kv: K,
        stack: &'a EmbassyThreadMatterStack<'a, B, E>,
        use_ble_random_addr: bool,
    ) -> Self
    where
        E: Embedding,
    {
        Self::wrap(
            driver,
            rand,
            ieee_eui64,
            kv,
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
        kv: K,
        context: &'a OtNetContext,
        ble_context: &'a BtpGattContext,
        use_ble_random_addr: bool,
    ) -> Self {
        Self {
            driver,
            ieee_eui64,
            kv,
            context,
            ble_context,
            rand,
            use_ble_random_addr,
            sed: None,
        }
    }

    /// Configure this node as a Sleepy End Device (see [`ThreadSedConfig`]).
    ///
    /// `active` is the application's [`SedHandle`], used to reopen the active
    /// window at runtime (e.g. on a button press or an ICD stay-active
    /// request). Applied to OpenThread right after it is created.
    #[must_use]
    pub fn with_sed(mut self, config: ThreadSedConfig, active: &'a SedHandle) -> Self {
        self.sed = Some(SedRuntime {
            handle: active,
            config,
        });
        self
    }
}

impl<T, K, R> wireless::Thread for EmbassyThread<'_, T, K, R>
where
    T: ThreadDriver,
    K: KvBlobStoreAccess,
    R: CryptoRng + Copy,
{
    // The Thread controller this driver produces. The operational task receives
    // `&OtNetCtl` (the driver passes `&net_ctl`), so the chain net-ctl type — and
    // hence this associated type — is `&'a OtNetCtl`. Naming it here lets the
    // commissioning and operational handler chains share one `WirelessNetCtl`
    // type (single monomorphization). `&OtNetCtl` satisfies the bounds via the
    // blanket `impl Trait for &T` impls.
    type NetCtl<'a>
        = &'a OtNetCtl<'a>
    where
        Self: 'a;

    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: wireless::ThreadTask,
    {
        self.driver
            .run(ThreadDriverTaskImpl {
                ieee_eui64: self.ieee_eui64,
                rand: self.rand,
                kv: &self.kv,
                context: self.context,
                task,
                sed: self.sed,
            })
            .await
    }
}

impl<T, K, R> wireless::ThreadCoex for EmbassyThread<'_, T, K, R>
where
    T: ThreadCoexDriver,
    K: KvBlobStoreAccess,
    R: CryptoRng + Copy,
{
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: wireless::ThreadCoexTask,
    {
        self.driver
            .run(ThreadCoexDriverTaskImpl {
                ieee_eui64: self.ieee_eui64,
                rand: self.rand,
                kv: &self.kv,
                context: self.context,
                ble_context: self.ble_context,
                use_ble_random_addr: self.use_ble_random_addr,
                task,
                sed: self.sed,
            })
            .await
    }
}

impl<T, K, R> Gatt for EmbassyThread<'_, T, K, R>
where
    T: BleDriver,
    K: KvBlobStoreAccess,
    R: Rng + Copy,
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
    resources: IfMutex<OtMatterResources>,
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

struct ThreadDriverTaskImpl<'a, A, K, C> {
    ieee_eui64: [u8; 8],
    rand: C,
    kv: K,
    context: &'a OtNetContext,
    task: A,
    sed: Option<SedRuntime<'a>>,
}

impl<A, K, C> ThreadDriverTask for ThreadDriverTaskImpl<'_, A, K, C>
where
    A: wireless::ThreadTask,
    K: KvBlobStoreAccess,
    C: CryptoRng + Copy,
{
    async fn run<R>(&mut self, radio: R) -> Result<(), Error>
    where
        R: Radio,
    {
        let mut resources = self.context.resources.lock().await;
        let resources = &mut *resources;

        let persister = OtPersist::new(&mut resources.settings_buf, &self.kv);
        persister.load()?;

        let mut settings = persister.settings();
        let mut rand = self.rand;

        let ot = OpenThread::new_with_udp_srp(
            self.ieee_eui64,
            &mut rand,
            &mut settings,
            &mut resources.ot,
            &mut resources.udp,
            &mut resources.srp,
        )
        .map_err(to_matter_err)?;

        let net_ctl = OtNetCtl::new(ot.clone());
        let net_stack = match self.sed {
            Some(sed) => OtNetStack::new(ot.clone()).with_sed(sed.handle),
            None => OtNetStack::new(ot.clone()),
        };
        let netif = OtNetif::new(ot.clone());
        let mut mdns = OtMdns::new(ot.clone(), &mut resources.mdns_buf);

        let mut main = pin!(self.task.run(&net_stack, &netif, &net_ctl, &mut mdns));
        let mut radio = pin!(async {
            ot.run(radio).await;
            #[allow(unreachable_code)]
            Ok(())
        });
        let mut persist = pin!(persister.run());
        if let Some(sed) = self.sed {
            ot.set_link_mode(false, false, false)
                .map_err(to_matter_err)?;
            if let Some(child_timeout_s) = sed.config.child_timeout_s {
                ot.set_child_timeout(child_timeout_s);
            }
        }

        let mut sed = pin!(run_sed_and_diag(&ot, self.sed));

        ot.enable_ipv6(true).map_err(to_matter_err)?;
        ot.srp_autostart().map_err(to_matter_err)?;

        let result = select4(&mut main, &mut radio, &mut persist, &mut sed)
            .coalesce()
            .await;

        let _ = ot.enable_thread(false);
        let _ = ot.srp_stop();
        let _ = ot.enable_ipv6(false);

        result
    }
}

struct ThreadCoexDriverTaskImpl<'a, A, K, C> {
    ieee_eui64: [u8; 8],
    rand: C,
    kv: K,
    context: &'a OtNetContext,
    ble_context: &'a BtpGattContext,
    task: A,
    use_ble_random_addr: bool,
    sed: Option<SedRuntime<'a>>,
}

impl<A, K, C> ThreadCoexDriverTask for ThreadCoexDriverTaskImpl<'_, A, K, C>
where
    A: wireless::ThreadCoexTask,
    K: KvBlobStoreAccess,
    C: CryptoRng + Copy,
{
    async fn run<R, B>(&mut self, radio: R, ble_ctl: B) -> Result<(), Error>
    where
        R: Radio,
        B: Controller,
    {
        let mut resources = self.context.resources.lock().await;
        let resources = &mut *resources;

        let persister = OtPersist::new(&mut resources.settings_buf, &self.kv);
        persister.load()?;

        let mut settings = persister.settings();
        let mut rand = self.rand;

        let ot = OpenThread::new_with_udp_srp(
            self.ieee_eui64,
            &mut rand,
            &mut settings,
            &mut resources.ot,
            &mut resources.udp,
            &mut resources.srp,
        )
        .map_err(to_matter_err)?;

        let net_ctl = OtNetCtl::new(ot.clone());
        let net_stack = match self.sed {
            Some(sed) => OtNetStack::new(ot.clone()).with_sed(sed.handle),
            None => OtNetStack::new(ot.clone()),
        };
        let netif = OtNetif::new(ot.clone());
        let mut mdns = OtMdns::new(ot.clone(), &mut resources.mdns_buf);
        let mut peripheral = BtpGattPeripheral::new(
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
        if let Some(sed) = self.sed {
            ot.set_link_mode(false, false, false)
                .map_err(to_matter_err)?;
            if let Some(child_timeout_s) = sed.config.child_timeout_s {
                ot.set_child_timeout(child_timeout_s);
            }
        }

        let mut sed = pin!(run_sed_and_diag(&ot, self.sed));

        ot.enable_ipv6(true).map_err(to_matter_err)?;
        ot.srp_autostart().map_err(to_matter_err)?;

        let result = select4(&mut main, &mut radio, &mut persist, &mut sed)
            .coalesce()
            .await;

        let _ = ot.enable_thread(false);
        let _ = ot.srp_stop();
        let _ = ot.enable_ipv6(false);

        result
    }
}

/// Drives the SED duty cycle: a short `active_poll_period_ms` window that
/// reopens on boot and on every [`SedHandle::request_active`] nudge, decaying
/// to `idle_poll_period_ms` after `active_hold` of quiet.
///
/// Runs as a branch of the driver task's `select4`, so every OpenThread call
/// stays on one task.
async fn run_sed(ot: &OpenThread<'_>, sed: Option<SedRuntime<'_>>) -> Result<(), Error> {
    let SedRuntime { handle, config } = match sed {
        Some(sed) => sed,
        None => {
            // No SED configured: this branch never completes.
            core::future::pending::<()>().await;
            unreachable!()
        }
    };

    let active_ms = config.active_poll_period_ms;
    let hold = config.active_hold;

    // Boot (and commissioning) starts responsive.
    let _ = ot.set_poll_period(active_ms);
    ot.set_child_supervision_check_timeout(config.active_child_supervision_check_timeout_s());

    loop {
        // Keep the active window open while nudges keep arriving.
        let mut deadline = Instant::now() + hold;
        loop {
            match select4(
                handle.wait_active(),
                handle.wait_fast(),
                handle.wait_idle_changed(),
                Timer::at(deadline),
            )
            .await
            {
                Either4::First(()) => deadline = Instant::now() + hold,
                Either4::Second(()) => {
                    run_fast_poll_burst(ot, handle, &config).await;
                    let _ = ot.set_poll_period(active_ms);
                    ot.set_child_supervision_check_timeout(
                        config.active_child_supervision_check_timeout_s(),
                    );
                }
                // Idle period changed mid-window; picked up at the next decay.
                Either4::Third(()) => {}
                Either4::Fourth(()) => break,
            }
        }

        // Quiet: decay to the current idle period.
        let _ = ot.set_poll_period(handle.idle_poll_period_ms());
        ot.set_child_supervision_check_timeout(handle.idle_child_supervision_check_timeout_s());

        // Stay idle until nudged, re-applying whenever the idle period changes.
        loop {
            match select3(
                handle.wait_active(),
                handle.wait_fast(),
                handle.wait_idle_changed(),
            )
            .await
            {
                Either3::First(()) => break,
                Either3::Second(()) => {
                    run_fast_poll_burst(ot, handle, &config).await;
                    let _ = ot.set_poll_period(handle.idle_poll_period_ms());
                    ot.set_child_supervision_check_timeout(
                        handle.idle_child_supervision_check_timeout_s(),
                    );
                }
                Either3::Third(()) => {
                    let _ = ot.set_poll_period(handle.idle_poll_period_ms());
                    ot.set_child_supervision_check_timeout(
                        handle.idle_child_supervision_check_timeout_s(),
                    );
                }
            }
        }

        let _ = ot.set_poll_period(active_ms);
        ot.set_child_supervision_check_timeout(config.active_child_supervision_check_timeout_s());
    }
}

/// Polls at the fast period until `fast_hold` passes without another
/// [`SedHandle::request_fast_polls`]. The caller restores its own poll period
/// afterwards. OpenThread sends the first poll at once when the period gets
/// shorter, so the burst starts without delay.
///
/// Other nudges that arrive during the burst are latched by their signals and
/// handled after it.
async fn run_fast_poll_burst(ot: &OpenThread<'_>, handle: &SedHandle, config: &ThreadSedConfig) {
    let _ = ot.set_poll_period(config.fast_poll_period_ms);
    ot.set_child_supervision_check_timeout(config.fast_child_supervision_check_timeout_s());

    let mut deadline = Instant::now() + config.fast_hold;
    while let Either::First(()) = select(handle.wait_fast(), Timer::at(deadline)).await {
        deadline = Instant::now() + config.fast_hold;
    }
}

/// Runs the SED duty-cycle driver and the Thread-role diagnostic concurrently.
/// Neither future ever completes, so this branch just keeps both alive.
async fn run_sed_and_diag(ot: &OpenThread<'_>, sed: Option<SedRuntime<'_>>) -> Result<(), Error> {
    let mut sed = pin!(run_sed(ot, sed));
    let mut diag = pin!(run_thread_diag(ot));

    let _ = select(&mut sed, &mut diag).await;

    core::future::pending().await
}

/// Logs the Thread device role whenever it changes, so a log capture shows
/// whether the node is attached, detached, or re-attaching.
async fn run_thread_diag(ot: &OpenThread<'_>) -> Result<(), Error> {
    const POLL: Duration = Duration::from_secs(5);

    let mut last: Option<DeviceRole> = None;

    loop {
        Timer::after(POLL).await;

        let role = ot.device_role();
        if last != Some(role) {
            last = Some(role);
            info!("Thread device role: {role:?}");
        }
    }
}

#[cfg(test)]
mod test {
    use super::child_supervision_check_timeout_s;

    struct TestCase {
        name: &'static str,
        poll_period_ms: u32,
        expected: u16,
    }

    #[test]
    fn test_child_supervision_check_timeout_s() {
        let test_cases = [
            TestCase {
                name: "fast-poll burst keeps the OpenThread default",
                poll_period_ms: 200,
                expected: 190,
            },
            TestCase {
                name: "short poll keeps the OpenThread default",
                poll_period_ms: 15_000,
                expected: 190,
            },
            TestCase {
                name: "long poll adds the interval and two poll periods",
                poll_period_ms: 900_000,
                expected: 1_929,
            },
            TestCase {
                name: "partial seconds round up",
                poll_period_ms: 100_500,
                expected: 331,
            },
            TestCase {
                name: "very long poll saturates at the u16 maximum",
                poll_period_ms: 40_000_000,
                expected: u16::MAX,
            },
        ];

        for TestCase {
            name,
            poll_period_ms,
            expected,
        } in test_cases
        {
            let result = child_supervision_check_timeout_s(poll_period_ms);
            assert_eq!(result, expected, "Failed case: '{name}'");
        }
    }
}

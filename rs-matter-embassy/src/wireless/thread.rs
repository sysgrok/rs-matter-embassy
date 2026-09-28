use core::pin::pin;

use embassy_futures::select::select4;

use openthread::{Capabilities, OpenThread, Radio};

use rs_matter_stack::matter::dm::clusters::icd_mgmt::{Icd, IcdNetParams};
use rs_matter_stack::matter::persist::KvBlobStoreAccess;

use crate::ble::{BtpGattContext, Controller};
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

use super::{
    BleDriver, BleDriverGattPeripheral, BleDriverTask, BleDriverTaskImpl,
    EmbassyWirelessMatterStack, PreexistingBleDriver,
};

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
/// as well as to a BLE driver to perform its work.
///
/// The task gets a `BleDriver` rather than a BLE controller, so that the BLE controller can be
/// created only while BLE is needed (i.e. while a commissioning window is advertised) and torn
/// down afterwards - which matters for battery-powered devices. A driver that has a pre-existing
/// controller can hand out `PreexistingBleDriver`.
pub trait ThreadCoexDriverTask {
    /// Run the task with the given Thread radio and BLE driver
    async fn run<R, B>(&mut self, radio: R, ble: B) -> Result<(), Error>
    where
        R: Radio,
        B: BleDriver;
}

impl<T> ThreadCoexDriverTask for &mut T
where
    T: ThreadCoexDriverTask,
{
    async fn run<R, B>(&mut self, radio: R, ble: B) -> Result<(), Error>
    where
        R: Radio,
        B: BleDriver,
    {
        (*self).run(radio, ble).await
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
        task.run(&mut self.0, PreexistingBleDriver::new(&self.1))
            .await
    }
}

impl<R, B> BleDriver for PreexistingThreadDriver<R, B>
where
    B: Controller,
{
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: BleDriverTask,
    {
        PreexistingBleDriver::new(&self.1).run(task).await
    }
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
    icd: Option<&'a Icd>,
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
            icd: None,
        }
    }

    /// Make the Thread node a Sleepy End Device following the ICD power mode of the given ICD
    /// Management cluster state - the same `Icd` instance that backs the application's
    /// `IcdMgmtHandler` on the root endpoint.
    ///
    /// The node then keeps its receiver off between data polls and polls its parent at the ICD
    /// polling interval: the fast (`SAI`) one while the ICD is in active mode, the slow (`SII`)
    /// one while idle. See the `icd_mgmt` module of `rs-matter` for the state machine.
    #[must_use]
    pub const fn with_icd(mut self, icd: &'a Icd) -> Self {
        self.icd = Some(icd);
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
                icd: self.icd,
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
                icd: self.icd,
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
    icd: Option<&'a Icd>,
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
        let net_stack = OtNetStack::new(ot.clone());
        let netif = OtNetif::new(ot.clone());
        let mut mdns = OtMdns::new(ot.clone(), &mut resources.mdns_buf);

        if let Some(icd) = self.icd {
            configure_sed(&ot, icd)?;
        }

        let mut main = pin!(self.task.run(&net_stack, &netif, &net_ctl, &mut mdns));
        let mut radio = pin!(async {
            ot.run(radio).await;
            #[allow(unreachable_code)]
            Ok(())
        });
        let mut persist = pin!(persister.run());
        let mut icd = pin!(run_icd(&ot, self.icd));
        ot.enable_ipv6(true).map_err(to_matter_err)?;
        ot.srp_autostart().map_err(to_matter_err)?;

        let result = select4(&mut main, &mut radio, &mut persist, &mut icd)
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
    icd: Option<&'a Icd>,
}

impl<A, K, C> ThreadCoexDriverTask for ThreadCoexDriverTaskImpl<'_, A, K, C>
where
    A: wireless::ThreadCoexTask,
    K: KvBlobStoreAccess,
    C: CryptoRng + Copy,
{
    async fn run<R, B>(&mut self, radio: R, ble: B) -> Result<(), Error>
    where
        R: Radio,
        B: BleDriver,
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
        let net_stack = OtNetStack::new(ot.clone());
        let netif = OtNetif::new(ot.clone());
        let mut mdns = OtMdns::new(ot.clone(), &mut resources.mdns_buf);
        // The BLE controller is created only while the stack runs the peripheral, i.e. while a
        // commissioning window is advertised over BLE.
        let mut peripheral = BleDriverGattPeripheral::new(
            ble,
            self.use_ble_random_addr.then_some(self.rand),
            self.ble_context,
        );

        if let Some(icd) = self.icd {
            configure_sed(&ot, icd)?;
        }

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
        let mut icd = pin!(run_icd(&ot, self.icd));
        ot.enable_ipv6(true).map_err(to_matter_err)?;
        ot.srp_autostart().map_err(to_matter_err)?;

        let result = select4(&mut main, &mut radio, &mut persist, &mut icd)
            .coalesce()
            .await;

        let _ = ot.enable_thread(false);
        let _ = ot.srp_stop();
        let _ = ot.enable_ipv6(false);

        result
    }
}

/// OpenThread's default child timeout, in seconds (`OPENTHREAD_CONFIG_MLE_CHILD_TIMEOUT_DEFAULT`).
const CHILD_TIMEOUT_DEFAULT_S: u32 = 240;

/// The margin added on top of the slowest polling interval when sizing the child timeout: two
/// missed polls plus some slack.
const CHILD_TIMEOUT_MARGIN_S: u32 = 30;

/// OpenThread's default child-supervision interval, in seconds
/// (`OPENTHREAD_CONFIG_CHILD_SUPERVISION_INTERVAL`): the child asks its parent for a supervision
/// message at least this often. Left at its default here.
const CHILD_SUPERVISION_INTERVAL_S: u32 = 129;

/// OpenThread's default child-supervision check timeout, in seconds
/// (`OPENTHREAD_CONFIG_CHILD_SUPERVISION_CHECK_TIMEOUT`).
const CHILD_SUPERVISION_CHECK_TIMEOUT_DEFAULT_S: u32 = 190;

/// Configure the node as a Sleepy End Device (receiver off when idle, MTD, stable network data
/// only) and size its keep-alives from the slowest polling interval the ICD will ever use.
///
/// Starts out data-polling; `run_icd` switches to CSL once the radio is up, if it can.
fn configure_sed(ot: &OpenThread<'_>, icd: &Icd) -> Result<(), Error> {
    let params = icd.net_params();

    ot.set_link_mode(false, false, false)
        .map_err(to_matter_err)?;

    ot.set_child_timeout(child_timeout_s(params.max_poll_interval_ms));

    apply_icd_params(ot, &params, false)
}

/// Apply the current ICD polling interval to OpenThread: as the CSL period if the radio can
/// do CSL (`csl`), else as the data-poll period.
///
/// A CSL child listens at its parent's transmit times instead of polling, so the "polling
/// interval" becomes the period between its receive windows - same reachability, no poll
/// transmission per listen. OpenThread caps the CSL period at ~10.5 s; a longer idle interval
/// (a LIT's) is listened at that cap, which is still cheaper than one poll per interval.
fn apply_icd_params(ot: &OpenThread<'_>, params: &IcdNetParams, csl: bool) -> Result<(), Error> {
    info!(
        "Thread SED: {:?} / {:?}, {} every {} ms",
        params.power_mode,
        params.operating_mode,
        if csl { "listening (CSL)" } else { "polling" },
        params.poll_interval_ms
    );

    if csl {
        // Whole 10-symbol units, within OpenThread's range.
        const CSL_UNIT_US: u32 = 160;

        let period_us = (params.poll_interval_ms as u64 * 1000)
            .min(OpenThread::CSL_MAX_PERIOD_US as u64) as u32
            / CSL_UNIT_US
            * CSL_UNIT_US;

        ot.set_csl_period(period_us.max(CSL_UNIT_US))
            .map_err(to_matter_err)?;

        // With CSL on, OpenThread's automatic poll period is the CSL keep-alive.
        ot.set_poll_period(0).map_err(to_matter_err)?;
    } else {
        ot.set_poll_period(params.poll_interval_ms)
            .map_err(to_matter_err)?;
    }

    ot.set_child_supervision_check_timeout(child_supervision_check_timeout_s(
        params.poll_interval_ms,
    ));

    Ok(())
}

/// Follow the ICD power mode: re-apply the polling interval whenever it changes.
///
/// Never completes; a no-op (that never completes either) without an ICD.
async fn run_icd(ot: &OpenThread<'_>, icd: Option<&Icd>) -> Result<(), Error> {
    let Some(icd) = icd else {
        core::future::pending::<()>().await;
        unreachable!()
    };

    // Whether the radio can do CSL is only known once it is up.
    ot.wait_radio_ready().await;

    let csl = ot
        .radio_caps()
        .is_some_and(|caps| caps.contains(Capabilities::RECEIVE_TIMING));

    if csl {
        info!("Thread SED: the radio supports timed receive, using CSL instead of data polls");
    }

    loop {
        apply_icd_params(ot, &icd.net_params(), csl)?;

        icd.wait_net_changed().await;
    }
}

/// The child timeout that fits a polling interval: the parent must not evict the child between
/// two polls, so the timeout exceeds the interval by a margin - but is never shorter than
/// OpenThread's default.
fn child_timeout_s(poll_period_ms: u32) -> u32 {
    poll_period_ms
        .div_ceil(1000)
        .saturating_add(CHILD_TIMEOUT_MARGIN_S)
        .max(CHILD_TIMEOUT_DEFAULT_S)
}

/// The child-supervision check timeout that matches a polling interval, never below
/// OpenThread's default.
///
/// A sleepy child only gets the parent's queued supervision message when it polls, so the
/// longest normal gap between frames from the parent is the supervision interval plus one
/// polling interval. With a check timeout below that gap (e.g. the 190 s default against a
/// 900 s interval), every idle period ends in a false "supervision timeout" and a needless
/// Child Update Request. One more polling interval is added as margin for a missed poll.
fn child_supervision_check_timeout_s(poll_period_ms: u32) -> u16 {
    CHILD_SUPERVISION_INTERVAL_S
        .saturating_add(poll_period_ms.div_ceil(1000).saturating_mul(2))
        .clamp(
            CHILD_SUPERVISION_CHECK_TIMEOUT_DEFAULT_S,
            u32::from(u16::MAX),
        ) as u16
}

#[cfg(test)]
mod test {
    use super::{child_supervision_check_timeout_s, child_timeout_s};

    #[test]
    fn supervision_check_timeout_follows_the_poll_period() {
        // Short polls keep OpenThread's default.
        assert_eq!(child_supervision_check_timeout_s(200), 190);
        assert_eq!(child_supervision_check_timeout_s(15_000), 190);
        // Long polls add the interval and two poll periods.
        assert_eq!(child_supervision_check_timeout_s(900_000), 1_929);
        // Partial seconds round up.
        assert_eq!(child_supervision_check_timeout_s(100_500), 331);
        // Very long polls saturate.
        assert_eq!(child_supervision_check_timeout_s(40_000_000), u16::MAX);
    }

    #[test]
    fn child_timeout_exceeds_the_poll_period() {
        assert_eq!(child_timeout_s(15_000), 240);
        assert_eq!(child_timeout_s(600_000), 630);
    }
}

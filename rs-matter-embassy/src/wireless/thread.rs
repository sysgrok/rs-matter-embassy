use core::marker::PhantomData;
use core::pin::pin;

use openthread::{Capabilities, OpenThread, Radio};

use rs_matter::dm::clusters::icd_mgmt::{Icd, IcdNetParams};
use rs_matter::persist::KvBlobStoreAccess;

use crate::ble::{BtpGattContext, Controller};
use crate::matter::crypto::{CryptoRng, Rng};
use crate::matter::dm::networks::wireless::ThreadNetworks;
use crate::matter::error::Error;
use crate::matter::utils::futures::{select4, Coalesce};
use crate::matter::utils::init::{init, Init};
use crate::matter::utils::sync::IfMutex;
use crate::ot::{radio_to_matter_err, to_matter_err, OtNetCtl, OtNetStack, OtPersist};
use crate::ot::{OtMatterResources, OtMdns, OtNetif};
use crate::stack::ble::GattPeripheral;
use crate::stack::network::{Embedding, StackNetwork, ThreadStackNetwork};
use crate::stack::wireless::{
    GattDriver, GattTask, ThreadCoexDriver, ThreadCoexTask, ThreadDriver, ThreadTask,
};
use crate::stack::MatterStack;

use super::{ControllerGattPeripheral, EmbassyGatt};

#[cfg(feature = "esp")]
pub mod esp_thread;
#[cfg(feature = "nrf")]
pub mod nrf;

/// The network of a `MatterStack` that runs Thread with [`EmbassyThread`]: the networks store
/// `S`, and the contexts of `openthread` and of the BLE controller, which the stack holds
/// so that they can be statically allocated.
pub struct EmbassyThreadNetwork<S = ThreadNetworks>(PhantomData<fn() -> S>);

impl<S> StackNetwork for EmbassyThreadNetwork<S>
where
    S: ThreadStackNetwork<Context = ()>,
{
    type Store = S::Store;
    type Context = EmbassyGatt<OtNetContext>;
}

impl<S> ThreadStackNetwork for EmbassyThreadNetwork<S> where S: ThreadStackNetwork<Context = ()> {}

/// The radio of a pre-existing, already created Thread radio and BLE controller,
/// rather than one which creates them when the Matter stack needs them.
pub struct PreexistingThreadRadio<R, B>(R, B);

impl<R, B> PreexistingThreadRadio<R, B> {
    /// Create a new instance of the `PreexistingThreadRadio` type.
    pub const fn new(radio: R, ble: B) -> Self {
        Self(radio, ble)
    }
}

impl<R, B, K, C> ThreadDriver for EmbassyThread<'_, PreexistingThreadRadio<R, B>, K, C>
where
    R: Radio,
    K: KvBlobStoreAccess,
    C: CryptoRng + Copy,
{
    type NetCtl<'a>
        = &'a OtNetCtl<'a>
    where
        Self: 'a;

    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: ThreadTask,
    {
        self.runner.run(&mut self.radio.0, task).await
    }
}

impl<R, B, K, C> ThreadCoexDriver for EmbassyThread<'_, PreexistingThreadRadio<R, B>, K, C>
where
    R: Radio,
    B: Controller,
    K: KvBlobStoreAccess,
    C: CryptoRng + Copy,
{
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: ThreadCoexTask,
    {
        let gatt = ControllerGattPeripheral::new(
            &self.radio.1,
            self.runner.ble_rand(),
            self.runner.ble_context(),
        );

        self.runner.run_coex(&mut self.radio.0, gatt, task).await
    }
}

impl<R, B, K, C> GattDriver for EmbassyThread<'_, PreexistingThreadRadio<R, B>, K, C>
where
    B: Controller,
    C: Rng + Copy,
{
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: GattTask,
    {
        task.run(ControllerGattPeripheral::new(
            &self.radio.1,
            self.runner.ble_rand(),
            self.runner.ble_context(),
        ))
        .await
    }
}

/// The Thread drivers of the stack over `openthread`, one for each of the radios `T`.
pub struct EmbassyThread<'a, T, K, R> {
    radio: T,
    runner: OtThreadRunner<'a, K, R>,
}

impl<'a, T, K, R> EmbassyThread<'a, T, K, R> {
    /// Create a new instance of the `EmbassyThread` type.
    pub fn new<const B: usize, S, E>(
        radio: T,
        rand: R,
        ieee_eui64: [u8; 8],
        kv: K,
        stack: &'a MatterStack<'a, B, S, E>,
        use_ble_random_addr: bool,
    ) -> Self
    where
        S: StackNetwork<Context = EmbassyGatt<OtNetContext>>,
        E: Embedding,
    {
        Self::wrap(
            radio,
            rand,
            ieee_eui64,
            kv,
            stack.context().net_context(),
            stack.context().ble_context(),
            use_ble_random_addr,
        )
    }

    /// Wrap a Thread radio with the given parameters.
    pub fn wrap(
        radio: T,
        rand: R,
        ieee_eui64: [u8; 8],
        kv: K,
        context: &'a OtNetContext,
        ble_context: &'a BtpGattContext,
        use_ble_random_addr: bool,
    ) -> Self {
        Self {
            radio,
            runner: OtThreadRunner::new(
                rand,
                ieee_eui64,
                kv,
                context,
                ble_context,
                use_ble_random_addr,
            ),
        }
    }

    /// Make the Thread node a Sleepy End Device following the given ICD power mode state
    /// machine - the same `Icd` instance that backs the application's ICD Management handler on
    /// the root endpoint: the `SitIcdMgmtHandler` of a SIT-only device, or the `LitIcdMgmtHandler`
    /// of a LIT-capable one (pass its `LitIcd::icd`).
    ///
    /// The node then keeps its receiver off between data polls and polls its parent at the ICD
    /// polling interval: the fast (`SAI`) one while the ICD is in active mode, the slow (`SII`)
    /// one while idle. See the `icd_mgmt` module of `rs-matter` for the state machine.
    #[must_use]
    pub const fn with_icd(mut self, icd: &'a Icd) -> Self {
        self.runner.icd = Some(icd);
        self
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

/// What the Thread drivers have in common, whatever their radio: running a task of
/// the stack over `openthread`.
///
/// For implementing the Thread drivers of the stack for a radio this crate does not know.
pub struct OtThreadRunner<'a, K, R> {
    ieee_eui64: [u8; 8],
    rand: R,
    kv: K,
    context: &'a OtNetContext,
    ble_context: &'a BtpGattContext,
    use_ble_random_addr: bool,
    icd: Option<&'a Icd>,
}

impl<'a, K, R> OtThreadRunner<'a, K, R> {
    /// Create a new instance of the `OtThreadRunner` type.
    pub const fn new(
        rand: R,
        ieee_eui64: [u8; 8],
        kv: K,
        context: &'a OtNetContext,
        ble_context: &'a BtpGattContext,
        use_ble_random_addr: bool,
    ) -> Self {
        Self {
            ieee_eui64,
            rand,
            kv,
            context,
            ble_context,
            use_ble_random_addr,
            icd: None,
        }
    }

    /// Return the GATT peripheral context.
    pub const fn ble_context(&self) -> &'a BtpGattContext {
        self.ble_context
    }
}

impl<K, R> OtThreadRunner<'_, K, R>
where
    R: Rng + Copy,
{
    /// Return the random number generator for the BLE address, if a random one should be used.
    pub fn ble_rand(&self) -> Option<R> {
        self.use_ble_random_addr.then_some(self.rand)
    }

    /// Return the random number generator.
    pub fn rand(&self) -> R {
        self.rand
    }
}

impl<K, C> OtThreadRunner<'_, K, C>
where
    K: KvBlobStoreAccess,
    C: CryptoRng + Copy,
{
    /// Run a Thread task over the given radio.
    pub async fn run<R, A>(&self, mut radio: R, mut task: A) -> Result<(), Error>
    where
        R: Radio,
        A: ThreadTask,
    {
        let mut resources = self.context.resources.lock().await;
        let resources = &mut *resources;

        let persister = OtPersist::new(&mut resources.settings_buf, &self.kv);
        persister.load()?;

        let mut settings = persister.settings();
        let mut rand = self.rand;

        let radio_caps = radio.init().await.map_err(radio_to_matter_err)?;

        let ot = OpenThread::new_with_udp_srp(
            self.ieee_eui64,
            &radio_caps,
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

        let mut main = pin!(task.run(&net_stack, &netif, &net_ctl, &mut mdns));
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

    /// Run a Thread task with concurrent commissioning over the given radio and GATT peripheral.
    pub async fn run_coex<R, G, A>(
        &self,
        mut radio: R,
        mut gatt: G,
        mut task: A,
    ) -> Result<(), Error>
    where
        R: Radio,
        G: GattPeripheral,
        A: ThreadCoexTask,
    {
        let mut resources = self.context.resources.lock().await;
        let resources = &mut *resources;

        let persister = OtPersist::new(&mut resources.settings_buf, &self.kv);
        persister.load()?;

        let mut settings = persister.settings();
        let mut rand = self.rand;

        let radio_caps = radio.init().await.map_err(radio_to_matter_err)?;

        let ot = OpenThread::new_with_udp_srp(
            self.ieee_eui64,
            &radio_caps,
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

        let mut main = pin!(task.run(&net_stack, &netif, &net_ctl, &mut mdns, &mut gatt));
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
/// only), with its keep-alives sized as `apply_icd_params` does.
///
/// Starts out data-polling; `run_icd` switches to CSL once the radio is up, if it can.
fn configure_sed(ot: &OpenThread<'_>, icd: &Icd) -> Result<(), Error> {
    let params = icd.net_params();

    ot.set_link_mode(false, false, false)
        .map_err(to_matter_err)?;

    apply_icd_params(ot, &params, false)
}

/// How often a CSL child polls its parent, only to stay synchronized with it.
///
/// A poll costs a short transmission and an ACK; going without one costs a sample window that
/// grows with the drift of both clocks - by tens of microseconds per second without a sync, in
/// every window.
const CSL_SYNC_POLL_PERIOD_MS: u32 = 10_000;

/// Apply the current ICD polling interval to OpenThread: as the CSL period if the radio can
/// do CSL (`csl`), else as the data-poll period.
///
/// A CSL child listens at its parent's transmit times instead of polling, so the "polling
/// interval" becomes the period between its receive windows - same reachability, no poll
/// transmission per listen. OpenThread caps the CSL period at ~10.5 s; a longer idle interval
/// (a LIT's) is listened at that cap, which is still cheaper than one poll per interval.
fn apply_icd_params(ot: &OpenThread<'_>, params: &IcdNetParams, csl: bool) -> Result<(), Error> {
    let child_timeout_s = child_timeout_s(params.max_poll_interval_ms);

    info!(
        "Thread SED: {:?} / {:?}, {} every {} ms, child timeout {} s",
        params.power_mode,
        params.operating_mode,
        if csl { "listening (CSL)" } else { "polling" },
        params.poll_interval_ms,
        child_timeout_s
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

        // With CSL on, the parent reaches the child in its sample windows, so receiving needs
        // no data polls. Polling every now and then still pays off: each poll carries the CSL
        // IE and is answered with an enhanced ACK, which re-synchronizes the child with the
        // parent - and OpenThread widens every sample window by the clock drift accumulated
        // since the last synchronization, so that without it an idle child keeps the receiver
        // on for longer and longer.
        ot.set_poll_period(CSL_SYNC_POLL_PERIOD_MS)
            .map_err(to_matter_err)?;
    } else {
        ot.set_poll_period(params.poll_interval_ms)
            .map_err(to_matter_err)?;
    }

    ot.set_child_supervision_check_timeout(child_supervision_check_timeout_s(
        params.poll_interval_ms,
    ));

    // From the slowest polling interval the ICD will ever use, which it may only set once it
    // runs (e.g. from its `SII`), after Thread came up: an attached child tells its parent.
    ot.set_child_timeout(child_timeout_s);

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

    let csl = ot.radio_caps().contains(Capabilities::RECEIVE_TIMING);

    if csl {
        info!("Thread SED: the radio supports timed receive, using CSL instead of data polls");
    }

    // The state machine signals every change of its own state, including the active deadline
    // moving out on each message received, which leaves the net params as they were. Only a
    // change of those is worth (re-)programming OpenThread for - and logging.
    let mut applied: Option<IcdNetParams> = None;

    loop {
        let params = icd.net_params();

        if applied.as_ref() != Some(&params) {
            apply_icd_params(ot, &params, csl)?;
            applied = Some(params);
        }

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

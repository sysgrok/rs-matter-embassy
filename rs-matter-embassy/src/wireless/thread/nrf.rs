//! Thread (and BLE) drivers for Nordic silicon.
//!
//! Two drivers live here:
//! - [`NrfThreadMpslRadioDriver`] drives the IEEE 802.15.4 radio through the Nordic
//!   C driver (the `nrf-802154` crate) on top of MPSL, and is therefore the only
//!   one capable of running Thread and BLE at the same time. It works on every
//!   supported part.
//! - `NrfThreadRustRadioDriver` drives the bare PHY of `embassy-nrf` with a
//!   software MAC on top. It is nRF52-only - the nRF54L series has no `embassy-nrf`
//!   radio driver - and it cannot coexist with BLE.
//!
//! # nRF54L
//!
//! MPSL asserts unless the chip runs at 128 MHz, which is also the only frequency
//! the 802.15.4 driver supports there, so the application must set
//! `config.clock_speed = ClockSpeed::CK128` before calling `embassy_nrf::init`.

use core::pin::pin;

use embassy_futures::select::{select, Either};

use embassy_nrf::interrupt;
use embassy_nrf::interrupt::typelevel::{Binding, Interrupt};
use embassy_nrf::peripherals::RADIO;
use embassy_nrf::Peri;

use nrf_mpsl::{
    ClockInterruptHandler as NrfBleClockInterruptHandler,
    HighPrioInterruptHandler as NrfBleHighPrioInterruptHandler,
    LowPrioInterruptHandler as NrfBleLowPrioInterruptHandler, MultiprotocolServiceLayer,
};

use nrf_sdc::SoftdeviceController;

use rs_matter_stack::matter::crypto::{CryptoRng, CryptoRngCore, RngCore};
use rs_matter_stack::matter::error::{Error, ErrorCode};
use rs_matter_stack::rand::RngAdaptor;

#[cfg(feature = "_nrf52")]
use embassy_nrf::interrupt::typelevel::Handler;
#[cfg(feature = "_nrf52")]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(feature = "_nrf52")]
use openthread::nrf::{Ieee802154Peripheral, NrfRadio};
#[cfg(feature = "_nrf52")]
use openthread::{
    EmbassyTimeTimer, MacRadio, MacRadioResources, PhyRadioRunner, ProxyRadio, ProxyRadioResources,
};
#[cfg(feature = "_nrf52")]
use portable_atomic::{AtomicBool, Ordering};
#[cfg(feature = "_nrf52")]
use rs_matter_stack::matter::utils::sync::Signal;

#[cfg(feature = "_nrf54l")]
pub use nrf_802154::CcmInterruptHandler;
pub use nrf_802154::{EguInterruptHandler, LpTimerInterruptHandler};

/// The peripherals the IEEE 802.15.4 driver takes ownership of. Chip-specific -
/// see the `nrf-802154` crate for what goes in.
pub use nrf_802154::RadioPeripherals as NrfIeee802154Peripherals;
/// The peripherals MPSL takes ownership of. Chip-specific - see the `nrf-mpsl`
/// crate for what goes in.
pub use nrf_mpsl::Peripherals as NrfMpslPeripherals;
/// The peripherals the SoftDevice Controller takes ownership of. Chip-specific -
/// see the `nrf-sdc` crate for what goes in.
pub use nrf_sdc::Peripherals as NrfSdcPeripherals;

#[cfg(feature = "_nrf52")]
pub use openthread::ProxyRadioResources as NrfThreadRadioResources;

/// The L2CAP buffer size the SoftDevice Controller is configured with, i.e. the largest ACL
/// payload a link can carry. A property of the controller, so it does not vary with the BLE host
/// backend compiled on top of it (and it matches the packet-pool MTU the `trouble` backend uses).
const L2CAP_MTU: usize = 251;

/// How many outgoing L2CAP buffers per link
const L2CAP_TXQ: u8 = 3;
/// How many incoming L2CAP buffers per link
const L2CAP_RXQ: u8 = 3;

#[cfg(feature = "_nrf52")]
static RUST_RADIO_STATE: RustRadioState = RustRadioState::new();

/// The interrupt lines MPSL and the IEEE 802.15.4 driver claim.
///
/// Which ones those are is chip-specific: the nRF54L series names its radio,
/// timer and low-power counter differently, runs MPSL's low-priority work off a
/// plain software interrupt rather than off an EGU shared with the 802.15.4
/// driver, and offloads frame encryption onto a CCM accelerator of its own.
mod irqs {
    use embassy_nrf::interrupt::typelevel;

    #[cfg(feature = "_nrf52")]
    pub type MpslLowPrio = typelevel::EGU0_SWI0;
    #[cfg(feature = "_nrf54l")]
    pub type MpslLowPrio = typelevel::SWI00;

    #[cfg(feature = "_nrf52")]
    pub type MpslRadio = typelevel::RADIO;
    #[cfg(feature = "_nrf54l")]
    pub type MpslRadio = typelevel::RADIO_0;

    #[cfg(feature = "_nrf52")]
    pub type MpslTimer = typelevel::TIMER0;
    #[cfg(feature = "_nrf54l")]
    pub type MpslTimer = typelevel::TIMER10;

    #[cfg(feature = "_nrf52")]
    pub type MpslRtc = typelevel::RTC0;
    #[cfg(feature = "_nrf54l")]
    pub type MpslRtc = typelevel::GRTC_3;

    #[cfg(feature = "_nrf52")]
    pub type RadioEgu = typelevel::EGU0_SWI0;
    #[cfg(feature = "_nrf54l")]
    pub type RadioEgu = typelevel::EGU10;

    #[cfg(feature = "_nrf52")]
    pub type RadioLpTimer = typelevel::RTC2;
    #[cfg(feature = "_nrf54l")]
    pub type RadioLpTimer = typelevel::GRTC_0;

    #[cfg(feature = "_nrf54l")]
    pub type RadioCcm = typelevel::AAR00_CCM00;
}

/// Bind `CLOCK_POWER` to this interrupt handler
#[cfg(feature = "_nrf54l")]
pub use nrf_mpsl::ClockInterruptHandler as NrfThreadClockInterruptHandler;
/// Bind `RADIO_0`, `TIMER10` and `GRTC_3` to this interrupt handler.
///
/// Unlike on the nRF52 family there is no pure-Rust PHY driver here to arbitrate
/// with over the radio, so this is plainly MPSL's own handler.
#[cfg(feature = "_nrf54l")]
pub use nrf_mpsl::HighPrioInterruptHandler as NrfThreadHighPrioInterruptHandler;
/// Bind `SWI00` to this interrupt handler
#[cfg(feature = "_nrf54l")]
pub use nrf_mpsl::LowPrioInterruptHandler as NrfThreadLowPrioInterruptHandler;

/// Bind `RADIO`, `TIMER0` and `RTC0` to this interrupt handler
///
/// The pure-Rust PHY driver and the BLE controller both want the `RADIO`
/// peripheral (and the timers MPSL keeps next to it), and only one of them runs
/// at a time, so this hands each interrupt to whichever currently owns the radio.
#[cfg(feature = "_nrf52")]
pub struct NrfThreadHighPrioInterruptHandler;
#[cfg(feature = "_nrf52")]
impl Handler<interrupt::typelevel::RADIO> for NrfThreadHighPrioInterruptHandler {
    unsafe fn on_interrupt() {
        if RUST_RADIO_STATE.irq_enabled() {
            // Call the IEEE 802.15.4 driver interrupt handler, if the driver is enabled
            embassy_nrf::radio::InterruptHandler::<RADIO>::on_interrupt();
        } else {
            <NrfBleHighPrioInterruptHandler as Handler<interrupt::typelevel::RADIO>>::on_interrupt(
            );
        }
    }
}
#[cfg(feature = "_nrf52")]
impl Handler<interrupt::typelevel::TIMER0> for NrfThreadHighPrioInterruptHandler {
    unsafe fn on_interrupt() {
        if !RUST_RADIO_STATE.irq_enabled() {
            <NrfBleHighPrioInterruptHandler as Handler<interrupt::typelevel::TIMER0>>::on_interrupt(
            );
        }
    }
}
#[cfg(feature = "_nrf52")]
impl Handler<interrupt::typelevel::RTC0> for NrfThreadHighPrioInterruptHandler {
    unsafe fn on_interrupt() {
        if !RUST_RADIO_STATE.irq_enabled() {
            <NrfBleHighPrioInterruptHandler as Handler<interrupt::typelevel::RTC0>>::on_interrupt();
        }
    }
}

/// Bind `EGU0_SWI0` to this interrupt handler
#[cfg(feature = "_nrf52")]
pub struct NrfThreadLowPrioInterruptHandler;
#[cfg(feature = "_nrf52")]
impl<T: Interrupt> Handler<T> for NrfThreadLowPrioInterruptHandler {
    unsafe fn on_interrupt() {
        if !RUST_RADIO_STATE.irq_enabled() {
            <NrfBleLowPrioInterruptHandler as Handler<T>>::on_interrupt();
        }
    }
}

/// Bind `CLOCK_POWER` to this interrupt handler
#[cfg(feature = "_nrf52")]
pub struct NrfThreadClockInterruptHandler;
#[cfg(feature = "_nrf52")]
impl Handler<interrupt::typelevel::CLOCK_POWER> for NrfThreadClockInterruptHandler {
    unsafe fn on_interrupt() {
        if !RUST_RADIO_STATE.irq_enabled() {
            <NrfBleClockInterruptHandler as Handler<interrupt::typelevel::CLOCK_POWER>>::on_interrupt();
        }
    }
}

/// Compile-time proof that the interrupts MPSL and the SoftDevice Controller need
/// have been bound with [`embassy_nrf::bind_interrupts!`].
///
/// Blanket-implemented; you never implement this yourself.
pub trait NrfBleInterruptBindings:
    Binding<irqs::MpslLowPrio, NrfThreadLowPrioInterruptHandler>
    + Binding<irqs::MpslRadio, NrfThreadHighPrioInterruptHandler>
    + Binding<irqs::MpslTimer, NrfThreadHighPrioInterruptHandler>
    + Binding<irqs::MpslRtc, NrfThreadHighPrioInterruptHandler>
    + Binding<interrupt::typelevel::CLOCK_POWER, NrfThreadClockInterruptHandler>
{
}

impl<T> NrfBleInterruptBindings for T where
    T: Binding<irqs::MpslLowPrio, NrfThreadLowPrioInterruptHandler>
        + Binding<irqs::MpslRadio, NrfThreadHighPrioInterruptHandler>
        + Binding<irqs::MpslTimer, NrfThreadHighPrioInterruptHandler>
        + Binding<irqs::MpslRtc, NrfThreadHighPrioInterruptHandler>
        + Binding<interrupt::typelevel::CLOCK_POWER, NrfThreadClockInterruptHandler>
{
}

#[cfg(feature = "_nrf52")]
#[derive(Copy, Clone)]
struct NrfThreadRustRadioInterrupts;
#[cfg(feature = "_nrf52")]
unsafe impl
    Binding<<RADIO as Ieee802154Peripheral>::Interrupt, embassy_nrf::radio::InterruptHandler<RADIO>>
    for NrfThreadRustRadioInterrupts
{
}

#[derive(Copy, Clone)]
struct NrfBleControllerInterrupts;
unsafe impl<T> Binding<T, NrfBleLowPrioInterruptHandler> for NrfBleControllerInterrupts where
    T: Interrupt
{
}
unsafe impl Binding<irqs::MpslRadio, NrfBleHighPrioInterruptHandler>
    for NrfBleControllerInterrupts
{
}
unsafe impl Binding<irqs::MpslTimer, NrfBleHighPrioInterruptHandler>
    for NrfBleControllerInterrupts
{
}
unsafe impl Binding<irqs::MpslRtc, NrfBleHighPrioInterruptHandler> for NrfBleControllerInterrupts {}
unsafe impl Binding<interrupt::typelevel::CLOCK_POWER, NrfBleClockInterruptHandler>
    for NrfBleControllerInterrupts
{
}

/// Re-states, for the benefit of the 802.15.4 driver, the bindings that the
/// caller of `NrfThreadMpslRadioDriver::new` had to prove it made.
#[derive(Copy, Clone)]
struct NrfThreadMpslRadioInterrupts;
unsafe impl Binding<irqs::RadioEgu, EguInterruptHandler> for NrfThreadMpslRadioInterrupts {}
unsafe impl Binding<irqs::RadioLpTimer, LpTimerInterruptHandler> for NrfThreadMpslRadioInterrupts {}
#[cfg(feature = "_nrf54l")]
unsafe impl Binding<irqs::RadioCcm, CcmInterruptHandler> for NrfThreadMpslRadioInterrupts {}

#[cfg(feature = "_nrf52")]
struct RustRadioState {
    irq_enabled: AtomicBool,
    enable: Signal<bool, CriticalSectionRawMutex>,
    state: Signal<bool, CriticalSectionRawMutex>,
}

#[cfg(feature = "_nrf52")]
impl RustRadioState {
    const fn new() -> Self {
        Self {
            irq_enabled: AtomicBool::new(false),
            enable: Signal::new(false),
            state: Signal::new(false),
        }
    }

    fn irq_enabled(&self) -> bool {
        self.irq_enabled.load(Ordering::SeqCst)
    }

    fn set_enabled(&self, enabled: bool) {
        self.irq_enabled.store(enabled, Ordering::SeqCst);
        self.enable.modify(|state| {
            if *state != enabled {
                *state = enabled;
                (true, ())
            } else {
                (false, ())
            }
        });
    }

    fn set_enabled_state(&self, enabled: bool) {
        self.state.modify(|state| {
            if *state != enabled {
                *state = enabled;
                (true, ())
            } else {
                (false, ())
            }
        });
    }

    async fn wait_enabled(&self, enabled: bool) {
        self.enable
            .wait(|state| (*state == enabled).then_some(()))
            .await;
    }

    pub(crate) async fn wait_enabled_state(&self, enabled: bool) {
        self.state
            .wait(|state| (*state == enabled).then_some(()))
            .await;
    }
}

/// Hand the radio hardware over to MPSL and wait until the pure-Rust PHY runner
/// has actually let go of it.
///
/// A no-op where that runner does not exist, i.e. everywhere but the nRF52 family.
async fn yield_radio_to_mpsl() {
    #[cfg(feature = "_nrf52")]
    {
        RUST_RADIO_STATE.set_enabled(false);
        RUST_RADIO_STATE.wait_enabled_state(false).await;
    }
}

/// Re-borrow a peripheral bundle, so that the driver owning it can be torn down
/// and built again on every BLE <-> Thread transition.
macro_rules! reborrow {
    ($ty:ident, $p:expr, [$($field:ident),* $(,)?]) => {
        $ty { $($field: $p.$field.reborrow()),* }
    };
}

#[cfg(feature = "_nrf52")]
fn reborrow_mpsl<'t>(p: &'t mut NrfMpslPeripherals<'_>) -> NrfMpslPeripherals<'t> {
    reborrow!(
        NrfMpslPeripherals,
        p,
        [rtc0, timer0, temp, ppi_ch19, ppi_ch30, ppi_ch31]
    )
}

#[cfg(feature = "_nrf54l")]
fn reborrow_mpsl<'t>(p: &'t mut NrfMpslPeripherals<'_>) -> NrfMpslPeripherals<'t> {
    reborrow!(
        NrfMpslPeripherals,
        p,
        [
            grtc_ch7, grtc_ch8, grtc_ch9, grtc_ch10, grtc_ch11, timer10, timer20, temp, ppi10_ch0,
            ppi20_ch1, ppib11_ch0, ppib21_ch0,
        ]
    )
}

#[cfg(feature = "_nrf52")]
fn reborrow_sdc<'t>(p: &'t mut NrfSdcPeripherals<'_>) -> NrfSdcPeripherals<'t> {
    reborrow!(
        NrfSdcPeripherals,
        p,
        [
            ppi_ch17, ppi_ch18, ppi_ch20, ppi_ch21, ppi_ch22, ppi_ch23, ppi_ch24, ppi_ch25,
            ppi_ch26, ppi_ch27, ppi_ch28, ppi_ch29,
        ]
    )
}

#[cfg(feature = "_nrf54l")]
fn reborrow_sdc<'t>(p: &'t mut NrfSdcPeripherals<'_>) -> NrfSdcPeripherals<'t> {
    reborrow!(
        NrfSdcPeripherals,
        p,
        [
            ppi00_ch1, ppi00_ch3, ppi10_ch1, ppi10_ch2, ppi10_ch3, ppi10_ch4, ppi10_ch5, ppi10_ch6,
            ppi10_ch7, ppi10_ch8, ppi10_ch9, ppi10_ch10, ppi10_ch11, ppib00_ch1, ppib00_ch2,
            ppib00_ch3, ppib10_ch1, ppib10_ch2, ppib10_ch3,
        ]
    )
}

#[cfg(feature = "_nrf52")]
fn reborrow_ieee802154<'t>(
    p: &'t mut NrfIeee802154Peripherals<'_>,
) -> NrfIeee802154Peripherals<'t> {
    reborrow!(NrfIeee802154Peripherals, p, [egu, hp_timer, lp_timer])
}

#[cfg(feature = "_nrf54l")]
fn reborrow_ieee802154<'t>(
    p: &'t mut NrfIeee802154Peripherals<'_>,
) -> NrfIeee802154Peripherals<'t> {
    reborrow!(
        NrfIeee802154Peripherals,
        p,
        [
            grtc_ch3, grtc_ch4, grtc_ch5, ppi20_ch2, ppi20_ch3, ppib11_ch1, ppib21_ch1, ppib11_ch2,
            ppib21_ch2,
        ]
    )
}

/// A runner for the NRF52 PHY radio implemented in pure Rust
/// Needs to run in a high-prio execution context
#[cfg(feature = "_nrf52")]
pub struct NrfThreadRustRadioRunner<'a, 'd> {
    runner: PhyRadioRunner<'a>,
    radio_peripheral: Peri<'d, RADIO>,
    mac_radio_resources: MacRadioResources,
}

#[cfg(feature = "_nrf52")]
impl<'a, 'd> NrfThreadRustRadioRunner<'a, 'd> {
    /// Create a new instance of the `NrfThreadRadioRunner` type.
    fn new(runner: PhyRadioRunner<'a>, radio_peripheral: Peri<'d, RADIO>) -> Self {
        Self {
            runner,
            radio_peripheral,
            mac_radio_resources: MacRadioResources::new(),
        }
    }

    /// Run the PHY radio
    pub async fn run(&mut self) -> ! {
        loop {
            RUST_RADIO_STATE.wait_enabled(true).await;

            {
                RUST_RADIO_STATE.set_enabled_state(true);

                info!("Thread radio started");

                // The proxy runner requires a full-MAC radio, so the bare PHY
                // gets the software MAC wrapped around it right here, on the
                // high-priority side - which is what lets the software MAC's
                // ACK deadlines be served from this execution context.
                let radio = MacRadio::new(
                    NrfRadio::new(embassy_nrf::radio::ieee802154::Radio::new(
                        self.radio_peripheral.reborrow(),
                        NrfThreadRustRadioInterrupts,
                    )),
                    EmbassyTimeTimer,
                    &mut self.mac_radio_resources,
                );

                let mut cmd = pin!(RUST_RADIO_STATE.wait_enabled(false));
                let mut runner = pin!(self.runner.run(radio));

                select(&mut cmd, &mut runner).await;

                RUST_RADIO_STATE.set_enabled_state(false);

                info!("Thread radio stopped");
            }
        }
    }
}

/// An MPSL driver helper.
struct MpslDriver<'d> {
    p: NrfMpslPeripherals<'d>,
}

impl<'d> MpslDriver<'d> {
    fn new(p: NrfMpslPeripherals<'d>) -> Self {
        Self { p }
    }

    async fn create_mpsl(&mut self) -> Result<MultiprotocolServiceLayer<'_>, Error> {
        yield_radio_to_mpsl().await;

        let lfclk_cfg = nrf_mpsl::raw::mpsl_clock_lfclk_cfg_t {
            source: nrf_mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
            rc_ctiv: nrf_mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
            rc_temp_ctiv: nrf_mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
            accuracy_ppm: nrf_mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
            skip_wait_lfclk_started: nrf_mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
        };

        MultiprotocolServiceLayer::new::<irqs::MpslLowPrio, _>(
            reborrow_mpsl(&mut self.p),
            NrfBleControllerInterrupts,
            lfclk_cfg,
        )
        .map_err(to_matter_err)
    }
}

/// An SDC driver helper.
struct SdcDriver<'d> {
    p: NrfSdcPeripherals<'d>,
}

impl<'d> SdcDriver<'d> {
    fn new(p: NrfSdcPeripherals<'d>) -> Self {
        Self { p }
    }

    fn create_controller<'t>(
        &'t mut self,
        mpsl: &'t MultiprotocolServiceLayer<'_>,
        sdc_mem: &'t mut nrf_sdc::Mem<3084>,
        rng: &'t mut (impl rand_core::CryptoRng + Send),
    ) -> Result<SoftdeviceController<'t>, Error> {
        nrf_sdc::Builder::new()
            .map_err(to_matter_err)?
            .support_adv()
            .support_peripheral()
            .peripheral_count(1)
            .map_err(to_matter_err)?
            .buffer_cfg(L2CAP_MTU as _, L2CAP_MTU as _, L2CAP_TXQ, L2CAP_RXQ)
            .map_err(to_matter_err)?
            .build(reborrow_sdc(&mut self.p), rng, mpsl, sdc_mem)
            .map_err(to_matter_err)
    }
}

/// An IEEE 802.15.4 driver helper.
struct IEEE802154Driver<'d> {
    radio: Peri<'d, RADIO>,
    p: NrfIeee802154Peripherals<'d>,
}

impl<'d> IEEE802154Driver<'d> {
    fn new(radio: Peri<'d, RADIO>, p: NrfIeee802154Peripherals<'d>) -> Self {
        Self { radio, p }
    }

    fn create_radio<'t>(
        &'t mut self,
        mpsl: &'t MultiprotocolServiceLayer<'_>,
    ) -> Result<nrf_802154::OpenThreadRadio<'t>, Error> {
        Ok(nrf_802154::OpenThreadRadio::new(nrf_802154::Radio::new(
            self.radio.reborrow(),
            reborrow_ieee802154(&mut self.p),
            NrfThreadMpslRadioInterrupts,
            mpsl,
        )))
    }
}

/// A `ThreadDriver` implementation for the NRF52 family of chips based on the Radio driver in `embassy-nrf`.
///
/// Note that the Radio driver in `embassy-nrf` is not capable of co-existence with BLE and is also relatively simplistic.
#[cfg(feature = "_nrf52")]
pub struct NrfThreadRustRadioDriver<'d, R> {
    mpsl: MpslDriver<'d>,
    sdc: SdcDriver<'d>,
    proxy: ProxyRadio<'d>,
    rand: R,
}

#[cfg(feature = "_nrf52")]
impl<'d, R> NrfThreadRustRadioDriver<'d, R> {
    /// Create a new instance of the `NrfThreadRustRadioDriver` type.
    ///
    /// # Arguments
    /// - `resources` - The resources for the radio proxying
    /// - `radio` - The radio peripheral instance
    /// - `mpsl_peripherals` - The peripherals MPSL takes ownership of
    /// - `sdc_peripherals` - The peripherals the SoftDevice Controller takes ownership of
    /// - `rand` - The random number generator
    /// - `_irqs` - The interrupt handlers
    pub fn new<I>(
        resources: &'d mut ProxyRadioResources,
        radio: Peri<'d, RADIO>,
        mpsl_peripherals: NrfMpslPeripherals<'d>,
        sdc_peripherals: NrfSdcPeripherals<'d>,
        rand: R,
        _irqs: I,
    ) -> (Self, NrfThreadRustRadioRunner<'d, 'd>)
    where
        I: NrfBleInterruptBindings
            + Binding<<RADIO as Ieee802154Peripheral>::Interrupt, NrfThreadHighPrioInterruptHandler>
            + 'd,
    {
        let (proxy, proxy_runner) = ProxyRadio::new(resources);

        let runner = NrfThreadRustRadioRunner::new(proxy_runner, radio);

        (
            Self {
                mpsl: MpslDriver::new(mpsl_peripherals),
                sdc: SdcDriver::new(sdc_peripherals),
                proxy,
                rand,
            },
            runner,
        )
    }
}

#[cfg(feature = "_nrf52")]
impl<R> super::ThreadDriver for NrfThreadRustRadioDriver<'_, R> {
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: super::ThreadDriverTask,
    {
        info!("About to enable Thread radio");

        let _guard = scopeguard::guard((), |_| RUST_RADIO_STATE.set_enabled(false));

        RUST_RADIO_STATE.set_enabled(true);
        RUST_RADIO_STATE.wait_enabled_state(true).await;

        info!("Running Thread radio task");

        task.run(&mut self.proxy).await
    }
}

#[cfg(feature = "_nrf52")]
impl<R> super::BleDriver for NrfThreadRustRadioDriver<'_, R>
where
    R: CryptoRngCore + Copy, /*+ Send*/
{
    async fn run<T>(&mut self, mut task: T) -> Result<(), Error>
    where
        T: super::BleDriverTask,
    {
        let mpsl = self.mpsl.create_mpsl().await?;

        // TODO: Externalize as resources
        // Mem is roughly MAC_CONNECTIONS * L2CAP_MTU * L2CAP_TXQ * L2CAP_RXQ
        let mut sdc_mem = nrf_sdc::Mem::<3084>::new();
        let mut rand = RngAdaptor::new(SendHack(self.rand));

        let controller = self.sdc.create_controller(&mpsl, &mut sdc_mem, &mut rand)?;

        // See the note in `ThreadDriver::run`: MPSL's low-priority processor must
        // be driven for the radio timeslots to be serviced.
        let mut mpsl_run = pin!(mpsl.run());
        let mut task = pin!(task.run(controller));

        let Either::Second(res) = select(&mut mpsl_run, &mut task).await;

        res
    }
}

/// A `ThreadDriver` implementation based on the IEEE 802.15.4 C driver from the
/// Nordic `nrfx` lib, running on top of MPSL.
///
/// This driver is capable of co-existence with BLE.
pub struct NrfThreadMpslRadioDriver<'d, R> {
    mpsl: MpslDriver<'d>,
    sdc: SdcDriver<'d>,
    ieee802154: IEEE802154Driver<'d>,
    rand: R,
}

impl<'d, R> NrfThreadMpslRadioDriver<'d, R> {
    /// Create a new instance of the `NrfThreadMpslRadioDriver` type.
    ///
    /// # Arguments
    /// - `radio` - The radio peripheral instance
    /// - `mpsl_peripherals` - The peripherals MPSL takes ownership of
    /// - `sdc_peripherals` - The peripherals the SoftDevice Controller takes ownership of
    /// - `ieee802154_peripherals` - The peripherals the IEEE 802.15.4 driver takes ownership of
    /// - `rand` - The random number generator
    /// - `_irqs` - The interrupt handlers
    pub fn new<I>(
        radio: Peri<'d, RADIO>,
        mpsl_peripherals: NrfMpslPeripherals<'d>,
        sdc_peripherals: NrfSdcPeripherals<'d>,
        ieee802154_peripherals: NrfIeee802154Peripherals<'d>,
        rand: R,
        _irqs: I,
    ) -> Self
    where
        I: NrfBleInterruptBindings + nrf_802154::InterruptBindings + 'd,
    {
        Self {
            mpsl: MpslDriver::new(mpsl_peripherals),
            sdc: SdcDriver::new(sdc_peripherals),
            ieee802154: IEEE802154Driver::new(radio, ieee802154_peripherals),
            rand,
        }
    }
}

impl<R> super::ThreadDriver for NrfThreadMpslRadioDriver<'_, R> {
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: super::ThreadDriverTask,
    {
        let mpsl = self.mpsl.create_mpsl().await?;

        let radio = self.ieee802154.create_radio(&mpsl)?;

        // MPSL's low-priority work processor (`mpsl.run()` -> `mpsl_low_priority_process()`)
        // MUST be driven, or the 802.15.4 timeslot session is never serviced: a
        // scheduled TX never completes and RX never re-arms, so the radio wedges
        // after the very first frame and Thread cannot attach.
        let mut mpsl_run = pin!(mpsl.run());
        let mut task = pin!(task.run(radio));

        let Either::Second(res) = select(&mut mpsl_run, &mut task).await;

        res
    }
}

impl<R> super::BleDriver for NrfThreadMpslRadioDriver<'_, R>
where
    R: CryptoRngCore + Copy, /*+ Send*/
{
    async fn run<T>(&mut self, mut task: T) -> Result<(), Error>
    where
        T: super::BleDriverTask,
    {
        let mpsl = self.mpsl.create_mpsl().await?;

        // TODO: Externalize as resources
        // Mem is roughly MAC_CONNECTIONS * L2CAP_MTU * L2CAP_TXQ * L2CAP_RXQ
        let mut sdc_mem = nrf_sdc::Mem::<3084>::new();
        let mut rand = RngAdaptor::new(SendHack(self.rand));

        let controller = self.sdc.create_controller(&mpsl, &mut sdc_mem, &mut rand)?;

        // See the note in `ThreadDriver::run`: MPSL's low-priority processor must
        // be driven for the radio timeslots to be serviced.
        let mut mpsl_run = pin!(mpsl.run());
        let mut task = pin!(task.run(controller));

        let Either::Second(res) = select(&mut mpsl_run, &mut task).await;

        res
    }
}

impl<R> super::ThreadCoexDriver for NrfThreadMpslRadioDriver<'_, R>
where
    R: CryptoRngCore + Copy, /*+ Send*/
{
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: super::ThreadCoexDriverTask,
    {
        let mpsl = self.mpsl.create_mpsl().await?;

        // TODO: Externalize as resources
        // Mem is roughly MAC_CONNECTIONS * L2CAP_MTU * L2CAP_TXQ * L2CAP_RXQ
        let mut sdc_mem = nrf_sdc::Mem::<3084>::new();
        let mut rand = RngAdaptor::new(SendHack(self.rand));

        let controller = self.sdc.create_controller(&mpsl, &mut sdc_mem, &mut rand)?;

        let radio = self.ieee802154.create_radio(&mpsl)?;

        // See the note in `ThreadDriver::run`: MPSL's low-priority processor must
        // be driven for the radio timeslots to be serviced.
        let mut mpsl_run = pin!(mpsl.run());
        let mut task = pin!(task.run(radio, controller));

        let Either::Second(res) = select(&mut mpsl_run, &mut task).await;

        res
    }
}

fn to_matter_err<E: core::fmt::Debug>(e: E) -> Error {
    error!("BLE error: {:?}", debug2format!(e));
    Error::new(ErrorCode::BtpError)
}

// TODO: figure out if we need to enforce `Send` on the rand returned by the `Crypto` trait in `rs-matter`
struct SendHack<T>(T);

impl<T> RngCore for SendHack<T>
where
    T: RngCore,
{
    fn next_u32(&mut self) -> u32 {
        self.0.next_u32()
    }

    fn next_u64(&mut self) -> u64 {
        self.0.next_u64()
    }

    fn fill_bytes(&mut self, dest: &mut [u8]) {
        self.0.fill_bytes(dest)
    }

    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand::Error> {
        self.0.try_fill_bytes(dest)
    }
}

impl<T> CryptoRng for SendHack<T> where T: CryptoRng + RngCore {}

unsafe impl<T> Send for SendHack<T> {}

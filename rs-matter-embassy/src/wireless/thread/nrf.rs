use core::pin::pin;

use embassy_futures::select::select;

use embassy_nrf::interrupt;
use embassy_nrf::interrupt::typelevel::Interrupt;
use embassy_nrf::interrupt::typelevel::{Binding, Handler};
use embassy_nrf::peripherals::{
    PPI_CH17, PPI_CH18, PPI_CH19, PPI_CH20, PPI_CH21, PPI_CH22, PPI_CH23, PPI_CH24, PPI_CH25,
    PPI_CH26, PPI_CH27, PPI_CH28, PPI_CH29, PPI_CH30, PPI_CH31, RADIO, RTC0, TEMP, TIMER0,
};
use embassy_nrf::radio::InterruptHandler;
use embassy_nrf::Peri;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use nrf_mpsl::MultiprotocolServiceLayer;

use nrf_sdc::mpsl::{
    ClockInterruptHandler as NrfBleClockInterruptHandler,
    HighPrioInterruptHandler as NrfBleHighPrioInterruptHandler,
    LowPrioInterruptHandler as NrfBleLowPrioInterruptHandler,
};
use nrf_sdc::SoftdeviceController;

use openthread::nrf::Ieee802154Peripheral;
use openthread::nrf::NrfRadio;
use openthread::{EmbassyTimeTimer, PhyRadioRunner, ProxyRadio, ProxyRadioResources};

pub use openthread::ProxyRadioResources as NrfThreadRadioResources;

use portable_atomic::{AtomicBool, Ordering};

use rand_core::CryptoRng;

use rs_matter_stack::matter::error::{Error, ErrorCode};
use rs_matter_stack::matter::utils::rand::Rand;
use rs_matter_stack::matter::utils::sync::Signal;
use rs_matter_stack::rand::MatterRngCore;

use crate::ble::MAX_MTU_SIZE;

/// How many outgoing L2CAP buffers per link
const L2CAP_TXQ: u8 = 3;
/// How many incoming L2CAP buffers per link
const L2CAP_RXQ: u8 = 3;

static RUST_RADIO_STATE: RustRadioState = RustRadioState::new();

/// Bind `RADIO`, `TIMER0` and `RTC0` to this interrupt handler
pub struct NrfThreadHighPrioInterruptHandler;
impl Handler<interrupt::typelevel::RADIO> for NrfThreadHighPrioInterruptHandler {
    unsafe fn on_interrupt() {
        if RUST_RADIO_STATE.irq_enabled() {
            // Call the IEEE 802.15.4 driver interrupt handler, if the driver is enabled
            InterruptHandler::<embassy_nrf::peripherals::RADIO>::on_interrupt();
        } else {
            <NrfBleHighPrioInterruptHandler as Handler<interrupt::typelevel::RADIO>>::on_interrupt(
            );
        }
    }
}
impl Handler<interrupt::typelevel::TIMER0> for NrfThreadHighPrioInterruptHandler {
    unsafe fn on_interrupt() {
        if !RUST_RADIO_STATE.irq_enabled() {
            <NrfBleHighPrioInterruptHandler as Handler<interrupt::typelevel::TIMER0>>::on_interrupt(
            );
        }
    }
}
impl Handler<interrupt::typelevel::RTC0> for NrfThreadHighPrioInterruptHandler {
    unsafe fn on_interrupt() {
        if !RUST_RADIO_STATE.irq_enabled() {
            <NrfBleHighPrioInterruptHandler as Handler<interrupt::typelevel::RTC0>>::on_interrupt();
        }
    }
}

/// Bind `EGU0_SWI0` to this interrupt handler
pub struct NrfThreadLowPrioInterruptHandler;
impl<T: Interrupt> Handler<T> for NrfThreadLowPrioInterruptHandler {
    unsafe fn on_interrupt() {
        if !RUST_RADIO_STATE.irq_enabled() {
            <NrfBleLowPrioInterruptHandler as Handler<T>>::on_interrupt();
        }
    }
}

/// Bind `CLOCK_POWER` to this interrupt handler
pub struct NrfThreadClockInterruptHandler;
impl Handler<interrupt::typelevel::CLOCK_POWER> for NrfThreadClockInterruptHandler {
    unsafe fn on_interrupt() {
        if !RUST_RADIO_STATE.irq_enabled() {
            <NrfBleClockInterruptHandler as Handler::<interrupt::typelevel::CLOCK_POWER>>::on_interrupt();
        }
    }
}

struct NrfThreadRadioInterrupts;
unsafe impl Binding<<RADIO as Ieee802154Peripheral>::Interrupt, InterruptHandler<RADIO>>
    for NrfThreadRadioInterrupts
{
}

struct NrfBleControllerInterrupts;
unsafe impl<T> Binding<T, NrfBleLowPrioInterruptHandler> for NrfBleControllerInterrupts where
    T: interrupt::typelevel::Interrupt
{
}
unsafe impl Binding<interrupt::typelevel::RADIO, NrfBleHighPrioInterruptHandler>
    for NrfBleControllerInterrupts
{
}
unsafe impl Binding<interrupt::typelevel::TIMER0, NrfBleHighPrioInterruptHandler>
    for NrfBleControllerInterrupts
{
}
unsafe impl Binding<interrupt::typelevel::RTC0, NrfBleHighPrioInterruptHandler>
    for NrfBleControllerInterrupts
{
}
unsafe impl Binding<interrupt::typelevel::CLOCK_POWER, NrfBleClockInterruptHandler>
    for NrfBleControllerInterrupts
{
}

struct RustRadioState {
    irq_enabled: AtomicBool,
    enable: Signal<CriticalSectionRawMutex, bool>,
    state: Signal<CriticalSectionRawMutex, bool>,
}

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

/// A runner for the NRF52 PHY radio implemented in pure Rust
/// Needs to run in a high-prio execution context
pub struct NrfThreadRustRadioRunner<'a, 'd> {
    runner: PhyRadioRunner<'a>,
    radio_peripheral: Peri<'d, RADIO>,
}

impl<'a, 'd> NrfThreadRustRadioRunner<'a, 'd> {
    /// Create a new instance of the `NrfThreadRadioRunner` type.
    fn new(runner: PhyRadioRunner<'a>, radio_peripheral: Peri<'d, RADIO>) -> Self {
        Self {
            runner,
            radio_peripheral,
        }
    }

    /// Run the PHY radio
    pub async fn run(&mut self) -> ! {
        loop {
            RUST_RADIO_STATE.wait_enabled(true).await;

            {
                RUST_RADIO_STATE.set_enabled_state(true);

                info!("Thread radio started");

                let radio = NrfRadio::new(embassy_nrf::radio::ieee802154::Radio::new(
                    self.radio_peripheral.reborrow(),
                    NrfThreadRadioInterrupts,
                ));

                let mut cmd = pin!(RUST_RADIO_STATE.wait_enabled(false));
                let mut runner = pin!(self.runner.run(radio, EmbassyTimeTimer));

                select(&mut cmd, &mut runner).await;

                RUST_RADIO_STATE.set_enabled_state(false);

                info!("Thread radio stopped");
            }
        }
    }
}

struct MpslDriver<'d> {
    rtc0: Peri<'d, RTC0>,
    timer0: Peri<'d, TIMER0>,
    temp: Peri<'d, TEMP>,
    ppi_ch19: Peri<'d, PPI_CH19>,
    ppi_ch30: Peri<'d, PPI_CH30>,
    ppi_ch31: Peri<'d, PPI_CH31>,
}

impl<'d> MpslDriver<'d> {
    /// Create a new instance of the `NrfMpslDriver` type.
    ///
    /// # Arguments
    /// - `rtc0` - The RTC0 peripheral instance
    /// - `timer0` - The TIMER0 peripheral instance
    /// - `temp` - The TEMP peripheral instance
    /// - `ppi_ch19` - The PPI channel 19 peripheral instance
    /// - `ppi_ch30` - The PPI channel 30 peripheral instance
    /// - `ppi_ch31` - The PPI channel 31 peripheral instance
    /// - `_irqs` - The interrupt handlers
    #[allow(clippy::too_many_arguments)]
    fn new<T, I>(
        rtc0: Peri<'d, RTC0>,
        timer0: Peri<'d, TIMER0>,
        temp: Peri<'d, TEMP>,
        ppi_ch19: Peri<'d, PPI_CH19>,
        ppi_ch30: Peri<'d, PPI_CH30>,
        ppi_ch31: Peri<'d, PPI_CH31>,
        _irqs: I,
    ) -> Self
    where
        T: Interrupt,
        I: Binding<T, NrfThreadLowPrioInterruptHandler>
            + Binding<interrupt::typelevel::RADIO, NrfThreadHighPrioInterruptHandler>
            + Binding<interrupt::typelevel::TIMER0, NrfThreadHighPrioInterruptHandler>
            + Binding<interrupt::typelevel::RTC0, NrfThreadHighPrioInterruptHandler>
            + Binding<interrupt::typelevel::CLOCK_POWER, NrfThreadClockInterruptHandler>
            + Binding<
                <embassy_nrf::peripherals::RADIO as Ieee802154Peripheral>::Interrupt,
                NrfThreadHighPrioInterruptHandler,
            >,
    {
        Self {
            rtc0,
            timer0,
            temp,
            ppi_ch19,
            ppi_ch30,
            ppi_ch31,
        }
    }

    async fn create_mpsl(&mut self) -> Result<MultiprotocolServiceLayer<'_>, Error> {
        RUST_RADIO_STATE.set_enabled(false);
        RUST_RADIO_STATE.wait_enabled_state(false).await;

        let mpsl_p = nrf_sdc::mpsl::Peripherals::new(
            self.rtc0.reborrow(),
            self.timer0.reborrow(),
            self.temp.reborrow(),
            self.ppi_ch19.reborrow(),
            self.ppi_ch30.reborrow(),
            self.ppi_ch31.reborrow(),
        );

        let lfclk_cfg = nrf_sdc::mpsl::raw::mpsl_clock_lfclk_cfg_t {
            source: nrf_sdc::mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
            rc_ctiv: nrf_sdc::mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
            rc_temp_ctiv: nrf_sdc::mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
            accuracy_ppm: nrf_sdc::mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
            skip_wait_lfclk_started: nrf_sdc::mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
        };

        nrf_sdc::mpsl::MultiprotocolServiceLayer::new::<interrupt::typelevel::EGU0_SWI0, _>(
            mpsl_p,
            NrfBleControllerInterrupts,
            lfclk_cfg,
        )
        .map_err(to_matter_err)
    }
}

/// A `ThreadDriver` implementation for the NRF52 family of chips.
struct SdcDriver<'d> {
    ppi_ch17: Peri<'d, PPI_CH17>,
    ppi_ch18: Peri<'d, PPI_CH18>,
    ppi_ch20: Peri<'d, PPI_CH20>,
    ppi_ch21: Peri<'d, PPI_CH21>,
    ppi_ch22: Peri<'d, PPI_CH22>,
    ppi_ch23: Peri<'d, PPI_CH23>,
    ppi_ch24: Peri<'d, PPI_CH24>,
    ppi_ch25: Peri<'d, PPI_CH25>,
    ppi_ch26: Peri<'d, PPI_CH26>,
    ppi_ch27: Peri<'d, PPI_CH27>,
    ppi_ch28: Peri<'d, PPI_CH28>,
    ppi_ch29: Peri<'d, PPI_CH29>,
}

impl<'d> SdcDriver<'d> {
    fn new(
        ppi_ch17: Peri<'d, PPI_CH17>,
        ppi_ch18: Peri<'d, PPI_CH18>,
        ppi_ch20: Peri<'d, PPI_CH20>,
        ppi_ch21: Peri<'d, PPI_CH21>,
        ppi_ch22: Peri<'d, PPI_CH22>,
        ppi_ch23: Peri<'d, PPI_CH23>,
        ppi_ch24: Peri<'d, PPI_CH24>,
        ppi_ch25: Peri<'d, PPI_CH25>,
        ppi_ch26: Peri<'d, PPI_CH26>,
        ppi_ch27: Peri<'d, PPI_CH27>,
        ppi_ch28: Peri<'d, PPI_CH28>,
        ppi_ch29: Peri<'d, PPI_CH29>,
    ) -> Self {
        Self {
            ppi_ch17,
            ppi_ch18,
            ppi_ch20,
            ppi_ch21,
            ppi_ch22,
            ppi_ch23,
            ppi_ch24,
            ppi_ch25,
            ppi_ch26,
            ppi_ch27,
            ppi_ch28,
            ppi_ch29,
        }
    }

    fn create_controller<'t>(
        &'t mut self,
        mpsl: &'t MultiprotocolServiceLayer<'_>,
        sdc_mem: &'t mut nrf_sdc::Mem<3084>,
        rng: &'t mut (impl CryptoRng + Send),
    ) -> Result<SoftdeviceController<'t>, Error> {
        let sdc_p = nrf_sdc::Peripherals::new(
            self.ppi_ch17.reborrow(),
            self.ppi_ch18.reborrow(),
            self.ppi_ch20.reborrow(),
            self.ppi_ch21.reborrow(),
            self.ppi_ch22.reborrow(),
            self.ppi_ch23.reborrow(),
            self.ppi_ch24.reborrow(),
            self.ppi_ch25.reborrow(),
            self.ppi_ch26.reborrow(),
            self.ppi_ch27.reborrow(),
            self.ppi_ch28.reborrow(),
            self.ppi_ch29.reborrow(),
        );

        nrf_sdc::Builder::new()
            .map_err(to_matter_err)?
            .support_adv()
            .map_err(to_matter_err)?
            .support_peripheral()
            .map_err(to_matter_err)?
            .peripheral_count(1)
            .map_err(to_matter_err)?
            .buffer_cfg(MAX_MTU_SIZE as _, MAX_MTU_SIZE as _, L2CAP_TXQ, L2CAP_RXQ)
            .map_err(to_matter_err)?
            .build(sdc_p, rng, mpsl, sdc_mem)
            .map_err(to_matter_err)
    }
}

struct IEEE802154Driver<'d> {
    radio: Peri<'d, RADIO>,
    egu: Peri<'d, embassy_nrf::peripherals::EGU0>,
}

impl<'d> IEEE802154Driver<'d> {
    fn new(radio: Peri<'d, RADIO>, egu: Peri<'d, embassy_nrf::peripherals::EGU0>) -> Self {
        Self { radio, egu }
    }

    fn create_radio<'t>(
        &'t mut self,
        mpsl: &'t MultiprotocolServiceLayer<'_>,
    ) -> Result<nrf_802154::Radio<'t>, Error> {
        Ok(nrf_802154::Radio::new(
            self.radio.reborrow(),
            self.egu.reborrow(),
            mpsl,
        ))
    }
}

/// A `ThreadDriver` implementation for the NRF52 family of chips.
pub struct NrfRustRadioThreadDriver<'d> {
    mpsl: MpslDriver<'d>,
    sdc: SdcDriver<'d>,
    proxy: ProxyRadio<'d>,
    rand: Rand,
}

impl<'d> NrfRustRadioThreadDriver<'d> {
    /// Create a new instance of the `NrfThreadRadio` type.
    ///
    /// # Arguments
    /// - `resources` - The resources for the radio proxying
    /// - `radio` - The radio peripheral instance
    /// - `rtc0` - The RTC0 peripheral instance
    /// - `timer0` - The TIMER0 peripheral instance
    /// - `temp` - The TEMP peripheral instance
    /// - `ppi_ch17` - The PPI channel 17 peripheral instance
    /// - `ppi_ch18` - The PPI channel 18 peripheral instance
    /// - `ppi_ch19` - The PPI channel 19 peripheral instance
    /// - `ppi_ch20` - The PPI channel 20 peripheral instance
    /// - `ppi_ch21` - The PPI channel 21 peripheral instance
    /// - `ppi_ch22` - The PPI channel 22 peripheral instance
    /// - `ppi_ch23` - The PPI channel 23 peripheral instance
    /// - `ppi_ch24` - The PPI channel 24 peripheral instance
    /// - `ppi_ch25` - The PPI channel 25 peripheral instance
    /// - `ppi_ch26` - The PPI channel 26 peripheral instance
    /// - `ppi_ch27` - The PPI channel 27 peripheral instance
    /// - `ppi_ch28` - The PPI channel 28 peripheral instance
    /// - `ppi_ch29` - The PPI channel 29 peripheral instance
    /// - `ppi_ch30` - The PPI channel 30 peripheral instance
    /// - `ppi_ch31` - The PPI channel 31 peripheral instance
    /// - `rand` - The random number generator
    /// - `_irqs` - The interrupt handlers
    #[allow(clippy::too_many_arguments)]
    pub fn new<T, I>(
        resources: &'d mut ProxyRadioResources,
        radio: Peri<'d, RADIO>,
        rtc0: Peri<'d, RTC0>,
        timer0: Peri<'d, TIMER0>,
        temp: Peri<'d, TEMP>,
        ppi_ch17: Peri<'d, PPI_CH17>,
        ppi_ch18: Peri<'d, PPI_CH18>,
        ppi_ch19: Peri<'d, PPI_CH19>,
        ppi_ch20: Peri<'d, PPI_CH20>,
        ppi_ch21: Peri<'d, PPI_CH21>,
        ppi_ch22: Peri<'d, PPI_CH22>,
        ppi_ch23: Peri<'d, PPI_CH23>,
        ppi_ch24: Peri<'d, PPI_CH24>,
        ppi_ch25: Peri<'d, PPI_CH25>,
        ppi_ch26: Peri<'d, PPI_CH26>,
        ppi_ch27: Peri<'d, PPI_CH27>,
        ppi_ch28: Peri<'d, PPI_CH28>,
        ppi_ch29: Peri<'d, PPI_CH29>,
        ppi_ch30: Peri<'d, PPI_CH30>,
        ppi_ch31: Peri<'d, PPI_CH31>,
        rand: Rand,
        irqs: I,
    ) -> (Self, NrfThreadRustRadioRunner<'d, 'd>)
    where
        T: Interrupt,
        I: Binding<T, NrfThreadLowPrioInterruptHandler>
            + Binding<interrupt::typelevel::RADIO, NrfThreadHighPrioInterruptHandler>
            + Binding<interrupt::typelevel::TIMER0, NrfThreadHighPrioInterruptHandler>
            + Binding<interrupt::typelevel::RTC0, NrfThreadHighPrioInterruptHandler>
            + Binding<interrupt::typelevel::CLOCK_POWER, NrfThreadClockInterruptHandler>
            + Binding<
                <embassy_nrf::peripherals::RADIO as Ieee802154Peripheral>::Interrupt,
                NrfThreadHighPrioInterruptHandler,
            >,
    {
        let caps = openthread::Capabilities::empty();
        // TODO: A bit dirty as we create it and then drop it immediately
        // NrfRadio::new(embassy_nrf::radio::ieee802154::Radio::new(
        //     &mut radio_peripheral,
        //     NrfThreadRadioInterrupts,
        // ))
        // .caps();

        let (proxy, proxy_runner) = ProxyRadio::new(caps, resources);

        let runner = NrfThreadRustRadioRunner::new(proxy_runner, radio);

        (
            Self {
                mpsl: MpslDriver::new(rtc0, timer0, temp, ppi_ch19, ppi_ch30, ppi_ch31, irqs),
                sdc: SdcDriver::new(
                    ppi_ch17, ppi_ch18, ppi_ch20, ppi_ch21, ppi_ch22, ppi_ch23, ppi_ch24, ppi_ch25,
                    ppi_ch26, ppi_ch27, ppi_ch28, ppi_ch29,
                ),
                proxy,
                rand,
            },
            runner,
        )
    }
}

impl super::ThreadDriver for NrfRustRadioThreadDriver<'_> {
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

impl super::BleDriver for NrfRustRadioThreadDriver<'_> {
    async fn run<T>(&mut self, mut task: T) -> Result<(), Error>
    where
        T: super::BleDriverTask,
    {
        RUST_RADIO_STATE.set_enabled(false);
        RUST_RADIO_STATE.wait_enabled_state(false).await;

        let mpsl = self.mpsl.create_mpsl().await?;

        // TODO: Externalize as resources
        // Mem is roughly MAC_CONNECTIONS * MAX_MTU_SIZE * L2CAP_TXQ * L2CAP_RXQ
        let mut sdc_mem = nrf_sdc::Mem::<3084>::new();

        let mut rng = MatterRngCore::new(self.rand);

        let controller = self.sdc.create_controller(&mpsl, &mut sdc_mem, &mut rng)?;

        task.run(controller).await
    }
}

/// A `ThreadDriver` implementation for the NRF52 family of chips.
pub struct NrfMpslThreadDriver<'d> {
    mpsl: MpslDriver<'d>,
    sdc: SdcDriver<'d>,
    ieee802154: IEEE802154Driver<'d>,
    rand: Rand,
}

impl<'d> NrfMpslThreadDriver<'d> {
    /// Create a new instance of the `NrfThreadRadio` type.
    ///
    /// # Arguments
    /// - `radio` - The radio peripheral instance
    /// - `egu` - The EGU0 peripheral instance
    /// - `rtc0` - The RTC0 peripheral instance
    /// - `timer0` - The TIMER0 peripheral instance
    /// - `temp` - The TEMP peripheral instance
    /// - `ppi_ch17` - The PPI channel 17 peripheral instance
    /// - `ppi_ch18` - The PPI channel 18 peripheral instance
    /// - `ppi_ch19` - The PPI channel 19 peripheral instance
    /// - `ppi_ch20` - The PPI channel 20 peripheral instance
    /// - `ppi_ch21` - The PPI channel 21 peripheral instance
    /// - `ppi_ch22` - The PPI channel 22 peripheral instance
    /// - `ppi_ch23` - The PPI channel 23 peripheral instance
    /// - `ppi_ch24` - The PPI channel 24 peripheral instance
    /// - `ppi_ch25` - The PPI channel 25 peripheral instance
    /// - `ppi_ch26` - The PPI channel 26 peripheral instance
    /// - `ppi_ch27` - The PPI channel 27 peripheral instance
    /// - `ppi_ch28` - The PPI channel 28 peripheral instance
    /// - `ppi_ch29` - The PPI channel 29 peripheral instance
    /// - `ppi_ch30` - The PPI channel 30 peripheral instance
    /// - `ppi_ch31` - The PPI channel 31 peripheral instance
    /// - `rand` - The random number generator
    /// - `_irqs` - The interrupt handlers
    #[allow(clippy::too_many_arguments)]
    pub fn new<T, I>(
        radio: Peri<'d, RADIO>,
        egu: Peri<'d, embassy_nrf::peripherals::EGU0>,
        rtc0: Peri<'d, RTC0>,
        timer0: Peri<'d, TIMER0>,
        temp: Peri<'d, TEMP>,
        ppi_ch17: Peri<'d, PPI_CH17>,
        ppi_ch18: Peri<'d, PPI_CH18>,
        ppi_ch19: Peri<'d, PPI_CH19>,
        ppi_ch20: Peri<'d, PPI_CH20>,
        ppi_ch21: Peri<'d, PPI_CH21>,
        ppi_ch22: Peri<'d, PPI_CH22>,
        ppi_ch23: Peri<'d, PPI_CH23>,
        ppi_ch24: Peri<'d, PPI_CH24>,
        ppi_ch25: Peri<'d, PPI_CH25>,
        ppi_ch26: Peri<'d, PPI_CH26>,
        ppi_ch27: Peri<'d, PPI_CH27>,
        ppi_ch28: Peri<'d, PPI_CH28>,
        ppi_ch29: Peri<'d, PPI_CH29>,
        ppi_ch30: Peri<'d, PPI_CH30>,
        ppi_ch31: Peri<'d, PPI_CH31>,
        rand: Rand,
        _irqs: I,
    ) -> Self
    where
        T: Interrupt,
        I: Binding<T, NrfThreadLowPrioInterruptHandler>
            + Binding<interrupt::typelevel::RADIO, NrfThreadHighPrioInterruptHandler>
            + Binding<interrupt::typelevel::TIMER0, NrfThreadHighPrioInterruptHandler>
            + Binding<interrupt::typelevel::RTC0, NrfThreadHighPrioInterruptHandler>
            + Binding<interrupt::typelevel::CLOCK_POWER, NrfThreadClockInterruptHandler>
            + Binding<
                <embassy_nrf::peripherals::RADIO as Ieee802154Peripheral>::Interrupt,
                NrfThreadHighPrioInterruptHandler,
            >,
    {
        Self {
            mpsl: MpslDriver::new(rtc0, timer0, temp, ppi_ch19, ppi_ch30, ppi_ch31, _irqs),
            sdc: SdcDriver::new(
                ppi_ch17, ppi_ch18, ppi_ch20, ppi_ch21, ppi_ch22, ppi_ch23, ppi_ch24, ppi_ch25,
                ppi_ch26, ppi_ch27, ppi_ch28, ppi_ch29,
            ),
            ieee802154: IEEE802154Driver::new(radio, egu),
            rand,
        }
    }
}

impl super::ThreadDriver for NrfMpslThreadDriver<'_> {
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: super::ThreadDriverTask,
    {
        RUST_RADIO_STATE.set_enabled(false);
        RUST_RADIO_STATE.wait_enabled_state(false).await;

        let mpsl = self.mpsl.create_mpsl().await?;

        let radio = self.ieee802154.create_radio(&mpsl)?;

        task.run(radio).await
    }
}

impl super::BleDriver for NrfMpslThreadDriver<'_> {
    async fn run<T>(&mut self, mut task: T) -> Result<(), Error>
    where
        T: super::BleDriverTask,
    {
        RUST_RADIO_STATE.set_enabled(false);
        RUST_RADIO_STATE.wait_enabled_state(false).await;

        let mpsl = self.mpsl.create_mpsl().await?;

        // TODO: Externalize as resources
        // Mem is roughly MAC_CONNECTIONS * MAX_MTU_SIZE * L2CAP_TXQ * L2CAP_RXQ
        let mut sdc_mem = nrf_sdc::Mem::<3084>::new();

        let mut rng = MatterRngCore::new(self.rand);

        let controller = self.sdc.create_controller(&mpsl, &mut sdc_mem, &mut rng)?;

        task.run(controller).await
    }
}

impl super::ThreadCoexDriver for NrfMpslThreadDriver<'_> {
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: super::ThreadCoexDriverTask,
    {
        RUST_RADIO_STATE.set_enabled(false);
        RUST_RADIO_STATE.wait_enabled_state(false).await;

        let mpsl = self.mpsl.create_mpsl().await?;

        // TODO: Externalize as resources
        // Mem is roughly MAC_CONNECTIONS * MAX_MTU_SIZE * L2CAP_TXQ * L2CAP_RXQ
        let mut sdc_mem = nrf_sdc::Mem::<3084>::new();

        let mut rng = MatterRngCore::new(self.rand);

        let controller = self.sdc.create_controller(&mpsl, &mut sdc_mem, &mut rng)?;

        let radio = self.ieee802154.create_radio(&mpsl)?;

        task.run(radio, controller).await
    }
}

fn to_matter_err<E: core::fmt::Debug>(e: E) -> Error {
    error!("BLE error: {:?}", debug2format!(e));
    Error::new(ErrorCode::BtpError)
}

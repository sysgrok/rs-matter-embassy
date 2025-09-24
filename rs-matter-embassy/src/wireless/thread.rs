use core::pin::pin;

use embassy_futures::select::select3;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};

use openthread::{OpenThread, Radio};

use crate::ble::{ControllerRef, TroubleBtpGattContext, TroubleBtpGattPeripheral};
use crate::matter::dm::networks::wireless::Thread;
use crate::matter::error::Error;
use crate::matter::utils::init::{init, Init};
use crate::matter::utils::rand::Rand;
use crate::matter::utils::select::Coalesce;
use crate::matter::utils::sync::IfMutex;
use crate::ot::{to_matter_err, OtNetCtl, OtNetStack, OtPersist};
use crate::ot::{OtMatterResources, OtMdns, OtNetif};
use crate::stack::network::{Embedding, Network};
use crate::stack::persist::{KvBlobStore, SharedKvBlobStore};
use crate::stack::rand::MatterRngCore;
use crate::stack::wireless::{self, Gatt, GattTask};

use super::{BleDriver, BleDriverTask, BleDriverTaskImpl, EmbassyWirelessMatterStack};

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
pub struct EmbassyThread<'a, T, S> {
    driver: T,
    ieee_eui64: [u8; 8],
    store: &'a SharedKvBlobStore<'a, S>,
    context: &'a OtNetContext,
    ble_context: &'a TroubleBtpGattContext<CriticalSectionRawMutex>,
    rand: Rand,
}

impl<'a, T, S> EmbassyThread<'a, T, S>
where
    T: ThreadDriver,
    S: KvBlobStore,
{
    /// Create a new instance of the `EmbassyThread` type.
    pub fn new<const B: usize, E>(
        driver: T,
        ieee_eui64: [u8; 8],
        store: &'a SharedKvBlobStore<'a, S>,
        stack: &'a EmbassyThreadMatterStack<'a, B, E>,
    ) -> Self
    where
        E: Embedding + 'static,
    {
        Self::wrap(
            driver,
            ieee_eui64,
            store,
            stack.network().embedding().net_context(),
            stack.network().embedding().ble_context(),
            stack.matter().rand(),
        )
    }

    /// Wrap an existing `ThreadDriver` with the given parameters.
    pub fn wrap(
        driver: T,
        ieee_eui64: [u8; 8],
        store: &'a SharedKvBlobStore<'a, S>,
        context: &'a OtNetContext,
        ble_context: &'a TroubleBtpGattContext<CriticalSectionRawMutex>,
        rand: Rand,
    ) -> Self {
        Self {
            driver,
            ieee_eui64,
            store,
            context,
            ble_context,
            rand,
        }
    }
}

impl<T, S> wireless::Thread for EmbassyThread<'_, T, S>
where
    T: ThreadDriver,
    S: KvBlobStore,
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
            })
            .await
    }
}

impl<T, S> wireless::ThreadCoex for EmbassyThread<'_, T, S>
where
    T: ThreadCoexDriver,
    S: KvBlobStore,
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
                task,
            })
            .await
    }
}

impl<T, S> Gatt for EmbassyThread<'_, T, S>
where
    T: BleDriver,
    S: KvBlobStore,
{
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: GattTask,
    {
        self.driver
            .run(BleDriverTaskImpl {
                task,
                rand: self.rand,
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

#[cfg(feature = "esp")]
pub mod esp_thread {
    use bt_hci::controller::ExternalController;

    use esp_radio::ble::controller::BleConnector;
    use openthread::esp::EspRadio;

    use rs_matter_stack::matter::error::Error;

    use crate::wireless::SLOTS;

    /// A `ThreadRadio` implementation for the ESP32 family of chips.
    pub struct EspThreadDriver<'a, 'd> {
        controller: &'a esp_radio::Controller<'d>,
        radio_peripheral: esp_hal::peripherals::IEEE802154<'d>,
        bt_peripheral: esp_hal::peripherals::BT<'d>,
    }

    impl<'a, 'd> EspThreadDriver<'a, 'd> {
        /// Create a new instance of the `EspThreadRadio` type.
        ///
        /// # Arguments
        /// - `peripheral` - The Thread radio peripheral instance.
        pub fn new(
            controller: &'a esp_radio::Controller<'d>,
            radio_peripheral: esp_hal::peripherals::IEEE802154<'d>,
            bt_peripheral: esp_hal::peripherals::BT<'d>,
        ) -> Self {
            Self {
                controller,
                radio_peripheral,
                bt_peripheral,
            }
        }
    }

    impl super::ThreadDriver for EspThreadDriver<'_, '_> {
        async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
        where
            A: super::ThreadDriverTask,
        {
            let radio = EspRadio::new(openthread::esp::Ieee802154::new(
                self.radio_peripheral.reborrow(),
            ));

            task.run(radio).await
        }
    }

    impl super::ThreadCoexDriver for EspThreadDriver<'_, '_> {
        async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
        where
            A: super::ThreadCoexDriverTask,
        {
            let ble_controller = ExternalController::<_, SLOTS>::new(BleConnector::new(
                self.controller,
                self.bt_peripheral.reborrow(),
            ));

            let radio = EspRadio::new(openthread::esp::Ieee802154::new(
                self.radio_peripheral.reborrow(),
            ));

            task.run(radio, ble_controller).await
        }
    }

    impl super::BleDriver for EspThreadDriver<'_, '_> {
        async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
        where
            A: super::BleDriverTask,
        {
            let ble_controller = ExternalController::<_, SLOTS>::new(BleConnector::new(
                self.controller,
                self.bt_peripheral.reborrow(),
            ));

            task.run(ble_controller).await
        }
    }
}

#[cfg(feature = "nrf")]
pub mod nrf {
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

    use nrf_sdc::mpsl::{
        ClockInterruptHandler as NrfBleClockInterruptHandler,
        HighPrioInterruptHandler as NrfBleHighPrioInterruptHandler,
        LowPrioInterruptHandler as NrfBleLowPrioInterruptHandler,
    };

    use openthread::nrf::Ieee802154Peripheral;
    use openthread::nrf::NrfRadio;
    pub use openthread::ProxyRadioResources as NrfThreadRadioResources;
    use openthread::{EmbassyTimeTimer, PhyRadioRunner, ProxyRadio, ProxyRadioResources};

    use portable_atomic::{AtomicBool, Ordering};

    use rs_matter_stack::matter::error::{Error, ErrorCode};
    use rs_matter_stack::matter::utils::rand::Rand;
    use rs_matter_stack::matter::utils::sync::Signal;
    use rs_matter_stack::rand::MatterRngCore;

    use crate::ble::MAX_MTU_SIZE;

    /// Bind `RADIO`, `TIMER0` and `RTC0` to this interrupt handler
    pub struct NrfThreadHighPrioInterruptHandler;
    impl Handler<interrupt::typelevel::RADIO> for NrfThreadHighPrioInterruptHandler {
        unsafe fn on_interrupt() {
            if NRF_THREAD_RADIO_STATE.irq_enabled() {
                // Call the IEEE 802.15.4 driver interrupt handler, if the driver is enabled
                InterruptHandler::<embassy_nrf::peripherals::RADIO>::on_interrupt();
            } else {
                <NrfBleHighPrioInterruptHandler as Handler::<interrupt::typelevel::RADIO>>::on_interrupt();
            }
        }
    }
    impl Handler<interrupt::typelevel::TIMER0> for NrfThreadHighPrioInterruptHandler {
        unsafe fn on_interrupt() {
            if !NRF_THREAD_RADIO_STATE.irq_enabled() {
                <NrfBleHighPrioInterruptHandler as Handler::<interrupt::typelevel::TIMER0>>::on_interrupt();
            }
        }
    }
    impl Handler<interrupt::typelevel::RTC0> for NrfThreadHighPrioInterruptHandler {
        unsafe fn on_interrupt() {
            if !NRF_THREAD_RADIO_STATE.irq_enabled() {
                <NrfBleHighPrioInterruptHandler as Handler::<interrupt::typelevel::RTC0>>::on_interrupt();
            }
        }
    }

    /// Bind `EGU0_SWI0` to this interrupt handler
    pub struct NrfThreadLowPrioInterruptHandler;
    impl<T: Interrupt> Handler<T> for NrfThreadLowPrioInterruptHandler {
        unsafe fn on_interrupt() {
            if !NRF_THREAD_RADIO_STATE.irq_enabled() {
                <NrfBleLowPrioInterruptHandler as Handler<T>>::on_interrupt();
            }
        }
    }

    /// Bind `CLOCK_POWER` to this interrupt handler
    pub struct NrfThreadClockInterruptHandler;
    impl Handler<interrupt::typelevel::CLOCK_POWER> for NrfThreadClockInterruptHandler {
        unsafe fn on_interrupt() {
            if !NRF_THREAD_RADIO_STATE.irq_enabled() {
                <NrfBleClockInterruptHandler as Handler::<interrupt::typelevel::CLOCK_POWER>>::on_interrupt();
            }
        }
    }

    struct NrfThreadRadioState {
        irq_enabled: AtomicBool,
        enable: Signal<CriticalSectionRawMutex, bool>,
        state: Signal<CriticalSectionRawMutex, bool>,
    }

    impl NrfThreadRadioState {
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

    static NRF_THREAD_RADIO_STATE: NrfThreadRadioState = NrfThreadRadioState::new();

    struct NrfThreadRadioInterrupts;

    unsafe impl Binding<<RADIO as Ieee802154Peripheral>::Interrupt, InterruptHandler<RADIO>>
        for NrfThreadRadioInterrupts
    {
    }

    /// A runner for the NRF52 PHY radio
    /// Needs to run in a high-prio execution context
    pub struct NrfThreadRadioRunner<'a, 'd> {
        runner: PhyRadioRunner<'a>,
        radio_peripheral: Peri<'d, RADIO>,
    }

    impl<'a, 'd> NrfThreadRadioRunner<'a, 'd> {
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
                NRF_THREAD_RADIO_STATE.wait_enabled(true).await;

                {
                    NRF_THREAD_RADIO_STATE.set_enabled_state(true);

                    info!("Thread radio started");

                    let radio = NrfRadio::new(embassy_nrf::radio::ieee802154::Radio::new(
                        self.radio_peripheral.reborrow(),
                        NrfThreadRadioInterrupts,
                    ));

                    let mut cmd = pin!(NRF_THREAD_RADIO_STATE.wait_enabled(false));
                    let mut runner = pin!(self.runner.run(radio, EmbassyTimeTimer));

                    select(&mut cmd, &mut runner).await;

                    NRF_THREAD_RADIO_STATE.set_enabled_state(false);

                    info!("Thread radio stopped");
                }
            }
        }
    }

    /// A `ThreadDriver` implementation for the NRF52 family of chips.
    pub struct NrfThreadDriver<'d> {
        proxy: ProxyRadio<'d>,
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
    }

    impl<'d> NrfThreadDriver<'d> {
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
            _irqs: I,
        ) -> (Self, NrfThreadRadioRunner<'d, 'd>)
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

            let runner = NrfThreadRadioRunner::new(proxy_runner, radio);

            (
                Self {
                    proxy,
                    rtc0,
                    timer0,
                    temp,
                    ppi_ch17,
                    ppi_ch18,
                    ppi_ch19,
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
                    ppi_ch30,
                    ppi_ch31,
                    rand,
                },
                runner,
            )
        }
    }

    impl super::ThreadDriver for NrfThreadDriver<'_> {
        async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
        where
            A: super::ThreadDriverTask,
        {
            info!("About to enable Thread radio");

            let _guard = scopeguard::guard((), |_| NRF_THREAD_RADIO_STATE.set_enabled(false));

            NRF_THREAD_RADIO_STATE.set_enabled(true);
            NRF_THREAD_RADIO_STATE.wait_enabled_state(true).await;

            info!("Running Thread radio task");

            task.run(&mut self.proxy).await
        }
    }

    impl super::BleDriver for NrfThreadDriver<'_> {
        async fn run<T>(&mut self, mut task: T) -> Result<(), Error>
        where
            T: super::BleDriverTask,
        {
            NRF_THREAD_RADIO_STATE.set_enabled(false);
            NRF_THREAD_RADIO_STATE.wait_enabled_state(false).await;

            let mpsl_p = nrf_sdc::mpsl::Peripherals::new(
                self.rtc0.reborrow(),
                self.timer0.reborrow(),
                self.temp.reborrow(),
                self.ppi_ch19.reborrow(),
                self.ppi_ch30.reborrow(),
                self.ppi_ch31.reborrow(),
            );

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

            let lfclk_cfg = nrf_sdc::mpsl::raw::mpsl_clock_lfclk_cfg_t {
                source: nrf_sdc::mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
                rc_ctiv: nrf_sdc::mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
                rc_temp_ctiv: nrf_sdc::mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
                accuracy_ppm: nrf_sdc::mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
                skip_wait_lfclk_started: nrf_sdc::mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED
                    != 0,
            };

            let mpsl = nrf_sdc::mpsl::MultiprotocolServiceLayer::new::<
                interrupt::typelevel::EGU0_SWI0,
                _,
            >(mpsl_p, NrfBleControllerInterrupts, lfclk_cfg)
            .map_err(to_matter_err)?;

            // TODO: Externalize as resources
            // Mem is roughly MAC_CONNECTIONS * MAX_MTU_SIZE * L2CAP_TXQ * L2CAP_RXQ
            let mut sdc_mem = nrf_sdc::Mem::<3084>::new();

            let mut rng = MatterRngCore::new(self.rand);

            let controller = nrf_sdc::Builder::new()
                .map_err(to_matter_err)?
                .support_adv()
                .map_err(to_matter_err)?
                .support_peripheral()
                .map_err(to_matter_err)?
                .peripheral_count(1)
                .map_err(to_matter_err)?
                .buffer_cfg(MAX_MTU_SIZE as _, MAX_MTU_SIZE as _, L2CAP_TXQ, L2CAP_RXQ)
                .map_err(to_matter_err)?
                .build(sdc_p, &mut rng, &mpsl, &mut sdc_mem)
                .map_err(to_matter_err)?;

            task.run(controller).await
        }
    }

    /// How many outgoing L2CAP buffers per link
    const L2CAP_TXQ: u8 = 3;
    /// How many incoming L2CAP buffers per link
    const L2CAP_RXQ: u8 = 3;

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

    fn to_matter_err<E: core::fmt::Debug>(e: E) -> Error {
        error!("BLE error: {:?}", debug2format!(e));
        Error::new(ErrorCode::BtpError)
    }
}

struct ThreadDriverTaskImpl<'a, A, S> {
    ieee_eui64: [u8; 8],
    rand: Rand,
    store: &'a SharedKvBlobStore<'a, S>,
    context: &'a OtNetContext,
    task: A,
}

impl<A, S> ThreadDriverTask for ThreadDriverTaskImpl<'_, A, S>
where
    A: wireless::ThreadTask,
    S: KvBlobStore,
{
    async fn run<R>(&mut self, radio: R) -> Result<(), Error>
    where
        R: Radio,
    {
        let mut rng = MatterRngCore::new(self.rand);
        let mut resources = self.context.resources.lock().await;
        let resources = &mut *resources;

        let persister = OtPersist::new(&mut resources.settings_buf, self.store);
        persister.load().await?;

        let mut settings = persister.settings();

        let ot = OpenThread::new_with_udp_srp(
            self.ieee_eui64,
            &mut rng,
            &mut settings,
            &mut resources.ot,
            &mut resources.udp,
            &mut resources.srp,
        )
        .map_err(to_matter_err)?;

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

        let result = select3(&mut main, &mut radio, &mut persist)
            .coalesce()
            .await;

        let _ = ot.enable_thread(false);
        let _ = ot.srp_stop();
        let _ = ot.enable_ipv6(false);

        result
    }
}

struct ThreadCoexDriverTaskImpl<'a, A, S> {
    ieee_eui64: [u8; 8],
    rand: Rand,
    store: &'a SharedKvBlobStore<'a, S>,
    context: &'a OtNetContext,
    ble_context: &'a TroubleBtpGattContext<CriticalSectionRawMutex>,
    task: A,
}

impl<A, S> ThreadCoexDriverTask for ThreadCoexDriverTaskImpl<'_, A, S>
where
    A: wireless::ThreadCoexTask,
    S: KvBlobStore,
{
    async fn run<R, B>(&mut self, radio: R, ble_ctl: B) -> Result<(), Error>
    where
        R: Radio,
        B: trouble_host::Controller,
    {
        let mut rng = MatterRngCore::new(self.rand);
        let mut resources = self.context.resources.lock().await;
        let resources = &mut *resources;

        let persister = OtPersist::new(&mut resources.settings_buf, self.store);
        persister.load().await?;

        let mut settings = persister.settings();

        let ot = OpenThread::new_with_udp_srp(
            self.ieee_eui64,
            &mut rng,
            &mut settings,
            &mut resources.ot,
            &mut resources.udp,
            &mut resources.srp,
        )
        .map_err(to_matter_err)?;

        let net_ctl = OtNetCtl::new(ot.clone());
        let net_stack = OtNetStack::new(ot.clone());
        let netif = OtNetif::new(ot.clone());
        let mut mdns = OtMdns::new(ot.clone());
        let mut peripheral = TroubleBtpGattPeripheral::new(ble_ctl, self.rand, self.ble_context);

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

        let result = select3(&mut main, &mut radio, &mut persist)
            .coalesce()
            .await;

        let _ = ot.enable_thread(false);
        let _ = ot.srp_stop();
        let _ = ot.enable_ipv6(false);

        result
    }
}

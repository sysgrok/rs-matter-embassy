//! An example of a Short Idle Time (SIT) Intermittently Connected Device (ICD) over Thread on
//! the nRF52840 / nRF54L15: a battery-powered radiator valve (a heating-only Thermostat).
//!
//! The nRF counterpart of the ESP32-C6 `trv_battery_thread` example - see it for what a SIT
//! ICD is and why an actuator is the right showcase for it. This one shows the golden case
//! for a SIT device: **reachable within a second, on a battery**. Two things make that
//! possible here:
//!
//! - **The radio listens rather than polls** (the `csl` feature, on by default).
//!   `nrf-802154` supports timed receive, so the Thread driver maps the ICD intervals onto a
//!   CSL (Coordinated Sampled Listening) period: the node announces a listening schedule and
//!   its Thread 1.2+ parent transmits into its short receive windows; the node itself only
//!   transmits now and then, to stay synchronized. Without `csl` the node polls its parent at
//!   the same intervals instead: a data request and its ACK each time.
//!   At a one-second interval and 0 dBm the two cost about the same - a poll's short
//!   transmission against a receive window that OpenThread widens with the clock drift since
//!   the last synchronization, and both wake the CPU and the radio's crystal alike. CSL pulls
//!   ahead with a higher transmit power, a shorter interval or a busy network (polls are traffic
//!   on the air for every sleepy node); polling does not depend on the parent's CSL support at
//!   all. Against a Thread 1.1 parent OpenThread falls back to polling on its own.
//! - **The MCU really sleeps.** `embassy-executor` parks the core with `WFE` whenever nothing
//!   is runnable, and on the nRF that is System ON idle with the RAM retained, a few µA. With
//!   the radio off between samples nothing else keeps the chip awake, so the device idles at
//!   its sleep floor with no sleep code at all in this example.
//!
//! Concurrent commissioning (BLE and Thread at the same time) is used; the BLE controller
//! only advertises while a commissioning window is open.
//!
//! For simplicity this example keeps the Matter state in RAM (`DummyKvBlobStore`), so a
//! power cycle means re-commissioning; the ESP example shows a flash-backed store.
//!
//! To measure the power draw, the debug probe has to be detached and the chip power-cycled:
//! an attached debugger keeps the chip's debug power domain on. Build with the `log-uart`
//! feature to still get the logs, and with `xiao` on a Seeed XIAO board (see `Cargo.toml`).
#![no_std]
#![no_main]
#![recursion_limit = "256"]

use core::mem::MaybeUninit;
use core::pin::pin;
use core::ptr::addr_of_mut;

use embassy_nrf::bind_interrupts;

use embassy_executor::Spawner;

use embedded_alloc::LlffHeap;

use defmt::{info, unwrap};

use rs_matter_embassy::matter::crypto::{default_crypto, Crypto, Rng};
use rs_matter_embassy::matter::dm::clusters::app::thermostat::{
    self, ControlSequenceOfOperationEnum, RelayStateBitmap, SystemModeEnum, ThermostatHooks,
};
use rs_matter_embassy::matter::dm::clusters::basic_info::BasicInfoConfig;
use rs_matter_embassy::matter::dm::clusters::decl::thermostat as thermostat_cluster;
use rs_matter_embassy::matter::dm::clusters::desc::{self, ClusterHandler as _};
use rs_matter_embassy::matter::dm::clusters::icd_mgmt::{
    ClusterHandler as _, Icd, IcdModeConfig, SitIcdMgmtHandler,
};
use rs_matter_embassy::matter::dm::devices::test::{
    DAC_PRIVKEY, TEST_DEV_ATT, TEST_DEV_COMM, TEST_DEV_DET,
};
use rs_matter_embassy::matter::dm::devices::{DEV_TYPE_ROOT_NODE, DEV_TYPE_THERMOSTAT};
use rs_matter_embassy::matter::dm::endpoints::ROOT_ENDPOINT_ID;
use rs_matter_embassy::matter::dm::{Async, Cluster, Dataver, EmptyHandler, Endpoint, Node};
use rs_matter_embassy::matter::persist::{DummyKvBlobStore, VENDOR_KEYS_START};
use rs_matter_embassy::matter::utils::cell::RefCell;
use rs_matter_embassy::matter::utils::init::InitMaybeUninit;
use rs_matter_embassy::matter::utils::sync::blocking::Mutex;
use rs_matter_embassy::matter::{clusters, devices, with, BasicCommData};
use rs_matter_embassy::stack::rand::reseeding_csprng;
#[cfg(feature = "nrf54l15")]
use rs_matter_embassy::wireless::nrf::CcmInterruptHandler;
#[cfg(any(feature = "nrf52840", feature = "xiao"))]
use rs_matter_embassy::wireless::nrf::NrfLfClock;
use rs_matter_embassy::wireless::nrf::{
    EguInterruptHandler, LpTimerInterruptHandler, NrfIeee802154Peripherals, NrfMpslPeripherals,
    NrfSdcPeripherals, NrfThreadClockInterruptHandler, NrfThreadHighPrioInterruptHandler,
    NrfThreadLowPrioInterruptHandler, NrfThreadMpslRadioDriver,
};
use rs_matter_embassy::wireless::{EmbassyThread, EmbassyThreadMatterStack};

use panic_rtt_target as _;

use tinyrlibc as _;

macro_rules! mk_static {
    ($t:ty) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        STATIC_CELL.uninit()
    }};
    ($t:ty,$val:expr) => {{
        mk_static!($t).write($val)
    }};
}

#[cfg(feature = "nrf52840")]
bind_interrupts!(struct Irqs {
    // MPSL's low-priority handler and the 802.15.4 notifications share EGU0_SWI0
    EGU0_SWI0 => NrfThreadLowPrioInterruptHandler, EguInterruptHandler;
    // MPSL clock handler
    CLOCK_POWER => NrfThreadClockInterruptHandler;
    // MPSL high-priority handlers
    RADIO => NrfThreadHighPrioInterruptHandler;
    TIMER0 => NrfThreadHighPrioInterruptHandler;
    RTC0 => NrfThreadHighPrioInterruptHandler;
    // 802.15.4 LP timer
    RTC2 => LpTimerInterruptHandler;
});

#[cfg(all(feature = "log-rtt", feature = "log-uart"))]
compile_error!("`log-uart` replaces `log-rtt`: build it with `--no-default-features`");

#[cfg(not(any(feature = "log-rtt", feature = "log-uart")))]
compile_error!("Logging needs either the `log-rtt` or the `log-uart` feature");

#[cfg(feature = "nrf54l15")]
bind_interrupts!(struct Irqs {
    // MPSL low-priority handler. Unlike on nRF52, nothing is shared with the
    // 802.15.4 driver here: it has its own EGU instance (EGU10).
    SWI00 => NrfThreadLowPrioInterruptHandler;
    // MPSL clock handler
    CLOCK_POWER => NrfThreadClockInterruptHandler;
    // MPSL high-priority handlers
    RADIO_0 => NrfThreadHighPrioInterruptHandler;
    TIMER10 => NrfThreadHighPrioInterruptHandler;
    GRTC_3 => NrfThreadHighPrioInterruptHandler;
    // 802.15.4 notifications
    EGU10 => EguInterruptHandler;
    // 802.15.4 LP timer (GRTC interrupt group 0)
    GRTC_0 => LpTimerInterruptHandler;
    // 802.15.4 frame encryption offload
    AAR00_CCM00 => CcmInterruptHandler;
});

/// The ICD mode timings this device advertises.
///
/// A SIT device: it stays active for a second after boot, and for a second after any Matter
/// message, so that a multi-message exchange does not fall back to the slow listening period
/// halfway through. How quickly the device can be reached is the `SII` in `TEST_BASIC_INFO`
/// (1 s).
///
/// `idle_mode_duration_s` is how long the device stays idle before it goes active on its own; a
/// SIT stays reachable within its `SII` throughout. Subscriptions to the device get it as their
/// max interval, their keep-alive reports go out a few seconds before it runs out, and idle mode
/// ends by then at the latest: with three minutes, the device wakes up on its own about every
/// three minutes, keep-alives included. Each such wake-up is a short exchange with the subscriber
/// that costs about as much as a quarter of a minute of idling. A longer idle mode saves less and
/// less, while a subscriber also takes about that long to notice a device gone silent (e.g. with
/// a flat battery) and to show it as offline.
const ICD_MODE: IcdModeConfig = IcdModeConfig {
    idle_mode_duration_s: 180,
    active_mode_duration_ms: 1000,
    active_mode_threshold_ms: 1000,
    user_active_mode_trigger_hint: 0,
    user_active_mode_trigger_instruction: "",
};

const BUMP_SIZE: usize = 24000;

#[global_allocator]
static HEAP: LlffHeap = LlffHeap::empty();

/// We need a bigger log ring-buffer or else the device QR code printout is half-lost
#[cfg(feature = "log-rtt")]
const LOG_RINGBUF_SIZE: usize = 2048;

#[embassy_executor::main]
async fn main(_s: Spawner) {
    // `rs-matter` uses the `x509` crate which (still) needs a few kilos of heap space
    {
        #[cfg(not(feature = "nimble"))]
        const HEAP_SIZE: usize = 8192;
        // NimBLE allocates its mbuf/transport pools (~13K with the stock counts) and its GATT
        // registry from this same heap.
        #[cfg(feature = "nimble")]
        const HEAP_SIZE: usize = 8192 + 16384;

        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(addr_of_mut!(HEAP_MEM) as usize, HEAP_SIZE) }
    }

    // Necessary `nrf-hal` initialization boilerplate

    #[cfg(feature = "log-rtt")]
    rtt_target::rtt_init_defmt!(rtt_target::ChannelMode::NoBlockSkip, LOG_RINGBUF_SIZE);

    // Panic messages do not go through defmt, so they stay on RTT, for a probe attached after
    // the fact.
    #[cfg(feature = "log-uart")]
    rtt_target::rtt_init_print!();

    info!("Starting...");

    let p = init();

    #[cfg(all(feature = "log-uart", feature = "nrf52840"))]
    _s.spawn(unwrap!(log_uart::run(p.UARTE0, p.P1_11)));
    #[cfg(all(feature = "log-uart", feature = "nrf54l15"))]
    _s.spawn(unwrap!(log_uart::run(p.SERIAL20, p.P1_11)));

    #[cfg(all(feature = "xiao", feature = "nrf52840"))]
    xiao::flash_deep_power_down(p.P0_25, p.P0_21, p.P0_20, p.P0_22, p.P0_23);
    #[cfg(all(feature = "xiao", feature = "nrf54l15"))]
    xiao::rf_switch_onboard_antenna(p.P2_03, p.P2_05);
    #[cfg(all(feature = "xiao", feature = "nrf54l15"))]
    xiao::crystal_load_capacitors();

    // The nRF54L has no standalone `RNG` peripheral; its entropy comes out of the CRACEN crypto engine instead.
    #[cfg(feature = "nrf52840")]
    let trng = embassy_nrf::rng::Rng::new_blocking(p.RNG);
    #[cfg(feature = "nrf54l15")]
    let trng = embassy_nrf::cracen::Cracen::new_blocking(p.CRACEN);

    // Create the crypto provider, using the NRF hardware TRNG as the source of randomness for a reseeding CSPRNG.
    let crypto = default_crypto(reseeding_csprng(trng, 1000).unwrap(), DAC_PRIVKEY);

    let mut weak_rand = crypto.weak_rand().unwrap();

    // Use a random/unique Matter discriminator for this session,
    // in case there are left-overs from our previous registrations in Thread SRP
    let discriminator = TEST_DEV_COMM.discriminator;
    //let discriminator = (weak_rand.next_u32() & 0xfff) as u16;

    // TODO
    let mut ieee_eui64 = [0; 8];
    weak_rand.fill_bytes(&mut ieee_eui64);

    // Allocate the Matter stack.
    // For MCUs, it is best to allocate it statically, so as to avoid program stack blowups (its memory footprint is ~ 35 to 50KB).
    // It is also (currently) a mandatory requirement when the wireless stack variation is used.
    let stack = mk_static!(EmbassyThreadMatterStack<BUMP_SIZE, ()>).init_with(
        EmbassyThreadMatterStack::init(
            &TEST_BASIC_INFO,
            BasicCommData {
                password: TEST_DEV_COMM.password,
                discriminator,
            },
            &TEST_DEV_ATT,
        ),
    );

    #[cfg(feature = "nrf52840")]
    let (mpsl_p, sdc_p, ieee802154_p) = (
        NrfMpslPeripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31),
        NrfSdcPeripherals::new(
            p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23, p.PPI_CH24,
            p.PPI_CH25, p.PPI_CH26, p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
        ),
        NrfIeee802154Peripherals::new(p.EGU0, p.TIMER2, p.RTC2),
    );

    // The nRF54L set looks nothing like the nRF52 one: the time base is the GRTC
    // rather than an RTC, and the GRTC lives in a different peripheral domain from
    // the radio, so carrying timestamps and hardware-timed radio tasks between the
    // two costs a PPIB channel pair in each direction.
    #[cfg(feature = "nrf54l15")]
    let (mpsl_p, sdc_p, ieee802154_p) = (
        NrfMpslPeripherals::new(
            p.GRTC_CH7,
            p.GRTC_CH8,
            p.GRTC_CH9,
            p.GRTC_CH10,
            p.GRTC_CH11,
            p.TIMER10,
            p.TIMER20,
            p.TEMP,
            p.PPI10_CH0,
            p.PPI20_CH1,
            p.PPIB11_CH0,
            p.PPIB21_CH0,
        ),
        NrfSdcPeripherals::new(
            p.PPI00_CH1,
            p.PPI00_CH3,
            p.PPI10_CH1,
            p.PPI10_CH2,
            p.PPI10_CH3,
            p.PPI10_CH4,
            p.PPI10_CH5,
            p.PPI10_CH6,
            p.PPI10_CH7,
            p.PPI10_CH8,
            p.PPI10_CH9,
            p.PPI10_CH10,
            p.PPI10_CH11,
            p.PPIB00_CH1,
            p.PPIB00_CH2,
            p.PPIB00_CH3,
            p.PPIB10_CH1,
            p.PPIB10_CH2,
            p.PPIB10_CH3,
        ),
        NrfIeee802154Peripherals::new(
            p.GRTC_CH3,
            p.GRTC_CH4,
            p.GRTC_CH5,
            p.PPI20_CH2,
            p.PPI20_CH3,
            p.PPIB11_CH1,
            p.PPIB21_CH1,
            p.PPIB11_CH2,
            p.PPIB21_CH2,
        ),
    );

    // The ICD power mode state machine. Backs the ICD Management cluster handler, and is
    // followed by the Thread driver. A SIT device has no registrations and no Check-In counter.
    let icd: &'static Icd = mk_static!(Icd).init_with(Icd::init(ICD_MODE));

    let thread_driver = NrfThreadMpslRadioDriver::new(
        p.RADIO,
        mpsl_p,
        sdc_p,
        ieee802154_p,
        crypto.rand().unwrap(),
        Irqs,
    );

    // The XIAO nRF52840 (like the nRF52840 DK) and the XIAO nRF54L15 have a 32.768 kHz crystal:
    // the CSL receive windows are sized by the low-frequency clock's accuracy, and the RC
    // oscillator's ±250 ppm would widen them (and wake the device for its recalibrations on
    // top). The nRF54L15 loads the crystal with its internal capacitors, which only the `xiao`
    // feature sets up for the board.
    #[cfg(any(feature = "nrf52840", feature = "xiao"))]
    let thread_driver = thread_driver.with_lf_clock(NrfLfClock::Xtal { accuracy_ppm: 20 });

    // The XIAO nRF54L15's RF switch draws ~90 µA when powered: power it only for the radio.
    #[cfg(all(feature = "xiao", feature = "nrf54l15"))]
    let thread_driver = thread_driver.with_radio_activity_hook(xiao::rf_switch_power, 200);

    // The radiator valve: a heating-only Thermostat. The handler owns and persists the
    // setpoints and the mode; the logic below is the (simulated) valve itself.
    let valve = RadiatorValve::new();
    let thermostat = thermostat::ThermostatHandler::new(
        Dataver::new_rand(&mut weak_rand),
        VALVE_ENDPOINT_ID,
        VENDOR_KEYS_START + 0x10,
        &valve,
    );

    // Chain our endpoint clusters
    let handler = EmptyHandler
        // The Endpoint 0 system clusters that are ours to provide.
        // The stack adds the operational network clusters (Network Commissioning,
        // General Commissioning, General Diagnostics and Wifi/Thread/Ethernet
        // Diagnostics) on top, because only it knows the network driver state.
        // Chain any extra Endpoint 0 clusters of your own the same way.
        .chain(
            |e, _| e == ROOT_ENDPOINT_ID,
            Async(EmbassyThreadMatterStack::<0, ()>::root_handler(
                &(),
                &mut weak_rand,
            )),
        )
        // The ICD Management cluster of a SIT device, on Endpoint 0 as well: no features, just
        // the mode timings. Its `run` hook drives the ICD power mode state machine.
        .chain(
            |e, c| e == ROOT_ENDPOINT_ID && c == ICD_MGMT_CLUSTER.id,
            Async(SitIcdMgmtHandler::new(Dataver::new_rand(&mut weak_rand), icd).adapt()),
        )
        // Our thermostat cluster, on Endpoint 1
        .chain(
            |e, c| e == VALVE_ENDPOINT_ID && c == RadiatorValve::CLUSTER.id,
            Async(thermostat::HandlerAdaptor(&thermostat)),
        )
        // Each Endpoint needs a Descriptor cluster too
        // Just use the one that `rs-matter` provides out of the box
        .chain(
            |e, c| e == VALVE_ENDPOINT_ID && c == desc::DescHandler::CLUSTER.id,
            Async(desc::DescHandler::new(Dataver::new_rand(&mut weak_rand)).adapt()),
        );

    // Create a KV BLOB store and load any previously saved state of `rs-matter`
    // `SeqMapKvBlobStore` saves to a user-supplied NOR Flash region
    // However, for this demo and for simplicity, we use a dummy KV BLOB store that does nothing
    let mut store = DummyKvBlobStore;
    stack.startup(&crypto, &mut store).await.unwrap();

    let kv = stack.matter().kv(store);

    // Run the Matter stack with our handler
    // Using `pin!` is completely optional, but reduces the size of the final future
    //
    // This step can be repeated in that the stack can be stopped and started multiple times, as needed.
    let matter = pin!(stack.run_coex(
        // The Matter stack needs to instantiate `openthread` - as a Sleepy End Device
        // following the ICD state
        EmbassyThread::new(
            thread_driver,
            crypto.rand().unwrap(),
            ieee_eui64,
            &kv,
            stack,
            true, // Use a random BLE address
        )
        .with_icd(icd),
        // The crypto provider
        &crypto,
        // Our `AsyncHandler` + `AsyncMetadata` impl
        (NODE, handler),
        // The Matter stack needs a blob store to store its state
        &kv,
        // No user future to run
        (),
    ));

    // Run Matter
    unwrap!(matter.await);
}

/// Basic info about our device.
///
/// The `SAI` / `SII` (session active / idle intervals) are what the device advertises to
/// controllers *and* the CSL periods (or poll periods, without `csl`) of the Thread node: it
/// listens every 300 ms while the ICD is active and every second while idle. The idle interval
/// is what a controller waits for, so it is the reaction time of the valve - and the main knob
/// for the battery, as the radio work once per interval is the bulk of the idle current. The
/// controllers also derive their retransmission timing from these, so they must be what the
/// radio really does.
const TEST_BASIC_INFO: BasicInfoConfig = BasicInfoConfig {
    sai: Some(300),
    sii: Some(1000),
    ..TEST_DEV_DET
};

/// Endpoint 0 (the root endpoint) always runs
/// the hidden Matter system clusters, so we pick ID=1
const VALVE_ENDPOINT_ID: u16 = 1;

/// The simulated radiator valve: the thermostat logic behind the Thermostat cluster handler.
///
/// A real one would read its temperature sensor and drive the valve motor from `apply`; this one
/// keeps a fixed room temperature and just remembers what the handler asked for.
struct RadiatorValve {
    state: Mutex<RefCell<ValveState>>,
}

struct ValveState {
    system_mode: SystemModeEnum,
    heating_setpoint: i16,
    local_temperature: i16,
}

impl RadiatorValve {
    const fn new() -> Self {
        Self {
            state: Mutex::new(RefCell::new(ValveState {
                system_mode: SystemModeEnum::Off,
                heating_setpoint: Self::OCCUPIED_HEATING_SETPOINT,
                local_temperature: 1950,
            })),
        }
    }

    /// Whether the valve is open: heating, and the room below the setpoint.
    fn heating(&self) -> bool {
        self.state.lock(|state| {
            let state = state.borrow();

            state.system_mode == SystemModeEnum::Heat
                && state.local_temperature < state.heating_setpoint
        })
    }
}

impl ThermostatHooks for RadiatorValve {
    /// A heating-only thermostat: the `HEAT` feature alone, the mandatory attributes plus the
    /// heat setpoint limits, and the one mandatory command.
    const CLUSTER: Cluster<'static> = thermostat_cluster::FULL_CLUSTER
        .with_revision(11)
        .with_features(thermostat_cluster::Feature::HEATING.bits())
        .with_attrs(with!(
            required;
            thermostat_cluster::AttributeId::AbsMinHeatSetpointLimit
                | thermostat_cluster::AttributeId::AbsMaxHeatSetpointLimit
                | thermostat_cluster::AttributeId::OccupiedHeatingSetpoint
                | thermostat_cluster::AttributeId::MinHeatSetpointLimit
                | thermostat_cluster::AttributeId::MaxHeatSetpointLimit
                | thermostat_cluster::AttributeId::ThermostatRunningState
        ))
        .with_cmds(with!(thermostat_cluster::CommandId::SetpointRaiseLower))
        .with_events(with!());

    const CONTROL_SEQUENCE_OF_OPERATION: ControlSequenceOfOperationEnum =
        ControlSequenceOfOperationEnum::HeatingOnly;

    /// 20.00 °C to begin with.
    const OCCUPIED_HEATING_SETPOINT: i16 = 2000;

    fn local_temperature(&self) -> Option<i16> {
        Some(self.state.lock(|state| state.borrow().local_temperature))
    }

    fn running_state(&self) -> RelayStateBitmap {
        if self.heating() {
            RelayStateBitmap::HEAT
        } else {
            RelayStateBitmap::empty()
        }
    }

    /// Called whenever a controller (or the `SetpointRaiseLower` command) changes the mode or
    /// a setpoint - the moment a real valve would start its motor.
    fn apply(&self, system_mode: SystemModeEnum, heating_setpoint: i16, _cooling_setpoint: i16) {
        self.state.lock(|state| {
            let mut state = state.borrow_mut();

            state.system_mode = system_mode;
            state.heating_setpoint = heating_setpoint;
        });

        info!(
            "Valve: mode {}, setpoint {}.{:02} C, {}",
            system_mode as u8,
            heating_setpoint / 100,
            (heating_setpoint % 100).abs(),
            if self.heating() { "open" } else { "closed" }
        );
    }
}

/// The ICD Management cluster metadata, exactly as served by `SitIcdMgmtHandler`: a SIT-only
/// device, which claims no ICD features and advertises no `ICD` DNS-SD TXT key.
const ICD_MGMT_CLUSTER: Cluster<'static> = SitIcdMgmtHandler::CLUSTER;

/// The Matter Thermostat (radiator valve) Node.
///
/// The root endpoint is the stack's usual Thread one plus the ICD Management cluster.
const NODE: Node = Node {
    endpoints: &[
        Endpoint::new(
            ROOT_ENDPOINT_ID,
            devices!(DEV_TYPE_ROOT_NODE),
            clusters!(thread; ICD_MGMT_CLUSTER),
        ),
        Endpoint::new(
            VALVE_ENDPOINT_ID,
            devices!(DEV_TYPE_THERMOSTAT),
            clusters!(desc::DescHandler::CLUSTER, RadiatorValve::CLUSTER),
        ),
    ],
};

/// Initialize `embassy-nrf` with a configuration MPSL and the 802.15.4 driver
/// can live with.
fn init() -> embassy_nrf::Peripherals {
    #[cfg(feature = "nrf54l15")]
    scrub_grtc();

    // The HF crystal is left to MPSL, which runs it only around radio activity: started here,
    // outside of MPSL's bookkeeping, it would never be stopped.
    #[allow(unused_mut)]
    let mut config = embassy_nrf::config::Config::default();

    // The XIAO nRF52840 has the inductor of the chip's DC/DC converter: run the core supply off
    // it rather than the LDO, which roughly halves the current of the radio and the CPU.
    #[cfg(all(feature = "xiao", feature = "nrf52840"))]
    {
        config.dcdc.reg1 = true;
    }

    // On nRF54L, force the 128 MHz PLL: `embassy-nrf` defaults to 64 MHz, but MPSL
    // asserts on anything else at startup, and 128 MHz is the only frequency the
    // 802.15.4 driver supports on this series.
    #[cfg(feature = "nrf54l15")]
    {
        config.clock_speed = embassy_nrf::config::ClockSpeed::CK128;
    }

    embassy_nrf::init(config)
}

/// Drop any GRTC interrupt state inherited from the firmware that ran before us.
///
/// The GRTC is in the always-on domain, so a soft reset - which is what a
/// debugger's `SYSRESETREQ` and therefore `probe-rs run` issues - leaves its
/// compare events latched and its per-domain interrupt enables set. The NVIC
/// *is* reset, so nothing fires until someone re-enables the line; embassy's
/// GRTC time driver does exactly that at the end of `embassy_nrf::init`.
///
/// From there an inherited enable is fatal: embassy's ISR only ever clears its
/// own channel, so a stale event on any other channel of the same domain
/// re-fires the instant the handler returns, and `init` never comes back.
/// Boards that shipped with Zephyr or Arduino hit this on the first flash,
/// because nrfx hands out GRTC channels from CC[0] upwards.
///
/// Cheap insurance, and it has to happen before `embassy_nrf::init`.
#[cfg(feature = "nrf54l15")]
fn scrub_grtc() {
    let r = embassy_nrf::pac::GRTC;

    // Every domain, not just ours: at this point in boot nothing else - not
    // MPSL, not the time driver - has claimed one yet.
    for group in 0..4 {
        r.intenclr(group).write(|w| w.0 = u32::MAX);
    }
    for cc in 0..12 {
        r.events_compare(cc).write_value(0);
    }
}

/// defmt logging over UART rather than RTT.
///
/// The logger only queues the encoded frames; `run` sends them out of a UARTE (TX only, P1.11,
/// 115200 baud), and switches the UARTE off as soon as the queue is empty, so that logging
/// costs nothing while the device sleeps: UARTE0 on the nRF52840 (the XIAO nRF52840's D6),
/// UARTE20 on the nRF54L15 (the XIAO nRF54L15's D5).
///
/// Not on the XIAO nRF54L15's P1.09, which its onboard probe bridges to USB: measuring the
/// power draw means running without USB, and then the unpowered probe gets fed through that
/// pin's input protection and keeps BLE connections from coming up. On the host:
/// `stty -F <port> 115200 raw -echo && defmt-print -e <elf> < <port>`.
#[cfg(feature = "log-uart")]
mod log_uart {
    use core::ptr::{addr_of, addr_of_mut};
    use core::sync::atomic::{AtomicBool, Ordering};

    #[cfg(feature = "nrf54l15")]
    use embassy_nrf::peripherals::{P1_11 as TXD, SERIAL20 as UARTE};
    #[cfg(feature = "nrf52840")]
    use embassy_nrf::peripherals::{P1_11 as TXD, UARTE0 as UARTE};
    use embassy_nrf::uarte::{self, UarteTx};
    use embassy_nrf::{bind_interrupts, Peri};

    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::pipe::Pipe;

    bind_interrupts!(struct Irqs {
        #[cfg(feature = "nrf52840")]
        UARTE0 => uarte::InterruptHandler<UARTE>;
        #[cfg(feature = "nrf54l15")]
        SERIAL20 => uarte::InterruptHandler<UARTE>;
    });

    /// The longest frame the logger queues; a longer one is dropped whole.
    const MAX_FRAME_LEN: usize = 256;

    /// How many bytes of frames can wait to be sent. At 115200 baud the UART drains only ~11 KB/s,
    /// and the startup burst (the QR code, the stack's allocations) is several KB in a fraction of
    /// a second; a frame that does not fit is dropped whole.
    const QUEUE_LEN: usize = 8192;

    /// The frames waiting to be sent.
    static QUEUE: Pipe<CriticalSectionRawMutex, QUEUE_LEN> = Pipe::new();

    /// Send the queued frames out of the UART, for as long as the program runs.
    #[embassy_executor::task]
    pub async fn run(mut uarte: Peri<'static, UARTE>, mut txd: Peri<'static, TXD>) {
        // EasyDMA only reads RAM, so the frames go out through this buffer.
        let mut chunk = [0; 64];

        loop {
            let mut len = QUEUE.read(&mut chunk).await;

            let mut tx = UarteTx::new(
                uarte.reborrow(),
                Irqs,
                txd.reborrow(),
                uarte::Config::default(),
            );

            while len > 0 {
                let _ = tx.write(&chunk[..len]).await;

                len = QUEUE.try_read(&mut chunk).unwrap_or(0);
            }

            // Dropping `tx` disables the UARTE until the next frame.
        }
    }

    #[defmt::global_logger]
    struct Logger;

    static TAKEN: AtomicBool = AtomicBool::new(false);
    static mut CS_RESTORE: critical_section::RestoreState =
        critical_section::RestoreState::invalid();
    static mut ENCODER: defmt::Encoder = defmt::Encoder::new();
    static mut FRAME: Frame = Frame::new();

    // The critical section is the `critical-section` crate's rather than masking all
    // interrupts: MPSL's implementation leaves its own high-priority interrupts unmasked,
    // as it must.
    unsafe impl defmt::Logger for Logger {
        fn acquire() {
            // Safety: paired with the `release` in `release`.
            let restore = unsafe { critical_section::acquire() };

            if TAKEN.load(Ordering::Relaxed) {
                panic!("defmt logger taken reentrantly");
            }

            TAKEN.store(true, Ordering::Relaxed);

            // Safety: the `static mut`s are only accessed in the critical section.
            unsafe {
                CS_RESTORE = restore;

                (*addr_of_mut!(FRAME)).len = 0;
                (*addr_of_mut!(ENCODER)).start_frame(push);
            }
        }

        unsafe fn flush() {}

        unsafe fn release() {
            // Safety: the `static mut`s are only accessed in the critical section.
            unsafe {
                (*addr_of_mut!(ENCODER)).end_frame(push);

                // A frame is queued whole or not at all: a partial one would garble the
                // next one too. Nothing reads the queue while the critical section is held.
                if let Some(mut frame) = (*addr_of!(FRAME)).bytes() {
                    if QUEUE.free_capacity() >= frame.len() {
                        // The queue is a ring: a write stops at its wrap.
                        while let Ok(written @ 1..) = QUEUE.try_write(frame) {
                            frame = &frame[written..];
                        }
                    }
                }

                TAKEN.store(false, Ordering::Relaxed);

                critical_section::release(CS_RESTORE);
            }
        }

        unsafe fn write(bytes: &[u8]) {
            // Safety: the `static mut`s are only accessed in the critical section.
            unsafe { (*addr_of_mut!(ENCODER)).write(bytes, push) }
        }
    }

    fn push(bytes: &[u8]) {
        // Safety: only called by the encoder, in the critical section.
        unsafe { (*addr_of_mut!(FRAME)).push(bytes) }
    }

    /// The frame being encoded.
    struct Frame {
        buf: [u8; MAX_FRAME_LEN],
        /// Grows past `MAX_FRAME_LEN` when the frame does not fit.
        len: usize,
    }

    impl Frame {
        const fn new() -> Self {
            Self {
                buf: [0; MAX_FRAME_LEN],
                len: 0,
            }
        }

        fn push(&mut self, bytes: &[u8]) {
            let end = self.len + bytes.len();

            if end <= MAX_FRAME_LEN {
                self.buf[self.len..end].copy_from_slice(bytes);
            }

            self.len = end;
        }

        fn bytes(&self) -> Option<&[u8]> {
            (self.len <= MAX_FRAME_LEN).then(|| &self.buf[..self.len])
        }
    }
}

/// The Seeed XIAO nRF52840 and XIAO nRF54L15 boards.
#[cfg(feature = "xiao")]
mod xiao {
    use embassy_nrf::gpio::{Level, Output, OutputDrive};
    #[cfg(feature = "nrf52840")]
    use embassy_nrf::peripherals::{P0_20, P0_21, P0_22, P0_23, P0_25};
    #[cfg(feature = "nrf54l15")]
    use embassy_nrf::peripherals::{P2_03, P2_05};
    use embassy_nrf::Peri;

    /// Connect the radio to the XIAO nRF54L15's onboard ceramic antenna.
    ///
    /// The board routes the 2.4 GHz path through an RF switch, powered from P2.03, which picks
    /// the antenna with P2.05 (low: the onboard one, high: the U.FL connector). Unpowered, the
    /// switch disconnects both: frames from a node close by still get through, but next to
    /// nothing reaches a parent a few meters away. Powered, it draws ~90 µA by itself, so it is
    /// left off here, and powered only around each radio activity ([`rf_switch_power`]).
    #[cfg(feature = "nrf54l15")]
    pub fn rf_switch_onboard_antenna(power: Peri<'static, P2_03>, select: Peri<'static, P2_05>) {
        Output::new(power, Level::Low, OutputDrive::Standard).persist();
        Output::new(select, Level::Low, OutputDrive::Standard).persist();
    }

    /// Power the RF switch, or not: the hook MPSL calls around each radio activity. It runs in
    /// MPSL's interrupt context, so it writes the GPIO registers directly.
    #[cfg(feature = "nrf54l15")]
    pub fn rf_switch_power(on: bool) {
        use embassy_nrf::pac;

        if on {
            pac::P2
                .outset()
                .write_value(pac::gpio::regs::Outset(1 << 3));
        } else {
            pac::P2
                .outclr()
                .write_value(pac::gpio::regs::Outclr(1 << 3));
        }
    }

    /// Load the XIAO nRF54L15's crystals - the 32 MHz one and the 32.768 kHz one, both for
    /// 16 pF - with the chip's internal capacitors, before either is started.
    ///
    /// Left at their reset value, both crystals run off their nominal frequency: the 32 MHz one
    /// clocks the radio's carrier, the 32.768 kHz one CSL's timing. The register values follow
    /// from the chip's own calibration (FICR), as in Nordic's start-up code.
    #[cfg(feature = "nrf54l15")]
    pub fn crystal_load_capacitors() {
        use embassy_nrf::pac;

        const LOAD_CAPACITANCE_FF: i64 = 16_000;

        // The FICR trims' slope is a 9-bit two's complement value.
        let slope = |slope: u16| (slope as i64 ^ 0x100) - 0x100;
        let div_round = |n: i64, d: i64| (n + d / 2) / d;

        let trim = pac::FICR.xosc32ktrim().read();
        let lfxo = div_round(
            (2 * LOAD_CAPACITANCE_FF - 12_000) * (slope(trim.slope()) + 392)
                + ((trim.offset() as i64) << 3) * 1000,
            512_000,
        );

        let trim = pac::FICR.xosc32mtrim().read();
        let hfxo = div_round(
            ((LOAD_CAPACITANCE_FF - 5_500) * (slope(trim.slope()) + 791)
                + ((trim.offset() as i64) << 2) * 1000)
                >> 8,
            1000,
        );

        defmt::info!(
            "Crystal load capacitors: 32 MHz INTCAP {}, 32.768 kHz INTCAP {}",
            hfxo,
            lfxo
        );

        pac::OSCILLATORS
            .xosc32ki()
            .intcap()
            .write(|w| w.set_val(lfxo as u8));
        pac::OSCILLATORS
            .xosc32m()
            .config()
            .intcap()
            .write(|w| w.set_val(hfxo as u8));
    }

    /// Put the XIAO nRF52840's onboard 2 MB QSPI NOR flash, which this example does not use,
    /// into deep power-down.
    ///
    /// Left alone, its pins stay disconnected on the nRF side and its chip select floats, which
    /// can keep the flash (partly) selected and its input buffers conducting. The command
    /// (0xB9) is bit-banged in SPI mode 0, and the flash's inputs then stay driven.
    #[cfg(feature = "nrf52840")]
    pub fn flash_deep_power_down(
        csn: Peri<'static, P0_25>,
        sck: Peri<'static, P0_21>,
        io0: Peri<'static, P0_20>,
        io2: Peri<'static, P0_22>,
        io3: Peri<'static, P0_23>,
    ) {
        const DEEP_POWER_DOWN: u8 = 0xb9;

        let mut csn = Output::new(csn, Level::High, OutputDrive::Standard);
        let mut sck = Output::new(sck, Level::Low, OutputDrive::Standard);
        let mut si = Output::new(io0, Level::Low, OutputDrive::Standard);
        // WP# and HOLD#: a low HOLD# would pause the command.
        let wp = Output::new(io2, Level::High, OutputDrive::Standard);
        let hold = Output::new(io3, Level::High, OutputDrive::Standard);

        csn.set_low();

        for bit in (0..8).rev() {
            si.set_level(Level::from(DEEP_POWER_DOWN & (1 << bit) != 0));
            sck.set_high();
            sck.set_low();
        }

        csn.set_high();

        for pin in [csn, sck, si, wp, hold] {
            pin.persist();
        }
    }
}

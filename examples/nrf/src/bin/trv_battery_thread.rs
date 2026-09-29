//! An example of a Short Idle Time (SIT) Intermittently Connected Device (ICD) over Thread on
//! the nRF52840 / nRF54L15: a battery-powered radiator valve (a heating-only Thermostat).
//!
//! The nRF counterpart of the ESP32-C6 `trv_battery_thread` example - see it for what a SIT
//! ICD is and why an actuator is the right showcase for it. This one shows the golden case
//! for a SIT device: **reachable within half a second, at a single-digit µA average**. Two
//! things make that possible here:
//!
//! - **The radio listens instead of polling.** `nrf-802154` supports timed receive, so the
//!   Thread driver maps the ICD intervals onto a CSL (Coordinated Sampled Listening) period:
//!   the node announces a listening schedule and its Thread 1.2+ parent transmits into the
//!   sub-millisecond receive windows, no data-poll transmission needed. A data poll is a
//!   transmission plus an ACK wait, milliseconds of radio time; a CSL sample is a few hundred
//!   microseconds of receive. That is what makes a 500 ms idle interval cost a few µA on
//!   average instead of tens, so the device can afford to be sub-second *all the time*.
//!   Against a Thread 1.1 parent OpenThread falls back to polling on its own.
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
/// (500 ms).
///
/// `idle_mode_duration_s` has no real role for a SIT device: it is how long a LIT may stay
/// unreachable before it wakes up on its own and sends its Check-Ins, while a SIT is reachable
/// within `SII` all along. The state machine still honors it (every idle period ends in a
/// short active one), so it is simply set to the 15 s a SIT may at most be idle for.
const ICD_MODE: IcdModeConfig = IcdModeConfig {
    idle_mode_duration_s: 15,
    active_mode_duration_ms: 1000,
    active_mode_threshold_ms: 1000,
    user_active_mode_trigger_hint: 0,
    user_active_mode_trigger_instruction: "",
};

const BUMP_SIZE: usize = 21000;

#[global_allocator]
static HEAP: LlffHeap = LlffHeap::empty();

/// We need a bigger log ring-buffer or else the device QR code printout is half-lost
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

    rtt_target::rtt_init_defmt!(rtt_target::ChannelMode::NoBlockSkip, LOG_RINGBUF_SIZE);

    info!("Starting...");

    let p = init();

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
    let discriminator = (weak_rand.next_u32() & 0xfff) as u16;

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
/// controllers *and* the CSL periods of the Thread node: it listens every 300 ms while the
/// ICD is active and every 500 ms while idle. The idle interval is what a controller waits
/// for, so it is the reaction time of the valve; with CSL a sub-second one is affordable
/// (a 15 s one, the slowest a SIT may use and the natural choice when polling, would save
/// less than a µA here). The controllers also derive their retransmission timing from these,
/// so they must be what the radio really does.
const TEST_BASIC_INFO: BasicInfoConfig = BasicInfoConfig {
    sai: Some(300),
    sii: Some(500),
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

    #[allow(unused_mut)]
    let mut config = embassy_nrf::config::Config::default();

    #[cfg(feature = "nrf52840")]
    {
        config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
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

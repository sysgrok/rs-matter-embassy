//! An example of a Short Idle Time (SIT) Intermittently Connected Device (ICD) over Thread:
//! a battery-powered radiator valve (a heating-only Thermostat).
//!
//! A SIT ICD is the profile for battery devices that must be *reachable on the controller's
//! initiative* within seconds: a controller writes a setpoint or changes the mode, and the user
//! expects the valve to react right away; controllers also read and subscribe to its local
//! temperature. That is what distinguishes it from a sensor that only pushes readings now and
//! then - such a device is a Long Idle Time ICD, see `temp_sensor_battery_thread`, and making it
//! a SIT wastes its battery on polls nobody needs.
//!
//! On the network a SIT device is a Thread Sleepy End Device that keeps its receiver off between
//! data polls and polls its parent at most every 15 seconds, so anything a controller sends
//! arrives within that window. Its sleep model is *light sleep*: the MCU parks between polls
//! with its RAM retained, so the Thread attachment, the Matter sessions and the subscriptions
//! all stay alive, and every wake-up is a few milliseconds: poll the parent, handle whatever it
//! queued, sleep again. The valve motor itself draws next to nothing, so the radio duty cycle
//! is what decides the battery life.
//!
//! What this example wires up:
//! - the ICD Management cluster on the root endpoint, backed by a shared `Icd` state;
//! - the Thread driver following that state: the node polls fast (`SAI`) while the ICD is in
//!   active mode and slowly (`SII`, capped to 15 s for SIT) while idle;
//! - a flash-backed KV store, so the fabrics, the ICD registrations, the CASE resumption
//!   records, the thermostat's setpoints and the OpenThread attachment state survive a reboot;
//! - CASE session resumption (the `case-resumption` feature), so a controller coming back after
//!   a while re-establishes its session with one round trip instead of a full handshake;
//! - `esp-rtos` automatic light sleep (the idle hook), which is what makes the MCU actually
//!   sleep between polls once the radio drivers allow it - see the note below.
//!
//! NOTE: as of this writing `esp-radio` holds a wake lock for as long as the radio is
//! initialized, which keeps the automatic light sleep from ever engaging. Until that is
//! lifted upstream, this example demonstrates the complete ICD protocol behavior (poll
//! periods, active/idle modes, BLE only during commissioning) but the C6 does not yet reach
//! its light-sleep current. Once `esp-radio` releases the lock while the 802.15.4 receiver is
//! off, nothing here needs to change.
//!
//! The BOOT button (GPIO9) held for a few seconds factory-resets the device.
#![no_std]
#![no_main]
#![recursion_limit = "256"]

use core::borrow::BorrowMut;
use core::pin::pin;

use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};

use esp_alloc::heap_allocator;
use esp_backtrace as _;
use esp_bootloader_esp_idf::partitions::{
    read_partition_table, DataPartitionSubType, PartitionType, PARTITION_TABLE_MAX_LEN,
};
use esp_hal::gpio::{Input, InputConfig, Pull};
use esp_hal::ram;
use esp_hal::timer::timg::TimerGroup;
use esp_metadata_generated::memory_range;
use esp_storage::FlashStorage;

use log::{info, warn};

use rs_matter_embassy::matter::crypto::{default_crypto, Crypto, Rng};
use rs_matter_embassy::matter::dm::clusters::app::thermostat::{
    self, ControlSequenceOfOperationEnum, RelayStateBitmap, SystemModeEnum, ThermostatHooks,
};
use rs_matter_embassy::matter::dm::clusters::basic_info::BasicInfoConfig;
use rs_matter_embassy::matter::dm::clusters::decl::thermostat as thermostat_cluster;
use rs_matter_embassy::matter::dm::clusters::desc::{self, ClusterHandler as _};
use rs_matter_embassy::matter::dm::clusters::icd_mgmt::{
    ClusterHandler as _, Icd, IcdMgmtHandler, IcdModeConfig,
};
use rs_matter_embassy::matter::dm::devices::test::{
    DAC_PRIVKEY, TEST_DEV_ATT, TEST_DEV_COMM, TEST_DEV_DET,
};
use rs_matter_embassy::matter::dm::devices::{DEV_TYPE_ROOT_NODE, DEV_TYPE_THERMOSTAT};
use rs_matter_embassy::matter::dm::endpoints::ROOT_ENDPOINT_ID;
use rs_matter_embassy::matter::dm::{Async, Cluster, Dataver, EmptyHandler, Endpoint, Node};
use rs_matter_embassy::matter::error::Error;
use rs_matter_embassy::matter::persist::{KvBlobStore, VENDOR_KEYS_START};
use rs_matter_embassy::matter::utils::cell::RefCell;
use rs_matter_embassy::matter::utils::init::InitMaybeUninit;
use rs_matter_embassy::matter::utils::select::Coalesce;
use rs_matter_embassy::matter::utils::sync::blocking::Mutex;
use rs_matter_embassy::matter::{clusters, devices, with};
use rs_matter_embassy::persist::SeqMapKvBlobStore;
use rs_matter_embassy::stack::rand::reseeding_csprng;
use rs_matter_embassy::wireless::esp::EspThreadDriver;
use rs_matter_embassy::wireless::{EmbassyThread, EmbassyThreadMatterStack};

use tinyrlibc as _;

extern crate alloc;

macro_rules! mk_static {
    ($t:ty) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        STATIC_CELL.uninit()
    }};
}

/// The amount of memory for allocating all `rs-matter-stack` futures created during
/// the execution of the `run*` methods.
/// This does NOT include the rest of the Matter stack.
const BUMP_SIZE: usize = 20000;

/// Heap strictly necessary only for Thread+BLE and for the only Matter dependency which needs (~4KB) alloc - `x509`
const HEAP_SIZE: usize = 100 * 1024;

const RECLAIMED_RAM: usize =
    memory_range!("DRAM2_UNINIT").end - memory_range!("DRAM2_UNINIT").start;

/// How long the BOOT button has to be held to factory-reset the device.
const RESET_SECS: u64 = 3;

/// The ICD mode timings this device advertises.
///
/// A SIT device: it may idle for up to a minute between its own wake-ups, stays active for a
/// second after each one, and for a second after any Matter message. Nothing here bounds the
/// *polling* interval - that is the `SII` in `BASIC_INFO`, capped to 15 s by the stack.
const ICD_MODE: IcdModeConfig = IcdModeConfig {
    idle_mode_duration_s: 60,
    active_mode_duration_ms: 1000,
    active_mode_threshold_ms: 1000,
    user_active_mode_trigger_hint: 0,
    user_active_mode_trigger_instruction: "",
};

/// How far ahead the persisted Check-In counter boundary jumps: this many Check-Ins may be sent
/// between two flash writes.
const ICD_COUNTER_EPOCH: u32 = 100;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_s: Spawner) {
    esp_println::logger::init_logger_from_env();

    info!("Starting...");

    heap_allocator!(size: HEAP_SIZE - RECLAIMED_RAM);
    heap_allocator!(#[ram(reclaimed)] size: RECLAIMED_RAM);

    // Necessary `esp-hal` initialization boilerplate

    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Automatic light sleep: whenever the scheduler runs out of ready tasks (and no wake lock
    // is held), the idle hook light-sleeps the chip until the next timer deadline or wake-up
    // source. See the module docs for why this does not engage with `esp-radio` yet.
    let sleep = esp_rtos::sleep::configure(peripherals.LPWR);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start_with_idle_hook(
        timg0.timer0,
        peripherals.FROM_CPU_INTR0,
        sleep.light_sleep_hook,
    );

    // Create the crypto provider, using the `esp-hal` TRNG/ADC1 as the source of randomness for a reseeding CSPRNG.
    let _trng_source = esp_hal::rng::TrngSource::new(peripherals.RNG, peripherals.ADC1);
    let crypto = default_crypto(
        reseeding_csprng(esp_hal::rng::Trng::try_new().unwrap(), 1000).unwrap(),
        DAC_PRIVKEY,
    );

    let mut weak_rand = crypto.weak_rand().unwrap();

    // TODO
    let mut ieee_eui64 = [0; 8];
    weak_rand.fill_bytes(&mut ieee_eui64);

    // Allocate the Matter stack.
    // For MCUs, it is best to allocate it statically, so as to avoid program stack blowups (its memory footprint is ~ 35 to 50KB).
    // It is also (currently) a mandatory requirement when the wireless stack variation is used.
    let stack = mk_static!(EmbassyThreadMatterStack::<BUMP_SIZE, ()>).init_with(
        EmbassyThreadMatterStack::init(&BASIC_INFO, TEST_DEV_COMM, &TEST_DEV_ATT),
    );

    // The shared ICD state: the registrations, the Check-In counter and the power mode state
    // machine. Backs the ICD Management cluster handler, and is followed by the Thread driver.
    let icd: &'static Icd = mk_static!(Icd).init_with(Icd::init(ICD_COUNTER_EPOCH, ICD_MODE));

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
        // The stack adds the operational network clusters on top.
        .chain(
            |e, _| e == ROOT_ENDPOINT_ID,
            Async(EmbassyThreadMatterStack::<0, ()>::root_handler(
                &(),
                &mut weak_rand,
            )),
        )
        // The ICD Management cluster, on Endpoint 0 as well. Its `run` hook drives the ICD
        // power mode state machine and sends the Check-In messages.
        .chain(
            |e, c| e == ROOT_ENDPOINT_ID && c == ICD_MGMT_CLUSTER.id,
            Async(IcdMgmtHandler::new(Dataver::new_rand(&mut weak_rand), icd).adapt()),
        )
        // Our thermostat cluster, on Endpoint 1
        .chain(
            |e, c| e == VALVE_ENDPOINT_ID && c == RadiatorValve::CLUSTER.id,
            Async(thermostat::HandlerAdaptor(&thermostat)),
        )
        // Each Endpoint needs a Descriptor cluster too
        .chain(
            |e, c| e == VALVE_ENDPOINT_ID && c == desc::DescHandler::CLUSTER.id,
            Async(desc::DescHandler::new(Dataver::new_rand(&mut weak_rand)).adapt()),
        );

    // A flash-backed KV BLOB store: an ICD has to remember its fabrics, its ICD registrations,
    // its CASE resumption records and its Thread attachment across reboots.
    let mut pt_buf = [0u8; PARTITION_TABLE_MAX_LEN];
    let mut store = get_persistent_store(peripherals.FLASH, &mut pt_buf[..]);
    stack.startup(&crypto, &mut store).await.unwrap();

    if stack.matter().has_fabrics() {
        info!(
            "To reset, press and hold the Boot Mode pin (GPIO9) for {} or more seconds",
            RESET_SECS
        );
    }

    {
        let kv = stack.matter().kv(&mut store);

        // Run the Matter stack with our handler
        let mut matter = pin!(stack.run_coex(
            // The Matter stack needs to instantiate an `openthread` Radio - as a Sleepy End
            // Device following the ICD state
            EmbassyThread::new(
                EspThreadDriver::new(peripherals.IEEE802154, peripherals.BT),
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
            (NODE, &handler),
            // The Matter stack needs a blob store to store its state
            &kv,
            // No user future to run
            (),
        ));

        // Run Matter and also wait for a reset signal. (The Thread driver logs every ICD power
        // mode transition, so a serial capture shows the duty cycle.)
        let mut wait_reset = pin!(wait_pin_low(Input::new(
            peripherals.GPIO9,
            InputConfig::default().with_pull(Pull::Down)
        )));

        select(&mut matter, &mut wait_reset)
            .coalesce()
            .await
            .unwrap();
    }

    // If we get here, with no errors, this means the user is willing to reset the storage
    // by holding the BOOT pin low 3 or more seconds
    warn!("Resetting storage");

    stack.reset(&crypto, (NODE, &handler), store).await.unwrap();

    warn!("Rebooting...");

    esp_hal::system::software_reset()
}

/// Basic info about our device.
///
/// The `SAI` / `SII` (session active / idle intervals) are what the device advertises to
/// controllers *and* the polling intervals of the Thread Sleepy End Device: 300 ms while
/// the ICD is active, 15 s while idle - the slowest a SIT device may poll.
const BASIC_INFO: BasicInfoConfig = BasicInfoConfig {
    sai: Some(300),
    sii: Some(15_000),
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
            "Valve: mode {:?}, setpoint {}.{:02} C, {}",
            system_mode,
            heating_setpoint / 100,
            (heating_setpoint % 100).abs(),
            if self.heating() { "open" } else { "closed" }
        );
    }
}

/// The ICD Management cluster metadata, exactly as served by `IcdMgmtHandler`.
const ICD_MGMT_CLUSTER: Cluster<'static> = IcdMgmtHandler::CLUSTER;

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

/// The BLOB storage returned by this function is persisting in the first partition of type 'NVS'
/// found in the NOR-FLASH of the chip.
///
/// If no such partition is found, the function will panic.
fn get_persistent_store<'d>(
    flash: esp_hal::peripherals::FLASH<'d>,
    mut buf: impl BorrowMut<[u8]>,
) -> impl KvBlobStore + 'd {
    let mut flash = FlashStorage::new(flash);
    let pt_buf = &mut buf.borrow_mut()[..PARTITION_TABLE_MAX_LEN];
    let pt = read_partition_table(&mut flash, pt_buf).unwrap();
    let nvs = pt
        .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
        .unwrap()
        .unwrap();

    let start = nvs.offset();
    let end = nvs.offset() + nvs.len();
    info!(
        "Will use NVS partition \"{}\" at {:#x}..{:#x}",
        nvs.label_as_str(),
        start,
        end
    );

    SeqMapKvBlobStore::new(BlockingAsync::new(flash), start..end)
}

/// Resolve once the BOOT pin is held low for `RESET_SECS`.
async fn wait_pin_low(mut pin: Input<'_>) -> Result<(), Error> {
    loop {
        pin.wait_for_low().await;

        // Debounce
        embassy_time::Timer::after_millis(50).await;

        if pin.is_low() {
            warn!(
                "Detected Boot Mode pin low, keep it low for {} more seconds to reset the storage",
                RESET_SECS
            );

            let result = select(
                pin.wait_for_high(),
                embassy_time::Timer::after_secs(RESET_SECS),
            )
            .await;

            if matches!(result, Either::Second(())) {
                break Ok(());
            }
        }
    }
}

//! An example of a Long Idle Time (LIT) Intermittently Connected Device (ICD) over Thread.
//!
//! A LIT ICD is the profile for devices that report rarely and never need to react quickly -
//! a soil moisture, temperature or air quality sensor. Its slow polling interval is not
//! bounded to 15 s: it polls (and is reachable) once per `IdleModeDuration`, minutes apart.
//! Controllers that want to talk to it *register* as Check-In clients through the ICD
//! Management cluster; whenever the device wakes up from idle mode it sends them a Check-In
//! message if their subscription is gone, and they then have `ActiveModeDuration` to
//! re-establish it. Until at least one client is registered, a LIT-capable device behaves as
//! a SIT one (it polls at least every 15 s), so that plain controllers can commission it.
//!
//! With wake-ups that far apart, the sleep model is *deep sleep*: the chip powers everything
//! but the low-power timer down (single-digit microamps on the ESP32-C6), and every wake-up is
//! a reboot. That works because everything that matters is persisted: the fabrics, the ICD
//! registrations and Check-In counter, the CASE resumption records, the subscriptions and -
//! from `rs-matter-embassy`'s OpenThread persister - the Thread attachment state, so that
//! OpenThread re-attaches to its parent with a single `Child Update Request` instead of a full
//! attach. The reboot itself (bootloader, radio, OpenThread, Matter) is the dominant cost of a
//! wake-up, which is why deep sleep only pays off with wake-ups minutes apart, and why the SIT
//! example (`trv_battery_thread`) uses light sleep instead.
//!
//! What this example wires up, on top of the `trv_battery_thread` (SIT) one:
//! - a LIT mode config (`ActiveModeThreshold >= 5 s`, minutes of idle time) and a `SII` of the
//!   same length, so that the advertised polling interval matches the sleep period;
//! - a user trigger (the `UserActiveModeTrigger` the cluster advertises): pulling GPIO4 low
//!   wakes the device from deep sleep; on the ESP32-C6 only GPIO0..7 have the low-power path
//!   a deep-sleep wake-up needs, so the BOOT button (GPIO9) cannot serve;
//! - the deep-sleep loop: once the ICD state machine reports idle mode, the application waits
//!   for pending flash writes to drain and deep-sleeps until the next poll is due.
//!
//! The device is a Temperature Sensor: it takes one (simulated) reading per wake-up, which a
//! subscribed controller receives as a report during the active window. A wake-up counter kept
//! in RTC RAM across deep sleeps drives the simulation, so consecutive reports differ. The BOOT
//! button (GPIO9) held for a few seconds while the device is awake factory-resets it.
#![no_std]
#![no_main]
#![recursion_limit = "256"]

use core::borrow::BorrowMut;
use core::pin::pin;

use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_executor::Spawner;
use embassy_futures::select::{select, select3, Either};

use esp_alloc::heap_allocator;
use esp_backtrace as _;
use esp_bootloader_esp_idf::partitions::{
    read_partition_table, DataPartitionSubType, PartitionType, PARTITION_TABLE_MAX_LEN,
};
use esp_hal::gpio::{Event, Input, InputConfig, Pull, WakeupConfig};
use esp_hal::ram;
use esp_hal::rtc_cntl::sleep::{LowPower, RtcSleepConfig};
use esp_hal::rtc_cntl::{wakeup_cause, WakeupSource};
use esp_hal::timer::timg::TimerGroup;
use esp_metadata_generated::memory_range;
use esp_storage::FlashStorage;

use log::{info, warn};

use rs_matter_embassy::matter::crypto::{default_crypto, Crypto, Rng};
use rs_matter_embassy::matter::dm::clusters::basic_info::BasicInfoConfig;
use rs_matter_embassy::matter::dm::clusters::decl::temperature_measurement::{
    self, ClusterHandler as _,
};
use rs_matter_embassy::matter::dm::clusters::desc::{self, ClusterHandler as _};
use rs_matter_embassy::matter::dm::clusters::icd_mgmt::{
    ClusterHandler as _, Icd, IcdMgmtHandler, IcdModeConfig, IcdPowerMode,
};
use rs_matter_embassy::matter::dm::devices::test::{
    DAC_PRIVKEY, TEST_DEV_ATT, TEST_DEV_COMM, TEST_DEV_DET,
};
use rs_matter_embassy::matter::dm::devices::DEV_TYPE_ROOT_NODE;
use rs_matter_embassy::matter::dm::endpoints::ROOT_ENDPOINT_ID;
use rs_matter_embassy::matter::dm::{
    Async, Cluster, Dataver, DeviceType, EmptyHandler, Endpoint, Node, ReadContext,
};
use rs_matter_embassy::matter::error::Error;
use rs_matter_embassy::matter::persist::KvBlobStore;
use rs_matter_embassy::matter::tlv::Nullable;
use rs_matter_embassy::matter::utils::init::InitMaybeUninit;
use rs_matter_embassy::matter::utils::select::Coalesce;
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

/// How long the device sleeps between two wake-ups, in seconds. Also the `IdleModeDuration`
/// and the `SII` the device advertises: they have to agree, as a deep-sleeping device polls
/// only when it wakes up.
const SLEEP_SECS: u32 = 300;

/// The ICD mode timings this device advertises.
///
/// A LIT device: it idles for `SLEEP_SECS` between its own wake-ups, stays active for five
/// seconds after each one (enough for a registered client to re-subscribe after the Check-In)
/// and for five seconds after any Matter message (the spec minimum for LIT).
const ICD_MODE: IcdModeConfig = IcdModeConfig {
    idle_mode_duration_s: SLEEP_SECS,
    active_mode_duration_ms: 5000,
    active_mode_threshold_ms: 5000,
    // `CustomInstruction`: the instruction string below tells the user how to wake it up.
    user_active_mode_trigger_hint: 0x0004,
    user_active_mode_trigger_instruction: "Pull GPIO4 low to wake the device",
};

/// How long to wait after entering idle mode before deep-sleeping, so that the background
/// persistence tasks (fabrics, subscriptions, OpenThread settings) flush to flash first.
const SLEEP_SETTLE_MS: u64 = 500;

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

    // Why did we boot: a cold start, the sleep timer, or the user trigger?
    let cause = wakeup_cause();
    if cause.is_empty() {
        info!("Cold boot");
    } else if cause.contains(WakeupSource::Gpio) {
        info!("Woken up by the user trigger (GPIO4)");
    } else {
        info!("Woken up from deep sleep by the timer");
    }

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    // The user trigger: GPIO4, pulled up, wakes the chip when pulled low. Configured for the
    // low-power path (the one that works with the digital GPIO peripheral powered down), and
    // armed - `listen` - right before each deep sleep.
    let mut wake_pin = Input::new(
        peripherals.GPIO4,
        InputConfig::default().with_pull(Pull::Up),
    );
    wake_pin
        .apply_wakeup_config(&WakeupConfig::default().with_low_power_path(true))
        .unwrap();

    let mut lpwr = LowPower::new(peripherals.LPWR);

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

    // One reading per wake-up. The wake-up counter lives in RTC RAM, which deep sleep keeps.
    let wake_count = {
        // SAFETY: the only access to the static, before any task runs.
        let count = unsafe { &mut *core::ptr::addr_of_mut!(WAKE_COUNT) };
        *count = count.wrapping_add(1);
        *count
    };
    let sensor = TemperatureSensor::new(
        Dataver::new_rand(&mut weak_rand),
        simulated_reading(wake_count),
    );
    info!(
        "Wake-up #{}, temperature {} centi-degrees",
        wake_count,
        simulated_reading(wake_count)
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
        // Our temperature sensor, on Endpoint 1
        .chain(
            |e, c| e == SENSOR_ENDPOINT_ID && c == TemperatureSensor::CLUSTER.id,
            Async(temperature_measurement::HandlerAdaptor(&sensor)),
        )
        // Each Endpoint needs a Descriptor cluster too
        .chain(
            |e, c| e == SENSOR_ENDPOINT_ID && c == desc::DescHandler::CLUSTER.id,
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

        // Deep-sleep as soon as the ICD state machine lets us. (The Thread driver logs every
        // ICD power mode transition, so a serial capture shows the duty cycle.)
        let mut deep_sleep = pin!(deep_sleep_when_idle(icd, &mut lpwr, &mut wake_pin));

        // Run Matter and also wait for a reset signal
        let mut wait_reset = pin!(wait_pin_low(Input::new(
            peripherals.GPIO9,
            InputConfig::default().with_pull(Pull::Down)
        )));

        select3(&mut matter, &mut deep_sleep, &mut wait_reset)
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

/// The wake-up counter, kept in RTC RAM across deep sleeps (zeroed on a cold boot only).
#[ram(unstable(rtc_fast, persistent))]
static mut WAKE_COUNT: u32 = 0;

/// A simulated temperature reading, in centi-degrees Celsius: a slow triangle wave around
/// 21 °C, one step per wake-up.
fn simulated_reading(wake_count: u32) -> i16 {
    const BASE: i16 = 2100;
    const STEPS: u32 = 20;

    let step = (wake_count % (2 * STEPS)) as i16;
    let delta = if step < STEPS as i16 {
        step
    } else {
        2 * STEPS as i16 - step
    };

    BASE + delta * 25
}

/// A minimal Temperature Measurement cluster
///
/// Always reports a fixed reading.
/// In real life it would read from a sensor on each wake-up.
struct TemperatureSensor {
    dataver: Dataver,
    reading: i16,
}

impl TemperatureSensor {
    const fn new(dataver: Dataver, reading: i16) -> Self {
        Self { dataver, reading }
    }
}

impl temperature_measurement::ClusterHandler for TemperatureSensor {
    // Only the mandatory attributes; `Tolerance` is optional and not served.
    const CLUSTER: Cluster<'static> = temperature_measurement::FULL_CLUSTER
        .with_attrs(with!(required))
        .with_cmds(with!());

    fn dataver(&self) -> u32 {
        self.dataver.get()
    }

    fn dataver_changed(&self) {
        self.dataver.changed();
    }

    fn measured_value(&self, _ctx: impl ReadContext) -> Result<Nullable<i16>, Error> {
        Ok(Nullable::some(self.reading))
    }

    fn min_measured_value(&self, _ctx: impl ReadContext) -> Result<Nullable<i16>, Error> {
        Ok(Nullable::some(-4000))
    }

    fn max_measured_value(&self, _ctx: impl ReadContext) -> Result<Nullable<i16>, Error> {
        Ok(Nullable::some(8500))
    }
}

/// Deep-sleep whenever the ICD enters idle mode, until the next poll is due.
///
/// Only once the device is commissioned: an uncommissioned device has a commissioning window
/// open (which keeps the ICD active anyway) or nothing to sleep for.
///
/// Never returns: a deep sleep ends in a reboot.
async fn deep_sleep_when_idle(
    icd: &Icd,
    lpwr: &mut LowPower<'_>,
    wake_pin: &mut Input<'_>,
) -> Result<(), Error> {
    loop {
        icd.wait_idle().await;

        // Let the persistence tasks flush whatever the active period changed.
        embassy_time::Timer::after_millis(SLEEP_SETTLE_MS).await;

        // Still idle? (A message might have arrived meanwhile.)
        if icd.power_mode() != IcdPowerMode::Idle {
            continue;
        }

        // Sleep until the next poll is due - never longer than the idle period, whichever
        // comes first. A LIT polls once per idle period; a LIT-capable device without
        // registered clients (operating as SIT) is capped to 15 s by the state machine.
        let poll_ms = icd.net_params().poll_interval_ms as u64;
        let idle_ms = icd
            .idle_until()
            .map(|until| until.saturating_duration_since(embassy_time::Instant::now()))
            .map(|d| d.as_millis())
            .unwrap_or(poll_ms);
        let sleep_ms = poll_ms.min(idle_ms).max(1000);

        info!("ICD idle: deep-sleeping for {} ms", sleep_ms);

        lpwr.set_wakeup_deadline(
            esp_hal::time::Instant::now() + esp_hal::time::Duration::from_millis(sleep_ms),
        );

        // Arm the user trigger. The pad is pulled up, so it does not wake us on its own.
        wake_pin.listen(Event::LowLevel);

        lpwr.sleep_deep(RtcSleepConfig::deep());
    }
}

/// Basic info about our device.
///
/// The `SAI` / `SII` (session active / idle intervals) are what the device advertises to
/// controllers *and* the polling intervals of the Thread Sleepy End Device: 300 ms while
/// the ICD is active, the sleep period while idle (a LIT is reachable once per idle period).
const BASIC_INFO: BasicInfoConfig = BasicInfoConfig {
    sai: Some(300),
    sii: Some(SLEEP_SECS * 1000),
    ..TEST_DEV_DET
};

/// Endpoint 0 (the root endpoint) always runs
/// the hidden Matter system clusters, so we pick ID=1
const SENSOR_ENDPOINT_ID: u16 = 1;

/// The Temperature Sensor device type (Matter Device Library), not yet among the device types
/// `rs-matter` predefines.
const DEV_TYPE_TEMPERATURE_SENSOR: DeviceType = DeviceType {
    dtype: 0x0302,
    drev: 2,
};

/// The ICD Management cluster metadata, exactly as served by `IcdMgmtHandler`.
const ICD_MGMT_CLUSTER: Cluster<'static> = IcdMgmtHandler::CLUSTER;

/// The Matter Temperature Sensor Node.
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
            SENSOR_ENDPOINT_ID,
            devices!(DEV_TYPE_TEMPERATURE_SENSOR),
            clusters!(desc::DescHandler::CLUSTER, TemperatureSensor::CLUSTER),
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

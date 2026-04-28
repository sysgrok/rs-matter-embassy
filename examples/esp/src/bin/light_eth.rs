//! An example utilizing the `EmbassyEthMatterStack` struct.
//! As the name suggests, this Matter stack assembly uses Ethernet as the main transport, as well as for commissioning.
//!
//! Notice thart we actually don't use Ethernet for real, as ESP32s don't have Ethernet ports out of the box.
//! Instead, we utilize Wifi, which - from the POV of Matter - is indistinguishable from Ethernet as long as the Matter
//! stack is not concerned with connecting to the Wifi network, managing its credentials etc. and can assume it "pre-exists".
//!
//! The example implements a fictitious Light device (an On-Off Matter cluster).
#![no_std]
#![no_main]
#![recursion_limit = "256"]

use core::env;
use core::pin::pin;

use embassy_executor::Spawner;
use embassy_futures::select::select;

use esp_alloc::heap_allocator;
use esp_backtrace as _;
use esp_hal::ram;
use esp_hal::timer::timg::TimerGroup;
use esp_metadata_generated::memory_range;
use esp_radio::wifi::sta::StationConfig;
use esp_radio::wifi::{Config, WifiController};

use log::info;

use rs_matter_embassy::epoch::epoch;
use rs_matter_embassy::eth::{EmbassyEthMatterStack, EmbassyEthernet, PreexistingEthDriver};
use rs_matter_embassy::matter::crypto::{default_crypto, Crypto};
use rs_matter_embassy::matter::dm::clusters::app::on_off::test::TestOnOffDeviceLogic;
use rs_matter_embassy::matter::dm::clusters::app::on_off::{self, OnOffHooks};
use rs_matter_embassy::matter::dm::clusters::desc::{self, ClusterHandler as _};
use rs_matter_embassy::matter::dm::devices::test::{
    DAC_PRIVKEY, TEST_DEV_ATT, TEST_DEV_COMM, TEST_DEV_DET,
};
use rs_matter_embassy::matter::dm::devices::DEV_TYPE_ON_OFF_LIGHT;
use rs_matter_embassy::matter::dm::{Async, Dataver, EmptyHandler, Endpoint, EpClMatcher, Node};
use rs_matter_embassy::matter::persist::DummyKvBlobStore;
use rs_matter_embassy::matter::utils::init::InitMaybeUninit;
use rs_matter_embassy::matter::utils::select::Coalesce;
use rs_matter_embassy::matter::{clusters, devices};
use rs_matter_embassy::stack::rand::reseeding_csprng;
use rs_matter_embassy::stack::utils::futures::IntoFaillble;

extern crate alloc;

macro_rules! mk_static {
    ($t:ty) => {{
        #[cfg(not(feature = "esp32"))]
        {
            static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
            STATIC_CELL.uninit()
        }
        #[cfg(feature = "esp32")]
        alloc::boxed::Box::leak(alloc::boxed::Box::<$t>::new_uninit())
    }};
}

/// The amount of memory for allocating all `rs-matter-stack` futures created during
/// the execution of the `run*` methods.
/// This does NOT include the rest of the Matter stack.
///
/// The futures of `rs-matter-stack` created during the execution of the `run*` methods
/// are allocated in a special way using a small bump allocator which results
/// in a much lower memory usage by those.
///
/// If - for your platform - this size is not enough, increase it until
/// the program runs without panics during the stack initialization.
const BUMP_SIZE: usize = 16500;

/// Heap strictly necessary only for Wifi+BLE and for the only Matter dependency which needs (~4KB) alloc - `x509`
#[cfg(not(feature = "esp32"))]
const HEAP_SIZE: usize = 100 * 1024;
/// On the esp32, we allocate the Matter Stack from heap as well, due to the non-contiguous memory regions on that chip
#[cfg(feature = "esp32")]
const HEAP_SIZE: usize = 140 * 1024;

const RECLAIMED_RAM: usize =
    memory_range!("DRAM2_UNINIT").end - memory_range!("DRAM2_UNINIT").start;

const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASS: &str = env!("WIFI_PASS");

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_s: Spawner) {
    esp_println::logger::init_logger_from_env();

    info!("Starting...");

    heap_allocator!(size: HEAP_SIZE - RECLAIMED_RAM);
    heap_allocator!(#[ram(reclaimed)] size: RECLAIMED_RAM);

    // Necessary `esp-hal` and `esp-wifi` initialization boilerplate

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT)
            .software_interrupt0,
    );

    // Allocate the Matter stack.
    // For MCUs, it is best to allocate it statically, so as to avoid program stack blowups (its memory footprint is ~ 35 to 50KB).
    // It is also (currently) a mandatory requirement when the wireless stack variation is used.
    let stack = mk_static!(EmbassyEthMatterStack::<BUMP_SIZE, ()>).init_with(
        EmbassyEthMatterStack::init(&TEST_DEV_DET, TEST_DEV_COMM, &TEST_DEV_ATT, epoch),
    );

    // Configure and start the Wifi first
    let wifi = peripherals.WIFI;
    let (controller, wifi_interface) =
        esp_radio::wifi::new(wifi, esp_radio::wifi::ControllerConfig::default()).unwrap();

    // Create the crypto provider, using the `esp-hal` TRNG as the source of randomness for a reseeding CSPRNG.
    let crypto = default_crypto(
        reseeding_csprng(esp_hal::rng::Trng::try_new().unwrap(), 1000).unwrap(),
        DAC_PRIVKEY,
    );

    let mut weak_rand = crypto.weak_rand().unwrap();

    // Our "light" on-off cluster.
    // It will toggle the light state every 5 seconds
    let on_off = on_off::OnOffHandler::new_standalone(
        Dataver::new_rand(&mut weak_rand),
        LIGHT_ENDPOINT_ID,
        TestOnOffDeviceLogic::new(true),
    );

    // Chain our endpoint clusters
    let handler = EmptyHandler
        // Our on-off cluster, on Endpoint 1
        .chain(
            EpClMatcher::new(
                Some(LIGHT_ENDPOINT_ID),
                Some(TestOnOffDeviceLogic::CLUSTER.id),
            ),
            on_off::HandlerAsyncAdaptor(&on_off),
        )
        // Each Endpoint needs a Descriptor cluster too
        // Just use the one that `rs-matter` provides out of the box
        .chain(
            EpClMatcher::new(Some(LIGHT_ENDPOINT_ID), Some(desc::DescHandler::CLUSTER.id)),
            Async(desc::DescHandler::new(Dataver::new_rand(&mut weak_rand)).adapt()),
        );

    // Create a KV BLOB store and load any previously saved state of `rs-matter`
    // `SeqMapKvBlobStore` saves to a user-supplied NOR Flash region
    // However, for this demo and for simplicity, we use a dummy KV BLOB store that does nothing
    let mut kv = DummyKvBlobStore;
    stack.startup(&crypto, &mut kv).await.unwrap();

    // Wrap the KV BLOB store as a shared reference, so that it can be used both by `rs-matter` and the user
    let kv = stack.create_shared_kv(kv).unwrap();

    // Run the Matter stack with our handler
    // Using `pin!` is completely optional, but reduces the size of the final future
    let mut matter = pin!(stack.run(
        // The Matter stack needs the ethernet inteface to run
        EmbassyEthernet::new(
            PreexistingEthDriver::new(wifi_interface.station),
            weak_rand,
            stack,
        ),
        // The crypto provider
        &crypto,
        // Our `AsyncHandler` + `AsyncMetadata` impl
        (NODE, handler),
        // The Matter stack needs a blob store to store its state
        &kv,
        // No user future to run
        (),
    ));

    // Schedule the Matter run & the Wifi connection monitoring together
    select(&mut matter, connection(controller).into_fallible())
        .coalesce()
        .await
        .unwrap();
}

async fn connection(mut controller: WifiController<'_>) {
    info!("start connection task");
    loop {
        if controller.is_connected() {
            // wait until we're no longer connected
            controller
                .wait_for_access_point_connected_event_async()
                .await
                .unwrap();
            embassy_time::Timer::after(embassy_time::Duration::from_millis(5000)).await
        }
        if !controller.is_started() {
            info!("Starting wifi");
            let client_config = Config::Station(
                StationConfig::default()
                    .with_ssid(WIFI_SSID)
                    .with_password(WIFI_PASS.into()),
            );
            controller.set_config(&client_config).unwrap();
        }
        info!("About to connect...");

        match controller.connect_async().await {
            Ok(_) => info!("Wifi connected!"),
            Err(e) => {
                info!("Failed to connect to wifi: {e:?}");
                embassy_time::Timer::after(embassy_time::Duration::from_millis(5000)).await
            }
        }
    }
}

/// Endpoint 0 (the root endpoint) always runs
/// the hidden Matter system clusters, so we pick ID=1
const LIGHT_ENDPOINT_ID: u16 = 1;

/// The Matter Light device Node
const NODE: Node = Node {
    endpoints: &[
        EmbassyEthMatterStack::<0, ()>::root_endpoint(),
        Endpoint {
            id: LIGHT_ENDPOINT_ID,
            device_types: devices!(DEV_TYPE_ON_OFF_LIGHT),
            clusters: clusters!(desc::DescHandler::CLUSTER, TestOnOffDeviceLogic::CLUSTER),
        },
    ],
};

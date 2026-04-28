//! An example utilizing the `EmbassyWifiMatterStack` struct
//! and additionally persisting the `rs-matter` state to the NOR Flash.
//!
//! As the name suggests, this Matter stack assembly uses Wifi as the main transport,
//! and thus BLE for commissioning.
//!
//! If you want to use Ethernet, utilize `EmbassyEthMatterStack` instead.
//! If you want to use non-concurrent commissioning, call `run` instead of `run_coex`
//! and provision a higher `BUMP_SIZE` because the non-concurrent commissioning has slightly higher
//! memory requirements on the futures' sizes.
//! (Note: Alexa does not work (yet) with non-concurrent commissioning.)
//!
//! The example implements a fictitious Light device (an On-Off Matter cluster).
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

use rs_matter_embassy::epoch::epoch;
use rs_matter_embassy::matter::crypto::{default_crypto, Crypto};
use rs_matter_embassy::matter::dm::clusters::app::on_off::test::TestOnOffDeviceLogic;
use rs_matter_embassy::matter::dm::clusters::app::on_off::{self, OnOffHooks};
use rs_matter_embassy::matter::dm::clusters::desc::{self, ClusterHandler as _};
use rs_matter_embassy::matter::dm::devices::test::{
    DAC_PRIVKEY, TEST_DEV_ATT, TEST_DEV_COMM, TEST_DEV_DET,
};
use rs_matter_embassy::matter::dm::devices::DEV_TYPE_ON_OFF_LIGHT;
use rs_matter_embassy::matter::dm::{Async, Dataver, EmptyHandler, Endpoint, EpClMatcher, Node};
use rs_matter_embassy::matter::error::Error;
use rs_matter_embassy::matter::persist::KvBlobStore;
use rs_matter_embassy::matter::utils::init::InitMaybeUninit;
use rs_matter_embassy::matter::utils::select::Coalesce;
use rs_matter_embassy::matter::{clusters, devices};
use rs_matter_embassy::persist::SeqMapKvBlobStore;
use rs_matter_embassy::stack::rand::reseeding_csprng;
use rs_matter_embassy::wireless::esp::EspWifiDriver;
use rs_matter_embassy::wireless::{EmbassyWifi, EmbassyWifiMatterStack};

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
const BUMP_SIZE: usize = 18000;

/// Heap strictly necessary only for Wifi+BLE and for the only Matter dependency which needs (~4KB) alloc - `x509`
#[cfg(not(feature = "esp32"))]
const HEAP_SIZE: usize = 100 * 1024;
/// On the esp32, we allocate the Matter Stack from heap as well, due to the non-contiguous memory regions on that chip
#[cfg(feature = "esp32")]
const HEAP_SIZE: usize = 140 * 1024;

const RECLAIMED_RAM: usize =
    memory_range!("DRAM2_UNINIT").end - memory_range!("DRAM2_UNINIT").start;

const RESET_SECS: u64 = 3;

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
    let stack = mk_static!(EmbassyWifiMatterStack::<BUMP_SIZE, ()>).init_with(
        EmbassyWifiMatterStack::init(&TEST_DEV_DET, TEST_DEV_COMM, &TEST_DEV_ATT, epoch),
    );

    // Create the crypto provider, using the `esp-hal` TRNG/ADC1 as the source of randomness for a reseeding CSPRNG.
    let _trng_source = esp_hal::rng::TrngSource::new(peripherals.RNG, peripherals.ADC1);
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
    let mut kv = get_persistent_store(peripherals.FLASH, stack.kv_store_buf().unwrap());
    stack.startup(&crypto, &mut kv).await.unwrap();

    if stack.is_commissioned() {
        info!(
            "To reset, press and hold the Boot Mode pin (GPIO9) for {} or more seconds",
            RESET_SECS
        );
    }

    {
        // Wrap the KV BLOB store as a shared reference, so that it can be used both by `rs-matter` and the user
        let kv = stack.create_shared_kv(&mut kv).unwrap();

        // Run the Matter stack with our handler
        // Using `pin!` is completely optional, but reduces the size of the final future
        //
        // This step can be repeated in that the stack can be stopped and started multiple times, as needed.
        let mut matter = pin!(stack.run_coex(
            // The Matter stack needs to instantiate an `embassy-net` `Driver` and `Controller`
            EmbassyWifi::new(
                EspWifiDriver::new(peripherals.WIFI, peripherals.BT),
                weak_rand,
                true, // Use a random BLE address
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

        // Run Matter and also wait for a reset signal
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

    stack.reset(kv).await.unwrap();

    warn!("Rebooting...");

    esp_hal::system::software_reset()
}

/// Endpoint 0 (the root endpoint) always runs
/// the hidden Matter system clusters, so we pick ID=1
const LIGHT_ENDPOINT_ID: u16 = 1;

/// The Matter Light device Node
const NODE: Node = Node {
    endpoints: &[
        EmbassyWifiMatterStack::<0, ()>::root_endpoint(),
        Endpoint {
            id: LIGHT_ENDPOINT_ID,
            device_types: devices!(DEV_TYPE_ON_OFF_LIGHT),
            clusters: clusters!(desc::DescHandler::CLUSTER, TestOnOffDeviceLogic::CLUSTER),
        },
    ],
};

/// The BLOB storage returned by this function is persisting in the first partition of type 'NVS'
/// found in the NOR-FLASH of the chip.
///
/// If no such partition is found, the function will panic.
///
/// You can alter this function to persist to a different partition,
/// or to use a completely different storage backend, as long as you return an implementation of `KvBlobStore`.
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
                break;
            }
        }
    }

    Ok(())
}

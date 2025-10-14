//! An example utilizing the `EmbassyThreadMatterStack` struct.
//!
//! As the name suggests, this Matter stack assembly uses Thread as the main transport,
//! and thus BLE for commissioning, in non-concurrent commissioning mode.
//!
//! If you want to use Ethernet, utilize `EmbassyEthMatterStack` instead.
//!
//! The example implements a fictitious Light device (an On-Off Matter cluster).
#![no_std]
#![no_main]
#![recursion_limit = "256"]

use core::pin::pin;

use alloc::boxed::Box;

use embassy_executor::Spawner;

use esp_alloc::heap_allocator;
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;

use log::info;

use rs_matter_embassy::epoch::epoch;
use rs_matter_embassy::matter::dm::clusters::basic_info::BasicInfoConfig;
use rs_matter_embassy::matter::dm::clusters::desc::{self, ClusterHandler as _};
use rs_matter_embassy::matter::dm::clusters::on_off::test::TestOnOffDeviceLogic;
use rs_matter_embassy::matter::dm::clusters::on_off::{self, OnOffHooks};
use rs_matter_embassy::matter::dm::devices::test::{TEST_DEV_ATT, TEST_DEV_COMM, TEST_DEV_DET};
use rs_matter_embassy::matter::dm::devices::DEV_TYPE_ON_OFF_LIGHT;
use rs_matter_embassy::matter::dm::{Async, Dataver, EmptyHandler, Endpoint, EpClMatcher, Node};
use rs_matter_embassy::matter::utils::init::InitMaybeUninit;
use rs_matter_embassy::matter::{clusters, devices, BasicCommData};
use rs_matter_embassy::rand::esp::{esp_init_rand, esp_rand};
use rs_matter_embassy::stack::persist::DummyKvBlobStore;
use rs_matter_embassy::stack::rand::RngCore;
use rs_matter_embassy::wireless::esp::EspThreadDriver;
use rs_matter_embassy::wireless::{EmbassyThread, EmbassyThreadMatterStack};

use tinyrlibc as _;

extern crate alloc;

const BUMP_SIZE: usize = 25000;

#[cfg(feature = "esp32")]
const HEAP_SIZE: usize = 40 * 1024; // 40KB for ESP32, which has a disjoint heap
#[cfg(any(feature = "esp32c3", feature = "esp32h2"))]
const HEAP_SIZE: usize = 160 * 1024;
#[cfg(not(any(feature = "esp32", feature = "esp32c3", feature = "esp32h2")))]
const HEAP_SIZE: usize = 186 * 1024;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_s: Spawner) {
    esp_println::logger::init_logger(log::LevelFilter::Debug);

    info!("Starting...");

    // Heap strictly necessary only for Wifi+BLE and for the only Matter dependency which needs (~4KB) alloc - `x509`
    // However since `esp32` specifically has a disjoint heap which causes bss size troubles, it is easier
    // to allocate the statics once from heap as well
    heap_allocator!(size: HEAP_SIZE);
    #[cfg(feature = "esp32")]
    heap_allocator!(#[link_section = ".dram2_uninit"] size: 96 * 1024);

    // == Step 1: ==
    // Necessary `esp-hal` initialization boilerplate

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut rng = esp_hal::rng::Rng::new();

    // Use a random/unique Matter discriminator for this session,
    // in case there are left-overs from our previous registrations in Thread SRP
    let discriminator = (rng.next_u32() & 0xfff) as u16;

    // TODO
    let mut ieee_eui64 = [0; 8];
    rng.fill_bytes(&mut ieee_eui64);

    // To erase generics, `Matter` takes a rand `fn` rather than a trait or a closure,
    // so we need to initialize the global `rand` fn once
    esp_init_rand(rng);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT)
            .software_interrupt0,
    );

    let init = esp_radio::init().unwrap();

    // == Step 2: ==
    // Allocate the Matter stack.
    // For MCUs, it is best to allocate it statically, so as to avoid program stack blowups (its memory footprint is ~ 35 to 50KB).
    // It is also (currently) a mandatory requirement when the wireless stack variation is used.
    let stack =
        &*Box::leak(Box::new_uninit()).init_with(EmbassyThreadMatterStack::<BUMP_SIZE, ()>::init(
            &TEST_BASIC_INFO,
            BasicCommData {
                password: TEST_DEV_COMM.password,
                discriminator,
            },
            &TEST_DEV_ATT,
            epoch,
            esp_rand,
        ));

    // == Step 4: ==
    // Our "light" on-off cluster.
    // It will toggle the light state every 5 seconds
    let on_off = on_off::OnOffHandler::new_standalone(
        Dataver::new_rand(stack.matter().rand()),
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
            Async(desc::DescHandler::new(Dataver::new_rand(stack.matter().rand())).adapt()),
        );

    // == Step 5: ==
    // Run the Matter stack with our handler
    // Using `pin!` is completely optional, but reduces the size of the final future
    //
    // This step can be repeated in that the stack can be stopped and started multiple times, as needed.
    let store = stack.create_shared_store(DummyKvBlobStore);
    let matter = pin!(stack.run(
        // The Matter stack needs to instantiate an `openthread` Radio
        EmbassyThread::new(
            EspThreadDriver::new(&init, peripherals.IEEE802154, peripherals.BT),
            ieee_eui64,
            &store,
            stack,
        ),
        // The Matter stack needs a persister to store its state
        &store,
        // Our `AsyncHandler` + `AsyncMetadata` impl
        (NODE, handler),
        // No user future to run
        (),
    ));

    // Run Matter
    matter.await.unwrap();
}

/// Basic info about our device
/// Both the matter stack as well as out mDNS-to-SRP bridge need this, hence extracted out
const TEST_BASIC_INFO: BasicInfoConfig = BasicInfoConfig {
    sai: Some(500),
    ..TEST_DEV_DET
};

/// Endpoint 0 (the root endpoint) always runs
/// the hidden Matter system clusters, so we pick ID=1
const LIGHT_ENDPOINT_ID: u16 = 1;

/// The Matter Light device Node
const NODE: Node = Node {
    id: 0,
    endpoints: &[
        EmbassyThreadMatterStack::<0, ()>::root_endpoint(),
        Endpoint {
            id: LIGHT_ENDPOINT_ID,
            device_types: devices!(DEV_TYPE_ON_OFF_LIGHT),
            clusters: clusters!(desc::DescHandler::CLUSTER, TestOnOffDeviceLogic::CLUSTER),
        },
    ],
};

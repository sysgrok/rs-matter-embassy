//! An example utilizing the `EmbassyThreadMatterStack` struct.
//!
//! As the name suggests, this Matter stack assembly uses Thread as the main transport,
//! and thus BLE for commissioning, in concurrent commissioning mode.
//!
//! The example implements a fictitious Light device (an On-Off Matter cluster).
#![no_std]
#![no_main]
#![recursion_limit = "256"]

use core::mem::MaybeUninit;
use core::pin::pin;
use core::ptr::addr_of_mut;

use embassy_nrf::peripherals::RNG;
use embassy_nrf::{bind_interrupts, rng};

use embassy_executor::Spawner;

use embedded_alloc::LlffHeap;

use defmt::{info, unwrap};

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
use rs_matter_embassy::rand::nrf::{nrf_init_rand, nrf_rand};
use rs_matter_embassy::stack::persist::DummyKvBlobStore;
use rs_matter_embassy::stack::rand::RngCore;
use rs_matter_embassy::wireless::nrf::{
    NrfMpslThreadDriver, NrfThreadClockInterruptHandler, NrfThreadHighPrioInterruptHandler,
    NrfThreadLowPrioInterruptHandler, NrfThreadRustRadioRunner,
};
use rs_matter_embassy::wireless::{EmbassyThread, EmbassyThreadMatterStack};

use panic_rtt_target as _;

use tinyrlibc as _;

macro_rules! mk_static {
    ($t:ty) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit();
        x
    }};
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write($val);
        x
    }};
}

bind_interrupts!(struct Irqs {
    RNG => rng::InterruptHandler<RNG>;
    EGU0_SWI0 => NrfThreadLowPrioInterruptHandler;
    CLOCK_POWER => NrfThreadClockInterruptHandler;
    RADIO => NrfThreadHighPrioInterruptHandler;
    TIMER0 => NrfThreadHighPrioInterruptHandler;
    RTC0 => NrfThreadHighPrioInterruptHandler;
});

const BUMP_SIZE: usize = 20500;

#[global_allocator]
static HEAP: LlffHeap = LlffHeap::empty();

/// We need a bigger log ring-buffer or else the device QR code printout is half-lost
const LOG_RINGBUF_SIZE: usize = 2048;

#[embassy_executor::main]
async fn main(_s: Spawner) {
    // `rs-matter` uses the `x509` crate which (still) needs a few kilos of heap space
    {
        const HEAP_SIZE: usize = 8192;

        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(addr_of_mut!(HEAP_MEM) as usize, HEAP_SIZE) }
    }

    // == Step 1: ==
    // Necessary `nrf-hal` initialization boilerplate

    rtt_target::rtt_init_defmt!(rtt_target::ChannelMode::NoBlockSkip, LOG_RINGBUF_SIZE);

    info!("Starting...");

    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;

    let p = embassy_nrf::init(config);

    let mut rng = rng::Rng::new_blocking(p.RNG);

    // Use a random/unique Matter discriminator for this session,
    // in case there are left-overs from our previous registrations in Thread SRP
    let discriminator = (rng.next_u32() & 0xfff) as u16;

    // TODO
    let mut ieee_eui64 = [0; 8];
    RngCore::fill_bytes(&mut rng, &mut ieee_eui64);

    // To erase generics, `Matter` takes a rand `fn` rather than a trait or a closure,
    // so we need to initialize the global `rand` fn once
    nrf_init_rand(rng);

    // == Step 2: ==
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
            epoch,
            nrf_rand,
        ),
    );

    let thread_driver = NrfMpslThreadDriver::new(
        p.RADIO,
        p.EGU0,
        p.RTC0,
        p.TIMER0,
        p.TEMP,
        p.PPI_CH17,
        p.PPI_CH18,
        p.PPI_CH19,
        p.PPI_CH20,
        p.PPI_CH21,
        p.PPI_CH22,
        p.PPI_CH23,
        p.PPI_CH24,
        p.PPI_CH25,
        p.PPI_CH26,
        p.PPI_CH27,
        p.PPI_CH28,
        p.PPI_CH29,
        p.PPI_CH30,
        p.PPI_CH31,
        stack.matter().rand(),
        Irqs,
    );

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

    let persist = stack
        .create_persist_with_comm_window(DummyKvBlobStore)
        .await
        .unwrap();

    // == Step 5: ==
    // Run the Matter stack with our handler
    // Using `pin!` is completely optional, but reduces the size of the final future
    //
    // This step can be repeated in that the stack can be stopped and started multiple times, as needed.
    let matter = pin!(stack.run_coex(
        // The Matter stack needs to instantiate `openthread`
        EmbassyThread::new(thread_driver, ieee_eui64, persist.store(), stack),
        // The Matter stack needs a persister to store its state
        &persist,
        // Our `AsyncHandler` + `AsyncMetadata` impl
        (NODE, handler),
        // No user future to run
        (),
    ));

    // Run Matter
    unwrap!(matter.await);
}

/// Basic info about our device
/// Both the matter stack as well as our mDNS-to-SRP bridge need this, hence extracted out
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

#[embassy_executor::task]
async fn run_radio(mut runner: NrfThreadRustRadioRunner<'static, 'static>) -> ! {
    runner.run().await
}

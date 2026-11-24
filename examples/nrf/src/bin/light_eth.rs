//! An example utilizing the `MatterStack` struct directly from `rs-matter-stack`, over Ethernet.
//! As the name suggests, this Matter stack assembly uses Ethernet as the main transport, as well as for commissioning.
//!
//! Notice thart we actually don't use Ethernet for real, as NRFs don't have Ethernet ports out of the box.
//! Instead, we utilize Thread, which - from the POV of Matter - is indistinguishable from Ethernet as long as the Matter
//! stack is not concerned with connecting to the Thread network, managing its credentials etc. and can assume it "pre-exists".
//!
//! The example implements a fictitious Light device (an On-Off Matter cluster).
#![no_std]
#![no_main]
#![recursion_limit = "256"]

use core::mem::MaybeUninit;
use core::pin::pin;
use core::ptr::addr_of_mut;

use embassy_nrf::{bind_interrupts, rng};

use embassy_executor::Spawner;

use embedded_alloc::LlffHeap;

use defmt::{info, unwrap};

use rs_matter_embassy::matter::crypto::{default_crypto, Crypto, Rng};
use rs_matter_embassy::matter::dm::clusters::app::on_off::test::TestOnOffDeviceLogic;
use rs_matter_embassy::matter::dm::clusters::app::on_off::{self, OnOffHooks};
use rs_matter_embassy::matter::dm::clusters::basic_info::BasicInfoConfig;
use rs_matter_embassy::matter::dm::clusters::desc::{self, ClusterHandler as _};
use rs_matter_embassy::matter::dm::devices::test::{
    DAC_PRIVKEY, TEST_BASIC_COMM, TEST_DEV_ATT, TEST_DEV_DET,
};
use rs_matter_embassy::matter::dm::devices::DEV_TYPE_ON_OFF_LIGHT;
use rs_matter_embassy::matter::dm::endpoints::ROOT_ENDPOINT_ID;
use rs_matter_embassy::matter::dm::networks::eth::EthNetwork;
use rs_matter_embassy::matter::dm::{Async, Dataver, EmptyHandler, Endpoint, Node};
use rs_matter_embassy::matter::persist::{DummyKvBlobStore, VENDOR_KEYS_START};
use rs_matter_embassy::matter::utils::futures::{select, Coalesce};
use rs_matter_embassy::matter::utils::init::InitMaybeUninit;
use rs_matter_embassy::matter::{clusters, devices, BasicCommData};
use rs_matter_embassy::ot::openthread::{OpenThread, Radio, RamSettings};
use rs_matter_embassy::ot::{OtMatterResources, OtMdns, OtNetStack, OtNetif};
use rs_matter_embassy::stack::eth::PreexistingEthernet;
use rs_matter_embassy::stack::MatterStack;
use rs_matter_embassy::wireless::nrf::{
    EguInterruptHandler, LpTimerInterruptHandler, NrfIeee802154Peripherals, NrfMpslPeripherals,
    NrfThreadClockInterruptHandler, NrfThreadHighPrioInterruptHandler,
    NrfThreadLowPrioInterruptHandler,
};

use nrf_mpsl::MultiprotocolServiceLayer;

use openthread_driver_nrf::NrfRadio;

use panic_rtt_target as _;

use rs_matter_embassy::rand::reseeding_csprng;
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
const BUMP_SIZE: usize = 15500;

#[global_allocator]
static HEAP: LlffHeap = LlffHeap::empty();

const THREAD_DATASET: &str = env!("THREAD_DATASET");

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

    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;

    let p = embassy_nrf::init(config);

    // Create the crypto provider, using the NRF RNG peripheral (which is a TRNG) as the source of randomness for a reseeding CSPRNG.
    let crypto = default_crypto(
        reseeding_csprng(rng::Rng::new_blocking(p.RNG), 1000).unwrap(),
        DAC_PRIVKEY,
    );

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
    let stack = mk_static!(MatterStack<BUMP_SIZE, EthNetwork>).init_with(MatterStack::init(
        &TEST_BASIC_INFO,
        BasicCommData {
            password: TEST_BASIC_COMM.password,
            discriminator,
        }
        .into(),
        &TEST_DEV_ATT,
    ));

    // The IEEE 802.15.4 radio runs on top of MPSL, clocked off the internal RC oscillator
    let lfclk_cfg = nrf_mpsl::raw::mpsl_clock_lfclk_cfg_t {
        source: nrf_mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: nrf_mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: nrf_mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: nrf_mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: nrf_mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };

    let mpsl = unwrap!(MultiprotocolServiceLayer::new(
        NrfMpslPeripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31),
        Irqs,
        lfclk_cfg,
    ));

    let mut radio = NrfRadio::new(nrf_802154::Radio::new(
        p.RADIO,
        NrfIeee802154Peripherals::new(p.EGU0, p.TIMER2, p.RTC2),
        Irqs,
        &mpsl,
    ));

    let ot_resources = mk_static!(OtMatterResources).init_with(OtMatterResources::init());

    let mut ot_settings = RamSettings::new(&mut ot_resources.settings_buf);
    let mut ot_rng = crypto.rand().unwrap();

    let radio_caps = unwrap!(radio.init().await);

    let ot = unwrap!(OpenThread::new_with_udp_srp(
        ieee_eui64,
        &radio_caps,
        &mut ot_rng,
        &mut ot_settings,
        &mut ot_resources.ot,
        &mut ot_resources.udp,
        &mut ot_resources.srp,
    ));

    // MPSL's low-priority work has to be driven for the radio to work at all
    let mut ot_runner = pin!(async {
        select(mpsl.run(), ot.run(radio)).await;
        #[allow(unreachable_code)]
        Ok(())
    });

    unwrap!(ot.srp_autostart());

    unwrap!(ot.set_active_dataset_tlv_hexstr(THREAD_DATASET));
    unwrap!(ot.enable_ipv6(true));
    unwrap!(ot.enable_thread(true));

    // Our "light" on-off cluster.
    // It will toggle the light state every 5 seconds
    let on_off = on_off::OnOffHandler::new_standalone(
        Dataver::new_rand(&mut weak_rand),
        LIGHT_ENDPOINT_ID,
        VENDOR_KEYS_START + 0x10,
        TestOnOffDeviceLogic::new(true),
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
            Async(MatterStack::<0, EthNetwork>::root_handler(
                stack.matter(),
                &(),
                &mut weak_rand,
            )),
        )
        // Our on-off cluster, on Endpoint 1
        .chain(
            |e, c| e == LIGHT_ENDPOINT_ID && c == TestOnOffDeviceLogic::CLUSTER.id,
            Async(on_off::HandlerAdaptor(&on_off)),
        )
        // Each Endpoint needs a Descriptor cluster too
        // Just use the one that `rs-matter` provides out of the box
        .chain(
            |e, c| e == LIGHT_ENDPOINT_ID && c == desc::DescHandler::CLUSTER.id,
            Async(desc::DescHandler::new(Dataver::new_rand(&mut weak_rand)).adapt()),
        );

    // Create a KV BLOB store and load any previously saved state of `rs-matter`
    // `SeqMapKvBlobStore` saves to a user-supplied NOR Flash region
    // However, for this demo and for simplicity, we use a dummy KV BLOB store that does nothing
    let mut store = DummyKvBlobStore;
    stack.startup(&crypto, &mut store).await.unwrap();

    // Run the Matter stack with our handler
    // Using `pin!` is completely optional, but reduces the size of the final future
    //
    // This step can be repeated in that the stack can be stopped and started multiple times, as needed.
    let mut matter = pin!(stack.run_eth(
        PreexistingEthernet::new(
            // The Matter stack needs to open two UDP sockets
            OtNetStack::new(ot.clone()),
            // The Matter stack needs access to the netif so as to detect network going up/down
            OtNetif::new(ot.clone()),
            // No Ethernet diagnostics, as the netif is not really Ethernet
            &(),
            // The Matter stack needs an mDNS to run
            OtMdns::new(ot.clone(), &mut ot_resources.mdns_buf),
        ),
        // The crypto provider
        &crypto,
        // Our `AsyncHandler` + `AsyncMetadata` impl
        (NODE, handler),
        // The Matter stack needs a blob store to store its state
        stack.matter().kv(store),
        // No user future to run
        (),
    ));

    // Schedule the Matter run & the OpenThread run together
    unwrap!(select(&mut matter, &mut ot_runner).coalesce().await);
}

/// Basic info about our device
/// Both the matter stack as well as our mDNS-to-SRP bridge need this, hence extracted out
const TEST_BASIC_INFO: BasicInfoConfig = BasicInfoConfig {
    sai: Some(1000),
    ..TEST_DEV_DET
};

/// Endpoint 0 (the root endpoint) always runs
/// the hidden Matter system clusters, so we pick ID=1
const LIGHT_ENDPOINT_ID: u16 = 1;

/// The Matter Light device Node
const NODE: Node = Node {
    endpoints: &[
        MatterStack::<0, EthNetwork>::eth_root_endpoint(),
        Endpoint::new(
            LIGHT_ENDPOINT_ID,
            devices!(DEV_TYPE_ON_OFF_LIGHT),
            clusters!(desc::DescHandler::CLUSTER, TestOnOffDeviceLogic::CLUSTER),
        ),
    ],
};

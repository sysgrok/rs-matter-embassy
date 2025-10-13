//! An example utilizing the `EmbassyWifiMatterStack` struct.
//!
//! As the name suggests, this Matter stack assembly uses Wifi as the main transport,
//! and thus BLE for commissioning.
//!
//! If you want to use Ethernet, utilize `EmbassyEthMatterStack` instead.
//! If you want to use non-concurrent commissioning, utilize `EmbassyWifiNCMatterStack` instead
//! (Note: Alexa does not work (yet) with non-concurrent commissioning.)
//!
//! The example implements a fictitious Light device (an On-Off Matter cluster).
#![no_std]
#![no_main]
#![recursion_limit = "256"]

use core::mem::MaybeUninit;
use core::pin::pin;
use core::ptr::addr_of_mut;

use bt_hci::controller::ExternalController;

use cyw43_pio::PioSpi;

use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Timer};

use embedded_alloc::LlffHeap;

use panic_rtt_target as _;

use defmt::{info, unwrap};

use rs_matter_embassy::enet::net::driver::{Driver as _, HardwareAddress as DriverHardwareAddress};
use rs_matter_embassy::enet::{
    create_link_local_ipv6, multicast_mac_for_link_local_ipv6, MDNS_MULTICAST_MAC_IPV4,
    MDNS_MULTICAST_MAC_IPV6,
};
use rs_matter_embassy::epoch::epoch;
use rs_matter_embassy::matter::dm::clusters::desc::{self, ClusterHandler as _};
use rs_matter_embassy::matter::dm::clusters::on_off::test::TestOnOffDeviceLogic;
use rs_matter_embassy::matter::dm::clusters::on_off::{self, OnOffHooks};
use rs_matter_embassy::matter::dm::devices::test::{TEST_DEV_ATT, TEST_DEV_COMM, TEST_DEV_DET};
use rs_matter_embassy::matter::dm::devices::DEV_TYPE_ON_OFF_LIGHT;
use rs_matter_embassy::matter::dm::{Async, Dataver, EmptyHandler, Endpoint, EpClMatcher, Node};
use rs_matter_embassy::matter::utils::init::InitMaybeUninit;
use rs_matter_embassy::matter::utils::select::Coalesce;
use rs_matter_embassy::matter::{clusters, devices};
use rs_matter_embassy::rand::rp::rp_rand;
use rs_matter_embassy::stack::persist::DummyKvBlobStore;
use rs_matter_embassy::wifi::rp::Cyw43WifiController;
use rs_matter_embassy::wireless::{EmbassyWifi, EmbassyWifiMatterStack, PreexistingWifiDriver};

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
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

const BUMP_SIZE: usize = 15500;

#[global_allocator]
static HEAP: LlffHeap = LlffHeap::empty();

/// We need a bigger log ring-buffer or else the device QR code printout is half-lost
const LOG_RINGBUF_SIZE: usize = 2048;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // `rs-matter` uses the `x509` crate which (still) needs a few kilos of heap space
    {
        const HEAP_SIZE: usize = 8192;

        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(addr_of_mut!(HEAP_MEM) as usize, HEAP_SIZE) }
    }

    // == Step 1: ==
    // Necessary `embassy-rp` and `cyw43` initialization boilerplate

    let p = embassy_rp::init(Default::default());

    rtt_target::rtt_init_defmt!(rtt_target::ChannelMode::NoBlockSkip, LOG_RINGBUF_SIZE);

    info!("Starting...");

    #[cfg(feature = "skip-cyw43-firmware")]
    let (fw, clm, btfw) = (&[], &[], &[]);

    #[cfg(not(feature = "skip-cyw43-firmware"))]
    let (fw, clm, btfw) = {
        let fw = include_bytes!("../../cyw43-firmware/43439A0.bin");
        let clm = include_bytes!("../../cyw43-firmware/43439A0_clm.bin");
        let btfw = include_bytes!("../../cyw43-firmware/43439A0_btfw.bin");
        (fw, clm, btfw)
    };

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        // NOTE: There is a BLE packet corruption bug with yet-unknown reason.
        // Lowering the pio-SPI clock by 8x seems to fix it or at least makes it
        // rare enough so that it does not happen during the BLE commissioning.
        cyw43_pio::DEFAULT_CLOCK_DIVIDER * 8,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    let state = mk_static!(cyw43::State, cyw43::State::new());
    let (net_device, bt_device, mut control, runner) =
        cyw43::new_with_bluetooth(state, pwr, spi, fw, btfw).await;
    unwrap!(spawner.spawn(cyw43_task(runner)));
    control.init(clm).await; // We should have the Wifi MAC address now

    // cyw43 is a bit special in that it needs to have allowlisted all multicast MAC addresses
    // it should listen on. Therefore, add the mDNS ipv4 and ipv6 multicast MACs to the list,
    // as well as the ipv6 neightbour solicitation requests' MAC.
    let DriverHardwareAddress::Ethernet(mac) = net_device.hardware_address() else {
        unreachable!()
    };
    unwrap!(
        control.add_multicast_address(MDNS_MULTICAST_MAC_IPV4).await,
        "Adding multicast addr failed",
    );
    unwrap!(
        control.add_multicast_address(MDNS_MULTICAST_MAC_IPV6).await,
        "Adding multicast addr failed",
    );
    unwrap!(
        control
            .add_multicast_address(multicast_mac_for_link_local_ipv6(&create_link_local_ipv6(
                &mac,
            )))
            .await,
        "Adding multicast addr failed",
    );

    let controller: ExternalController<_, 20> = ExternalController::new(bt_device);

    // == Step 2: ==
    // Statically allocate the Matter stack.
    // For MCUs, it is best to allocate it statically, so as to avoid program stack blowups (its memory footprint is ~ 35 to 50KB).
    // It is also (currently) a mandatory requirement when the wireless stack variation is used.
    let stack = mk_static!(EmbassyWifiMatterStack<BUMP_SIZE, ()>).init_with(
        EmbassyWifiMatterStack::init(&TEST_DEV_DET, TEST_DEV_COMM, &TEST_DEV_ATT, epoch, rp_rand),
    );

    // == Step 3: ==
    // Our "light" on-off cluster.
    // Can be anything implementing `rs_matter::dm::AsyncHandler`
    let on_off = on_off::OnOffHandler::new_standalone(
        Dataver::new_rand(stack.matter().rand()),
        1,
        TestOnOffDeviceLogic::new(),
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

    // == Step 4: ==
    // Run the Matter stack with our handler
    // Using `pin!` is completely optional, but reduces the size of the final future
    //
    // This step can be repeated in that the stack can be stopped and started multiple times, as needed.
    let store = stack.create_shared_store(DummyKvBlobStore);
    let mut matter = pin!(stack.run(
        // The Matter stack needs Wifi
        EmbassyWifi::new(
            PreexistingWifiDriver::new(
                net_device,
                Cyw43WifiController::<NoopRawMutex>::new(control),
                controller
            ),
            stack
        ),
        // The Matter stack needs a persister to store its state
        // `EmbassyPersist`+`EmbassyKvBlobStore` saves to a user-supplied NOR Flash region
        // However, for this demo and for simplicity, we use a dummy persister that does nothing
        &store,
        // Our `AsyncHandler` + `AsyncMetadata` impl
        (NODE, handler),
        // No user future to run
        (),
    ));

    // Just for demoing purposes:
    //
    // Run a sample loop that simulates state changes triggered by the HAL
    // Changes will be properly communicated to the Matter controllers
    // (i.e. Google Home, Alexa) and other Matter devices thanks to subscriptions
    let mut device = pin!(async {
        loop {
            // Simulate user toggling the light with a physical switch every 5 seconds
            Timer::after(Duration::from_secs(5)).await;

            // Toggle
            on_off.set_on_off(!on_off.on_off());

            // Let the Matter stack know that we have changed
            // the state of our Light device
            stack.notify_cluster_changed(1, TestOnOffDeviceLogic::CLUSTER.id);

            info!("Light toggled");
        }
    });

    // Schedule the Matter run & the device loop together
    unwrap!(select(&mut matter, &mut device).coalesce().await);
}

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

/// Endpoint 0 (the root endpoint) always runs
/// the hidden Matter system clusters, so we pick ID=1
const LIGHT_ENDPOINT_ID: u16 = 1;

/// The Matter Light device Node
const NODE: Node = Node {
    id: 0,
    endpoints: &[
        EmbassyWifiMatterStack::<0, ()>::root_endpoint(),
        Endpoint {
            id: LIGHT_ENDPOINT_ID,
            device_types: devices!(DEV_TYPE_ON_OFF_LIGHT),
            clusters: clusters!(desc::DescHandler::CLUSTER, TestOnOffDeviceLogic::CLUSTER),
        },
    ],
};

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

use embassy_nrf::bind_interrupts;

use embassy_executor::Spawner;

use embedded_alloc::LlffHeap;

use defmt::{info, unwrap};

use rs_matter_embassy::matter::crypto::{default_crypto, Crypto, RngCore};
use rs_matter_embassy::matter::dm::clusters::app::on_off::test::TestOnOffDeviceLogic;
use rs_matter_embassy::matter::dm::clusters::app::on_off::{self, OnOffHooks};
use rs_matter_embassy::matter::dm::clusters::basic_info::BasicInfoConfig;
use rs_matter_embassy::matter::dm::clusters::desc::{self, ClusterHandler as _};
use rs_matter_embassy::matter::dm::devices::test::{
    DAC_PRIVKEY, TEST_DEV_ATT, TEST_DEV_COMM, TEST_DEV_DET,
};
use rs_matter_embassy::matter::dm::devices::DEV_TYPE_ON_OFF_LIGHT;
use rs_matter_embassy::matter::dm::{Async, Dataver, EmptyHandler, Endpoint, EpClMatcher, Node};
use rs_matter_embassy::matter::persist::DummyKvBlobStore;
use rs_matter_embassy::matter::utils::init::InitMaybeUninit;
use rs_matter_embassy::matter::{clusters, devices, BasicCommData};
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

    let thread_driver = NrfThreadMpslRadioDriver::new(
        p.RADIO,
        mpsl_p,
        sdc_p,
        ieee802154_p,
        crypto.rand().unwrap(),
        Irqs,
    );

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
    let mut store = DummyKvBlobStore;
    stack.startup(&crypto, &mut store).await.unwrap();

    let kv = stack.matter().kv(store);

    // Run the Matter stack with our handler
    // Using `pin!` is completely optional, but reduces the size of the final future
    //
    // This step can be repeated in that the stack can be stopped and started multiple times, as needed.
    let matter = pin!(stack.run_coex(
        // The Matter stack needs to instantiate `openthread`
        EmbassyThread::new(
            thread_driver,
            crypto.rand().unwrap(),
            ieee_eui64,
            &kv,
            stack,
            true, // Use a random BLE address
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
    endpoints: &[
        EmbassyThreadMatterStack::<0, ()>::root_endpoint(),
        Endpoint::new(
            LIGHT_ENDPOINT_ID,
            devices!(DEV_TYPE_ON_OFF_LIGHT),
            clusters!(desc::DescHandler::CLUSTER, TestOnOffDeviceLogic::CLUSTER),
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

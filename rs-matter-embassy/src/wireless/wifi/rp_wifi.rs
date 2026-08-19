use core::cell::{Cell, RefCell};
use core::convert::Infallible;
use core::future::{poll_fn, Future};
use core::pin::{pin, Pin};
use core::ptr::addr_of_mut;
use core::task::{Context, Poll};

use bt_hci::cmd::{AsyncCmd, SyncCmd};
use bt_hci::controller::{ControllerCmdAsync, ControllerCmdSync, ExternalController};
use bt_hci::data::{AclPacket, IsoPacket, SyncPacket};
use bt_hci::ControllerToHostPacket;

use embedded_io::ErrorType;

use cyw43::{Aligned, Control, A4};
use cyw43_pio::PioSpi;

use embassy_futures::select::select;
use embassy_futures::select::Either::{First, Second};

use embassy_net_driver_channel::Device;

use embassy_rp::dma::{self, Channel};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::interrupt::typelevel::{Binding, DMA_IRQ_0, PIO0_IRQ_0};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, PIN_23, PIN_24, PIN_25, PIN_29, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::Peri;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use crate::enet::net::driver::{Driver as _, HardwareAddress as DriverHardwareAddress};
use crate::enet::{
    create_link_local_ipv6, multicast_mac_for_link_local_ipv6, MDNS_MULTICAST_MAC_IPV4,
    MDNS_MULTICAST_MAC_IPV6,
};
use crate::matter::error::Error;
use crate::matter::utils::sync::blocking::Mutex;
use crate::wifi::rp::Cyw43WifiController;

#[derive(Copy, Clone)]
struct Cyw43PioInterrupts;

unsafe impl Binding<PIO0_IRQ_0, InterruptHandler<PIO0>> for Cyw43PioInterrupts {}
unsafe impl Binding<DMA_IRQ_0, dma::InterruptHandler<DMA_CH0>> for Cyw43PioInterrupts {}
unsafe impl Binding<DMA_IRQ_0, dma::InterruptHandler<DMA_CH1>> for Cyw43PioInterrupts {}

/// The `cyw43` driver state holds the network RX/TX packet buffers, as well as the
/// BLE HCI packet buffers, and is therefore quite large (~21KB).
///
/// Since the `run*` methods below are driven by futures which - inside the Matter stack -
/// are allocated from a small bump allocator, keeping the state as a local of those futures
/// would inflate the bump memory requirements by its full size. Hence the state lives in a
/// `static` instead, and is handed out - exclusively - via this guard.
static CYW43_STATE_TAKEN: Mutex<Cell<bool>, CriticalSectionRawMutex> = Mutex::new(Cell::new(false));
static mut CYW43_STATE: cyw43::State = cyw43::State::new();

/// An exclusive borrow of the singleton `cyw43::State` instance.
///
/// Released on drop, so that the Wifi driver can be stopped and started again.
struct Cyw43State(&'static mut cyw43::State);

impl Cyw43State {
    /// Take the singleton `cyw43::State` instance.
    ///
    /// Panics if the state is already taken.
    fn take() -> Self {
        CYW43_STATE_TAKEN.lock(|taken| {
            if taken.replace(true) {
                panic!("The cyw43 driver state is already in use");
            }
        });

        // SAFETY: The flag above ensures that at most one `Cyw43State` instance exists
        // at any point in time, and the `Drop` impl below puts the flag back, which means
        // there is no other live borrow of the state at this point.
        //
        // Re-initializing (rather than just handing out) the state matters because - unlike
        // the fresh `State::new()` the `run*` methods used to keep as a future local - the
        // static is reused when the driver is stopped and started again, and `cyw43::new*`
        // only re-initializes some of its fields.
        unsafe {
            addr_of_mut!(CYW43_STATE).write(cyw43::State::new());

            Self(&mut *addr_of_mut!(CYW43_STATE))
        }
    }
}

impl Drop for Cyw43State {
    fn drop(&mut self) {
        CYW43_STATE_TAKEN.lock(|taken| taken.set(false));
    }
}

/// Re-shape a never-returning future into one with a *nameable* `Infallible` output, so that it
/// can be type-erased as `dyn Future` (`cyw43::Runner::run` returns `!`, which cannot be named).
async fn never_ending<F>(fut: F) -> Infallible
where
    F: Future,
{
    fut.await;

    // Unreachable for `cyw43::Runner::run`; parking rather than returning keeps the
    // `Infallible` promise honest for any other future that might be passed in.
    core::future::pending().await
}

/// The `cyw43` runner future, borrowed by everything that needs it polled.
///
/// `cyw43::Runner::run` is not just the Wifi worker: it owns the PIO-SPI bus, and therefore it is
/// also the *BLE HCI transport* - `cyw43::bluetooth::BtDriver`, which the `bt-hci`
/// `ExternalController` sits on, is a mere pair of channels whose other ends the runner services.
///
/// That distinction matters for the NimBLE BLE host. Being a C stack, it services an HCI
/// command-ack round-trip by blocking the calling task (`nimble-rs` spends the wait pumping its own
/// HCI bridge and then parking), which withholds the executor for the duration. Were the runner
/// reachable only as another arm of the driver's `select` below, it would not be polled across such
/// a wait - so the command would never reach the chip, the ack would never arrive, and the host
/// would hang on its very first HCI `Reset`. Nothing rescues it either: with the `bluetooth`
/// feature the runner busy-polls the chip rather than waiting on an interrupt.
///
/// Hence the runner is shared, and [`Cyw43Controller`] polls it while awaiting any HCI operation.
/// That puts it squarely inside the "drive the bridge, then park on its I/O" loop NimBLE already
/// runs: the runner registers the parker's waker with its DMA transfers, so the park wakes on
/// their completion. The `trouble` host never blocks this way, but sharing the runner is correct
/// for it too, so both backends take the same path.
struct SharedRunner<'a>(RefCell<Pin<&'a mut (dyn Future<Output = Infallible> + 'a)>>);

impl<'a> SharedRunner<'a> {
    /// Share the given (pinned) `cyw43` runner future.
    fn new(runner: Pin<&'a mut (dyn Future<Output = Infallible> + 'a)>) -> Self {
        Self(RefCell::new(runner))
    }

    /// Poll the runner once, unless it is already being polled further up the stack.
    fn poll(&self, cx: &mut Context<'_>) -> Poll<Infallible> {
        let Ok(mut runner) = self.0.try_borrow_mut() else {
            // Already being polled further up the stack, and that poll registers the waker.
            return Poll::Pending;
        };

        runner.as_mut().poll(cx)
    }
}

/// A `bt-hci` controller that drives the [`SharedRunner`] - i.e. the transport it is speaking
/// through - for as long as it is awaiting an HCI operation.
///
/// Necessary because on this chip the transport is a separate future rather than something the
/// controller's own I/O drives; see [`SharedRunner`] for why that is fatal without this wrapper.
struct Cyw43Controller<'a, 'r, C> {
    ctl: C,
    runner: &'a SharedRunner<'r>,
}

impl<'a, 'r, C> Cyw43Controller<'a, 'r, C> {
    /// Create a new instance.
    const fn new(ctl: C, runner: &'a SharedRunner<'r>) -> Self {
        Self { ctl, runner }
    }

    /// Await `op`, polling the runner alongside it.
    async fn with<T>(&self, op: impl Future<Output = T>) -> T {
        match select(op, poll_fn(|cx| self.runner.poll(cx))).await {
            First(result) => result,
            Second(never) => match never {},
        }
    }
}

impl<C> ErrorType for Cyw43Controller<'_, '_, C>
where
    C: ErrorType,
{
    type Error = C::Error;
}

impl<C> bt_hci::controller::Controller for Cyw43Controller<'_, '_, C>
where
    C: bt_hci::controller::Controller,
{
    fn write_acl_data(&self, packet: &AclPacket) -> impl Future<Output = Result<(), Self::Error>> {
        self.with(self.ctl.write_acl_data(packet))
    }

    fn write_sync_data(
        &self,
        packet: &SyncPacket,
    ) -> impl Future<Output = Result<(), Self::Error>> {
        self.with(self.ctl.write_sync_data(packet))
    }

    fn write_iso_data(&self, packet: &IsoPacket) -> impl Future<Output = Result<(), Self::Error>> {
        self.with(self.ctl.write_iso_data(packet))
    }

    fn read<'a>(
        &self,
        buf: &'a mut [u8],
    ) -> impl Future<Output = Result<ControllerToHostPacket<'a>, Self::Error>> {
        self.with(self.ctl.read(buf))
    }
}

impl<C, Q> ControllerCmdSync<Q> for Cyw43Controller<'_, '_, C>
where
    C: ControllerCmdSync<Q>,
    Q: SyncCmd + ?Sized,
{
    fn exec(
        &self,
        cmd: &Q,
    ) -> impl Future<Output = Result<Q::Return, bt_hci::cmd::Error<Self::Error>>> {
        self.with(self.ctl.exec(cmd))
    }
}

impl<C, Q> ControllerCmdAsync<Q> for Cyw43Controller<'_, '_, C>
where
    C: ControllerCmdAsync<Q>,
    Q: AsyncCmd + ?Sized,
{
    fn exec(&self, cmd: &Q) -> impl Future<Output = Result<(), bt_hci::cmd::Error<Self::Error>>> {
        self.with(self.ctl.exec(cmd))
    }
}

/// A `WifiDriver` implementation for the ESP32 family of chips.
pub struct RpWifiDriver<'d> {
    pwr: Peri<'d, PIN_23>,
    cs: Peri<'d, PIN_25>,
    dio: Peri<'d, PIN_24>,
    clk: Peri<'d, PIN_29>,
    dma0: Peri<'d, DMA_CH0>,
    dma1: Peri<'d, DMA_CH1>,
    pio: Peri<'d, PIO0>,
    fmw: Option<&'d Aligned<A4, [u8]>>,
    fmw_clm: Option<&'d Aligned<A4, [u8]>>,
    fmw_bt: Option<&'d Aligned<A4, [u8]>>,
    nvram: Option<&'d Aligned<A4, [u8]>>,
}

impl<'d> RpWifiDriver<'d> {
    /// Create a new instance of the `Esp32WifiDriver` type.
    ///
    /// # Arguments
    /// - `pwr` - The power control pin for the cyw chip.
    /// - `cs` - The chip select pin for the SPI interface of the cyw chip.
    /// - `dio` - The data input/output pin for the SPI interface of the cyw chip.
    /// - `clk` - The clock pin for the SPI interface of the cyw chip.
    /// - `dma` - The DMA channel for the cyw chip.
    /// - `pio` - The PIO instance for the cyw chip.
    /// - `irq` - The interrupt handler for the PIO instance.
    /// - `fmw` - The optional firmware binary for the cyw chip.
    /// - `fmw_clm` - The optional CLM firmware binary for the cyw chip.
    /// - `fmw_bt` - The optional Bluetooth firmware binary for the cyw chip.
    /// - `nvram` - The optional NVRAM binary for the cyw chip.
    pub fn new<I>(
        pwr: Peri<'d, PIN_23>,
        cs: Peri<'d, PIN_25>,
        dio: Peri<'d, PIN_24>,
        clk: Peri<'d, PIN_29>,
        dma0: Peri<'d, DMA_CH0>,
        dma1: Peri<'d, DMA_CH1>,
        pio: Peri<'d, PIO0>,
        _irq: I,
        fmw: Option<&'d Aligned<A4, [u8]>>,
        fmw_clm: Option<&'d Aligned<A4, [u8]>>,
        fmw_bt: Option<&'d Aligned<A4, [u8]>>,
        nvram: Option<&'d Aligned<A4, [u8]>>,
    ) -> Self
    where
        I: Binding<PIO0_IRQ_0, InterruptHandler<PIO0>> + 'd,
        I: Binding<DMA_IRQ_0, dma::InterruptHandler<DMA_CH0>> + 'd,
        I: Binding<DMA_IRQ_0, dma::InterruptHandler<DMA_CH1>> + 'd,
    {
        Self {
            pwr,
            cs,
            dio,
            clk,
            dma0,
            dma1,
            pio,
            fmw,
            fmw_clm,
            fmw_bt,
            nvram,
        }
    }

    async fn init_net_controller<const N: usize>(
        net_device: &mut Device<'_, N>,
        net_controller: &mut Control<'_>,
        fmw_clm: Option<&Aligned<A4, [u8]>>,
    ) {
        net_controller.init(fmw_clm.unwrap_or(&Aligned([]))).await;

        // cyw43 is a bit special in that it needs to have allowlisted all multicast MAC addresses
        // it should listen on. Therefore, add the mDNS ipv4 and ipv6 multicast MACs to the list,
        // as well as the ipv6 neightbour solicitation requests' MAC.
        let DriverHardwareAddress::Ethernet(mac) = net_device.hardware_address() else {
            unreachable!()
        };
        unwrap!(
            net_controller
                .add_multicast_address(MDNS_MULTICAST_MAC_IPV4)
                .await,
            "Adding multicast addr failed",
        );
        unwrap!(
            net_controller
                .add_multicast_address(MDNS_MULTICAST_MAC_IPV6)
                .await,
            "Adding multicast addr failed",
        );
        unwrap!(
            net_controller
                .add_multicast_address(multicast_mac_for_link_local_ipv6(&create_link_local_ipv6(
                    &mac,
                )))
                .await,
            "Adding multicast addr failed",
        );
    }
}

impl super::WifiDriver for RpWifiDriver<'_> {
    type NetCtl<'a>
        = crate::wifi::rp::Cyw43WifiController<'a>
    where
        Self: 'a;

    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: super::WifiDriverTask,
    {
        let state = Cyw43State::take();
        let fmw_clm = self.fmw_clm;

        let pwr = Output::new(self.pwr.reborrow(), Level::Low);
        let cs = Output::new(self.cs.reborrow(), Level::High);
        let mut pio = Pio::new(self.pio.reborrow(), Cyw43PioInterrupts);
        let dma0 = Channel::new(self.dma0.reborrow(), Cyw43PioInterrupts);
        let dma1 = Channel::new(self.dma1.reborrow(), Cyw43PioInterrupts);

        let spi = PioSpi::new(
            &mut pio.common,
            pio.sm0,
            // NOTE: There is a BLE packet corruption bug with yet-unknown reason.
            // Lowering the pio-SPI clock by 8x seems to fix it or at least makes it
            // rare enough so that it does not happen during the BLE commissioning.
            cyw43_pio::DEFAULT_CLOCK_DIVIDER * 8,
            pio.irq0,
            cs,
            self.dio.reborrow(),
            self.clk.reborrow(),
            dma0,
            dma1,
        );

        let (mut net_device, mut net_controller, runner) = cyw43::new(
            &mut *state.0,
            pwr,
            spi,
            self.fmw.unwrap_or(&Aligned([])),
            self.nvram.unwrap_or(&Aligned([])),
        )
        .await;

        // NOTE: `Control` talks to the chip through the ioctl channel serviced by the runner,
        // so the runner has to be polled concurrently with the controller initialization,
        // or else the initialization would hang forever.
        let mut runner = pin!(runner.run());
        let mut task = pin!(async {
            Self::init_net_controller(&mut net_device, &mut net_controller, fmw_clm).await;

            task.run(net_device, Cyw43WifiController::new(net_controller))
                .await
        });

        match select(&mut runner, &mut task).await {
            First(_) => Ok(()),
            Second(r) => r,
        }
    }
}

impl super::WifiCoexDriver for RpWifiDriver<'_> {
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: super::WifiCoexDriverTask,
    {
        let state = Cyw43State::take();
        let fmw_clm = self.fmw_clm;

        let pwr = Output::new(self.pwr.reborrow(), Level::Low);
        let cs = Output::new(self.cs.reborrow(), Level::High);
        let mut pio = Pio::new(self.pio.reborrow(), Cyw43PioInterrupts);
        let dma0 = Channel::new(self.dma0.reborrow(), Cyw43PioInterrupts);
        let dma1 = Channel::new(self.dma1.reborrow(), Cyw43PioInterrupts);

        let spi = PioSpi::new(
            &mut pio.common,
            pio.sm0,
            // NOTE: There is a BLE packet corruption bug with yet-unknown reason.
            // Lowering the pio-SPI clock by 8x seems to fix it or at least makes it
            // rare enough so that it does not happen during the BLE commissioning.
            cyw43_pio::DEFAULT_CLOCK_DIVIDER * 8,
            pio.irq0,
            cs,
            self.dio.reborrow(),
            self.clk.reborrow(),
            dma0,
            dma1,
        );

        let (mut net_device, bt_device, mut net_controller, runner) = cyw43::new_with_bluetooth(
            &mut *state.0,
            pwr,
            spi,
            self.fmw.unwrap_or(&Aligned([])),
            self.fmw_bt.unwrap_or(&Aligned([])),
            self.nvram.unwrap_or(&Aligned([])),
        )
        .await;

        // NOTE: `Control` talks to the chip through the ioctl channel serviced by the runner,
        // so the runner has to be polled concurrently with the controller initialization,
        // or else the initialization would hang forever.
        let mut runner_fut = pin!(never_ending(runner.run()));
        let runner = SharedRunner::new(runner_fut.as_mut());

        let mut task = pin!(async {
            Self::init_net_controller(&mut net_device, &mut net_controller, fmw_clm).await;

            task.run(
                net_device,
                Cyw43WifiController::new(net_controller),
                Cyw43Controller::new(ExternalController::<_, 20>::new(bt_device), &runner),
            )
            .await
        });

        // The runner arm goes last so that it re-registers the executor's waker with the runner's
        // I/O after every poll of the task - repairing the registration whenever a NimBLE
        // pump-while-pending wait has polled the runner with its parker waker instead.
        let mut runner_arm = pin!(poll_fn(|cx| runner.poll(cx)));

        match select(&mut task, &mut runner_arm).await {
            First(r) => r,
            Second(never) => match never {},
        }
    }
}

impl super::BleDriver for RpWifiDriver<'_> {
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: super::BleDriverTask,
    {
        let state = Cyw43State::take();

        let pwr = Output::new(self.pwr.reborrow(), Level::Low);
        let cs = Output::new(self.cs.reborrow(), Level::High);
        let mut pio = Pio::new(self.pio.reborrow(), Cyw43PioInterrupts);
        let dma0 = Channel::new(self.dma0.reborrow(), Cyw43PioInterrupts);
        let dma1 = Channel::new(self.dma1.reborrow(), Cyw43PioInterrupts);

        let spi = PioSpi::new(
            &mut pio.common,
            pio.sm0,
            // NOTE: There is a BLE packet corruption bug with yet-unknown reason.
            // Lowering the pio-SPI clock by 8x seems to fix it or at least makes it
            // rare enough so that it does not happen during the BLE commissioning.
            cyw43_pio::DEFAULT_CLOCK_DIVIDER * 8,
            pio.irq0,
            cs,
            self.dio.reborrow(),
            self.clk.reborrow(),
            dma0,
            dma1,
        );

        let (_net_device, bt_device, _net_controller, runner) = cyw43::new_with_bluetooth(
            &mut *state.0,
            pwr,
            spi,
            self.fmw.unwrap_or(&Aligned([])),
            self.fmw_bt.unwrap_or(&Aligned([])),
            self.nvram.unwrap_or(&Aligned([])),
        )
        .await;

        //Self::init_net_controller(&mut net_device, &mut net_controller, self.fmw_clm).await;

        let mut runner_fut = pin!(never_ending(runner.run()));
        let runner = SharedRunner::new(runner_fut.as_mut());

        let mut task = pin!(task.run(Cyw43Controller::new(
            ExternalController::<_, 20>::new(bt_device),
            &runner,
        )));

        // The runner arm goes last, for the reason given in the `WifiCoexDriver` impl above.
        let mut runner_arm = pin!(poll_fn(|cx| runner.poll(cx)));

        match select(&mut task, &mut runner_arm).await {
            First(r) => r,
            Second(never) => match never {},
        }
    }
}

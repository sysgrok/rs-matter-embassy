use core::pin::pin;

use bt_hci::controller::ExternalController;

use cyw43::Control;
use cyw43_pio::PioSpi;
use embassy_futures::select::select;
use embassy_futures::select::Either::{First, Second};
use embassy_net_driver_channel::Device;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::interrupt::typelevel::{Binding, PIO0_IRQ_0};
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_24, PIN_25, PIN_29, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::Peri;

use embassy_sync::blocking_mutex::raw::NoopRawMutex;

use crate::enet::net::driver::{Driver as _, HardwareAddress as DriverHardwareAddress};
use crate::enet::{
    create_link_local_ipv6, multicast_mac_for_link_local_ipv6, MDNS_MULTICAST_MAC_IPV4,
    MDNS_MULTICAST_MAC_IPV6,
};
use crate::matter::error::Error;
use crate::wifi::rp::Cyw43WifiController;

struct Cyw43PioInterrupts;

unsafe impl Binding<PIO0_IRQ_0, InterruptHandler<PIO0>> for Cyw43PioInterrupts {}

/// A `WifiDriver` implementation for the ESP32 family of chips.
pub struct RpWifiDriver<'d> {
    pwr: Peri<'d, PIN_23>,
    cs: Peri<'d, PIN_25>,
    dio: Peri<'d, PIN_24>,
    clk: Peri<'d, PIN_29>,
    dma: Peri<'d, DMA_CH0>,
    pio: Peri<'d, PIO0>,
    fmw: Option<&'d [u8]>,
    fmw_clm: Option<&'d [u8]>,
    fmw_bt: Option<&'d [u8]>,
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
    pub fn new<I>(
        pwr: Peri<'d, PIN_23>,
        cs: Peri<'d, PIN_25>,
        dio: Peri<'d, PIN_24>,
        clk: Peri<'d, PIN_29>,
        dma: Peri<'d, DMA_CH0>,
        pio: Peri<'d, PIO0>,
        _irq: I,
        fmw: Option<&'d [u8]>,
        fmw_clm: Option<&'d [u8]>,
        fmw_bt: Option<&'d [u8]>,
    ) -> Self
    where
        I: Binding<PIO0_IRQ_0, InterruptHandler<PIO0>>,
    {
        Self {
            pwr,
            cs,
            dio,
            clk,
            dma,
            pio,
            fmw,
            fmw_clm,
            fmw_bt,
        }
    }

    async fn init_net_controller<const N: usize>(
        net_device: &mut Device<'_, N>,
        net_controller: &mut Control<'_>,
        fmw_clm: Option<&[u8]>,
    ) {
        net_controller.init(fmw_clm.unwrap_or(&[])).await;

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
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: super::WifiDriverTask,
    {
        let mut state = cyw43::State::new();

        let pwr = Output::new(self.pwr.reborrow(), Level::Low);
        let cs = Output::new(self.cs.reborrow(), Level::High);
        let mut pio = Pio::new(self.pio.reborrow(), Cyw43PioInterrupts);
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
            self.dma.reborrow(),
        );

        let (mut net_device, mut net_controller, runner) =
            cyw43::new(&mut state, pwr, spi, self.fmw.unwrap_or(&[])).await;

        Self::init_net_controller(&mut net_device, &mut net_controller, self.fmw_clm).await;

        let mut runner = pin!(runner.run());
        let mut task = pin!(task.run(
            net_device,
            Cyw43WifiController::<NoopRawMutex>::new(net_controller),
        ));

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
        let mut state = cyw43::State::new();

        let pwr = Output::new(self.pwr.reborrow(), Level::Low);
        let cs = Output::new(self.cs.reborrow(), Level::High);
        let mut pio = Pio::new(self.pio.reborrow(), Cyw43PioInterrupts);
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
            self.dma.reborrow(),
        );

        let (mut net_device, bt_device, mut net_controller, runner) = cyw43::new_with_bluetooth(
            &mut state,
            pwr,
            spi,
            self.fmw.unwrap_or(&[]),
            self.fmw_bt.unwrap_or(&[]),
        )
        .await;

        Self::init_net_controller(&mut net_device, &mut net_controller, self.fmw_clm).await;

        let mut runner = pin!(runner.run());
        let mut task = pin!(task.run(
            net_device,
            Cyw43WifiController::<NoopRawMutex>::new(net_controller),
            ExternalController::<_, 20>::new(bt_device),
        ));

        match select(&mut runner, &mut task).await {
            First(_) => Ok(()),
            Second(r) => r,
        }
    }
}

impl super::BleDriver for RpWifiDriver<'_> {
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: super::BleDriverTask,
    {
        let mut state = cyw43::State::new();

        let pwr = Output::new(self.pwr.reborrow(), Level::Low);
        let cs = Output::new(self.cs.reborrow(), Level::High);
        let mut pio = Pio::new(self.pio.reborrow(), Cyw43PioInterrupts);
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
            self.dma.reborrow(),
        );

        let (_net_device, bt_device, _net_controller, runner) = cyw43::new_with_bluetooth(
            &mut state,
            pwr,
            spi,
            self.fmw.unwrap_or(&[]),
            self.fmw_bt.unwrap_or(&[]),
        )
        .await;

        //Self::init_net_controller(&mut net_device, &mut net_controller, self.fmw_clm).await;

        let mut runner = pin!(runner.run());
        let mut task = pin!(task.run(ExternalController::<_, 20>::new(bt_device),));

        match select(&mut runner, &mut task).await {
            First(_) => Ok(()),
            Second(r) => r,
        }
    }
}

// BLE: `TroubleBtpGattPeripheral` - an implementation of the `GattPeripheral` trait from `rs-matter`.

#![allow(clippy::useless_conversion)] // https://github.com/embassy-rs/trouble/issues/248

use core::future::Future;
use core::mem::MaybeUninit;

use bt_hci::cmd::{AsyncCmd, SyncCmd};
use bt_hci::controller::{ControllerCmdAsync, ControllerCmdSync};
use bt_hci::data::{AclPacket, IsoPacket, SyncPacket};
use bt_hci::ControllerToHostPacket;

use embassy_futures::join::join;
use embassy_futures::select::select;
use embassy_sync::blocking_mutex::raw::RawMutex;

use embedded_io::ErrorType;

use rs_matter_stack::matter::crypto::RngCore;
use rs_matter_stack::matter::error::{Error, ErrorCode};
use rs_matter_stack::matter::transport::network::btp::{
    AdvData, GattPeripheral, GattPeripheralEvent, C1_CHARACTERISTIC_UUID, C2_CHARACTERISTIC_UUID,
    MATTER_BLE_SERVICE_UUID16,
};
use rs_matter_stack::matter::transport::network::BtAddr;
use rs_matter_stack::matter::utils::init::{init, Init};
use rs_matter_stack::matter::utils::storage::Vec;
use rs_matter_stack::matter::utils::sync::IfMutex;

use trouble_host::att::{AttCfm, AttClient, AttReq, AttRsp, AttUns};
use trouble_host::prelude::*;
use trouble_host::{self, BleHostError, Controller, HostResources};

use crate::fmt::Bytes;

const MAX_CONNECTIONS: usize = 1;
pub(crate) const MAX_MTU_SIZE: usize = DefaultPacketPool::MTU;
const MAX_CHANNELS: usize = 2;
const ADV_SETS: usize = 1;

pub type GPHostResources =
    HostResources<DefaultPacketPool, MAX_CONNECTIONS, MAX_CHANNELS, ADV_SETS>;

type External = [u8; 0];

// GATT Server definition
#[gatt_server]
struct Server {
    matter_service: MatterService,
}

/// Matter service
#[gatt_service(uuid = MATTER_BLE_SERVICE_UUID16)]
struct MatterService {
    #[characteristic(uuid = C1_CHARACTERISTIC_UUID, write)]
    c1: External,
    #[characteristic(uuid = C2_CHARACTERISTIC_UUID, write, indicate)]
    c2: External,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct IndBuffer {
    addr: BtAddr,
    data: Vec<u8, MAX_MTU_SIZE>,
    in_flight: bool,
}

impl IndBuffer {
    #[inline(always)]
    const fn new() -> Self {
        Self {
            addr: BtAddr([0; 6]),
            data: Vec::new(),
            in_flight: false,
        }
    }

    fn init() -> impl Init<Self> {
        init!(Self {
            addr: BtAddr([0; 6]),
            data <- Vec::init(),
            in_flight: false,
        })
    }
}

/// The `'static` state of the `TroubleBtpGattPeripheral` struct.
/// Isolated as a separate struct to allow for `const fn` construction
/// and static allocation.
pub struct TroubleBtpGattContext<M>
where
    M: RawMutex,
{
    ind: IfMutex<M, IndBuffer>,
    resources: IfMutex<M, GPHostResources>,
}

impl<M> TroubleBtpGattContext<M>
where
    M: RawMutex,
{
    /// Create a new instance.
    #[allow(clippy::large_stack_frames)]
    #[inline(always)]
    pub const fn new() -> Self {
        Self {
            ind: IfMutex::new(IndBuffer::new()),
            resources: IfMutex::new(GPHostResources::new()),
        }
    }

    /// Return an in-place initializer for the type.
    #[allow(clippy::large_stack_frames)]
    pub fn init() -> impl Init<Self> {
        init!(Self {
            ind <- IfMutex::init(IndBuffer::init()),
            // Note: below will break if `HostResources` stops being a bunch of `MaybeUninit`s
            resources <- IfMutex::init(unsafe { MaybeUninit::<GPHostResources>::uninit().assume_init() }),
        })
    }

    // pub(crate) fn reset(&self) -> Result<(), ()> {
    //     unwrap!(self.ind
    //         .try_lock()
    //         .map(|mut ind| {
    //             ind.data.clear();
    //         })); // TODO

    //     Ok(())
    // }
}

impl<M> Default for TroubleBtpGattContext<M>
where
    M: RawMutex,
{
    // TODO
    #[allow(clippy::large_stack_frames)]
    #[inline(always)]
    fn default() -> Self {
        Self::new()
    }
}

/// A GATT peripheral implementation for the BTP protocol in `rs-matter` via `trouble-host`.
/// Implements the `GattPeripheral` trait.
pub struct TroubleBtpGattPeripheral<'a, M, R, C>
where
    M: RawMutex,
    R: RngCore + Copy,
    C: Controller,
{
    // TODO: Ideally this should be the controller itself, but this is not possible
    // until `bt-hci` is updated with `impl<C: Controller>` Controller for &C {}`
    ble_ctl: IfMutex<M, C>,
    rand: Option<R>,
    context: &'a TroubleBtpGattContext<M>,
}

impl<'a, M, R, C> TroubleBtpGattPeripheral<'a, M, R, C>
where
    M: RawMutex,
    R: RngCore + Copy,
    C: Controller,
{
    /// Create a new instance.
    ///
    /// Creation might fail if the GATT context cannot be reset, so user should ensure
    /// that there are no other GATT peripherals running before calling this function.
    pub const fn new(ble_ctl: C, rand: Option<R>, context: &'a TroubleBtpGattContext<M>) -> Self {
        Self {
            ble_ctl: IfMutex::new(ble_ctl),
            rand,
            context,
        }
    }

    /// Run the GATT peripheral.
    pub async fn run<F>(
        &self,
        service_name: &str,
        service_adv_data: &AdvData,
        mut callback: F,
    ) -> Result<(), ()>
    where
        F: FnMut(GattPeripheralEvent) + Send,
    {
        info!("Starting advertising and GATT service");

        let ble_ctl = self.ble_ctl.lock().await;
        let mut resources = self.context.resources.lock().await;

        let controller = ControllerRef::new(&*ble_ctl);

        let stack = trouble_host::new(controller, &mut resources);

        let stack = if let Some(mut rand) = self.rand {
            let mut address = [0; 6];

            // Generate a valid BLE Static Random Address by regenerating until compliant
            loop {
                rand.fill_bytes(&mut address);

                // Ensure the address is a valid BLE Static Random Address
                // (two most significant bits of the MSB must be 11)
                address[5] |= 0xC0;

                // Check that the lower 46 bits contain at least one 0 and one 1
                // by verifying the address is not all 1s or all 0s (after masking the top 2 bits)
                let mut has_zero = false;
                let mut has_one = false;

                // Check all bytes
                for i in 0..5 {
                    if address[i] != 0xFF {
                        has_zero = true;
                    }
                    if address[i] != 0x00 {
                        has_one = true;
                    }
                }
                // Check lower 6 bits of the MSB (bits 0-5, excluding bits 6-7 which we set to 11)
                let msb_lower = address[5] & 0x3F;
                if msb_lower != 0x3F {
                    has_zero = true;
                }
                if msb_lower != 0x00 {
                    has_one = true;
                }

                if has_zero && has_one {
                    break; // Address is valid
                }
            }

            info!("Random GATT address = {:?}", address);

            stack.set_random_address(Address::random(address))
        } else {
            stack
        };

        let Host {
            mut peripheral,
            runner,
            ..
        } = stack.build();

        let server = unwrap!(Server::new_with_config(GapConfig::Peripheral(
            PeripheralConfig {
                name: "TrouBLE",                                             // TODO
                appearance: &appearance::power_device::GENERIC_POWER_DEVICE, // TODO
            }
        )));

        let _ = join(Self::run_ble(runner), async {
            loop {
                match Self::advertise(service_name, service_adv_data, &mut peripheral).await {
                    Ok(conn) => {
                        let conn = conn.with_attribute_server(&server).unwrap();

                        let events =
                            Self::handle_events(&server, &conn, &self.context.ind, &mut callback);
                        let indications =
                            Self::handle_indications(&server, &conn, &self.context.ind);

                        select(events, indications).await;
                    }
                    Err(e) => {
                        panic!("[adv] error: {:?}", debug2format!(e)); // TODO: defmt
                    }
                }
            }
        })
        .await;

        Ok(())
    }

    async fn run_ble<CC, P>(mut runner: Runner<'_, CC, P>)
    where
        CC: Controller,
        P: PacketPool,
    {
        loop {
            if let Err(e) = runner.run().await {
                panic!("[ble_task] error: {:?}", debug2format!(e)); // TODO: defmt
            }
        }
    }

    async fn handle_indications(
        server: &Server<'_>,
        conn: &GattConnection<'_, '_, DefaultPacketPool>,
        ind: &IfMutex<M, IndBuffer>,
    ) -> Result<(), trouble_host::Error> {
        loop {
            let mut ind = ind
                .lock_if(|ind| !ind.data.is_empty() && !ind.in_flight)
                .await;

            ind.in_flight = true;

            GattData::send_unsolicited(
                conn.raw(),
                AttUns::Indicate {
                    handle: server.matter_service.c2.handle,
                    data: &ind.data,
                },
            )
            .await?;

            trace!("GATT: Indicate {} len {}", Bytes(&ind.data), ind.data.len());
        }
    }

    /// Stream Events until the connection closes.
    ///
    /// This function will handle the GATT events and process them.
    /// This is how we interact with read and write requests.
    async fn handle_events<F>(
        server: &Server<'_>,
        conn: &GattConnection<'_, '_, DefaultPacketPool>,
        ind: &IfMutex<M, IndBuffer>,
        mut callback: F,
    ) -> Result<(), trouble_host::Error>
    where
        F: FnMut(GattPeripheralEvent) + Send,
    {
        fn to_bt_addr(addr: &BdAddr) -> BtAddr {
            let raw = addr.raw();
            BtAddr([raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]])
        }

        loop {
            match conn.next().await {
                GattConnectionEvent::Disconnected { reason } => {
                    info!("GATT: Disconnect {:?}", reason);

                    callback(GattPeripheralEvent::NotifyUnsubscribed(to_bt_addr(
                        &conn.raw().peer_address(),
                    )));
                    break;
                }
                GattConnectionEvent::Gatt { event } => {
                    let mut write_reply = false;

                    match event.payload().incoming() {
                        AttClient::Request(AttReq::Write {
                            handle,
                            data: bytes,
                        }) => {
                            if handle == server.matter_service.c1.handle {
                                trace!(
                                    "GATT: C1 Write {} len {} / MTU {}",
                                    Bytes(bytes),
                                    bytes.len(),
                                    conn.raw().att_mtu()
                                );

                                callback(GattPeripheralEvent::Write {
                                    address: to_bt_addr(&conn.raw().peer_address()),
                                    data: bytes,
                                    gatt_mtu: Some(conn.raw().att_mtu()),
                                });

                                write_reply = true;
                            } else if Some(handle) == server.matter_service.c2.cccd_handle {
                                let subscribed = bytes[0] != 0;

                                trace!("GATT: Write to C2 CCC descriptor: {:?}", bytes);

                                if subscribed {
                                    callback(GattPeripheralEvent::NotifySubscribed(to_bt_addr(
                                        &conn.raw().peer_address(),
                                    )));
                                } else {
                                    callback(GattPeripheralEvent::NotifyUnsubscribed(to_bt_addr(
                                        &conn.raw().peer_address(),
                                    )));
                                }

                                write_reply = true;
                            }
                        }
                        AttClient::Confirmation(AttCfm::ConfirmIndication) => {
                            trace!("GATT: Confirm indication");

                            ind.with(|ind| {
                                assert!(!ind.data.is_empty() && ind.in_flight);

                                ind.data.clear();
                                ind.in_flight = false;

                                Some(())
                            })
                            .await;

                            continue;
                        }
                        _ => (),
                    }

                    if write_reply {
                        unwrap!(event.into_payload().reply(AttRsp::Write).await);
                    } else {
                        match event.accept() {
                            Ok(reply) => {
                                reply.send().await;
                            }
                            Err(e) => {
                                warn!("GATT: Error accepting event: {:?}", e);
                            }
                        }
                    }
                }
                _ => (),
            }
        }

        info!("GATT: Task finished");

        Ok(())
    }

    /// Create an advertiser to use to connect to a BLE Central, and wait for it to connect.
    async fn advertise<'p, CC, P>(
        service_name: &str,
        service_adv_data: &AdvData,
        peripheral: &mut Peripheral<'p, CC, P>,
    ) -> Result<Connection<'p, P>, BleHostError<CC::Error>>
    where
        CC: Controller,
        P: PacketPool,
    {
        let service_adv_enc_data = service_adv_data
            .service_payload_iter()
            .collect::<Vec<_, 8>>();

        let adv_data = [
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceData16 {
                uuid: MATTER_BLE_SERVICE_UUID16.to_le_bytes(),
                data: &service_adv_enc_data,
            },
            AdStructure::CompleteLocalName(service_name.as_bytes()),
        ];

        let mut adv_enc_data = [0; 31];
        let len = AdStructure::encode_slice(&adv_data, &mut adv_enc_data)?;

        let advertiser = peripheral
            .advertise(
                &Default::default(),
                Advertisement::ConnectableScannableUndirected {
                    adv_data: &adv_enc_data[..len],
                    scan_data: &[],
                },
            )
            .await?;

        info!("GATT: Advertising");

        let conn = advertiser.accept().await?;

        info!("GATT: Connection established");

        Ok(conn)
    }

    /// Indicate new data on characteristic `C2` to a remote peer.
    pub async fn indicate(&self, data: &[u8], address: BtAddr) {
        self.context
            .ind
            .with(|ind| {
                if ind.data.is_empty() {
                    unwrap!(ind.data.extend_from_slice(data));
                    ind.addr = address;

                    Some(())
                } else {
                    None
                }
            })
            .await;
    }
}

impl<M, R, C> GattPeripheral for TroubleBtpGattPeripheral<'_, M, R, C>
where
    M: RawMutex,
    R: RngCore + Copy,
    C: Controller,
{
    async fn run<F>(&self, service_name: &str, adv_data: &AdvData, callback: F) -> Result<(), Error>
    where
        F: FnMut(GattPeripheralEvent) + Send + Clone + 'static,
    {
        TroubleBtpGattPeripheral::run(self, service_name, adv_data, callback)
            .await
            .map_err(|_| {
                error!("Running TroubleBtpGattPeripheral failed");
                ErrorCode::BtpError
            })?;

        Ok(())
    }

    async fn indicate(&self, data: &[u8], address: BtAddr) -> Result<(), Error> {
        TroubleBtpGattPeripheral::indicate(self, data, address).await;

        Ok(())
    }
}

/// A newtype allowing to use a bt_hci `&Controller` as a `Controller`
/// A workaround for:
/// https://github.com/embassy-rs/bt-hci/issues/32
pub struct ControllerRef<'a, C>(&'a C);

impl<'a, C> ControllerRef<'a, C> {
    /// Create a new instance.
    pub const fn new(controller: &'a C) -> Self {
        Self(controller)
    }
}

impl<C> ErrorType for ControllerRef<'_, C>
where
    C: ErrorType,
{
    type Error = C::Error;
}

impl<C> bt_hci::controller::Controller for ControllerRef<'_, C>
where
    C: bt_hci::controller::Controller,
{
    fn write_acl_data(&self, packet: &AclPacket) -> impl Future<Output = Result<(), Self::Error>> {
        self.0.write_acl_data(packet)
    }

    fn write_sync_data(
        &self,
        packet: &SyncPacket,
    ) -> impl Future<Output = Result<(), Self::Error>> {
        self.0.write_sync_data(packet)
    }

    fn write_iso_data(&self, packet: &IsoPacket) -> impl Future<Output = Result<(), Self::Error>> {
        self.0.write_iso_data(packet)
    }

    fn read<'a>(
        &self,
        buf: &'a mut [u8],
    ) -> impl Future<Output = Result<ControllerToHostPacket<'a>, Self::Error>> {
        self.0.read(buf)
    }
}

impl<C> bt_hci::controller::blocking::Controller for ControllerRef<'_, C>
where
    C: bt_hci::controller::blocking::Controller,
{
    fn write_acl_data(&self, packet: &AclPacket) -> Result<(), Self::Error> {
        self.0.write_acl_data(packet)
    }

    fn write_sync_data(&self, packet: &SyncPacket) -> Result<(), Self::Error> {
        self.0.write_sync_data(packet)
    }

    fn write_iso_data(&self, packet: &IsoPacket) -> Result<(), Self::Error> {
        self.0.write_iso_data(packet)
    }

    fn try_write_acl_data(
        &self,
        packet: &AclPacket,
    ) -> Result<(), bt_hci::controller::blocking::TryError<Self::Error>> {
        self.0.try_write_acl_data(packet)
    }

    fn try_write_sync_data(
        &self,
        packet: &SyncPacket,
    ) -> Result<(), bt_hci::controller::blocking::TryError<Self::Error>> {
        self.0.try_write_sync_data(packet)
    }

    fn try_write_iso_data(
        &self,
        packet: &IsoPacket,
    ) -> Result<(), bt_hci::controller::blocking::TryError<Self::Error>> {
        self.0.try_write_iso_data(packet)
    }

    fn read<'a>(&self, buf: &'a mut [u8]) -> Result<ControllerToHostPacket<'a>, Self::Error> {
        self.0.read(buf)
    }

    fn try_read<'a>(
        &self,
        buf: &'a mut [u8],
    ) -> Result<ControllerToHostPacket<'a>, bt_hci::controller::blocking::TryError<Self::Error>>
    {
        self.0.try_read(buf)
    }
}

impl<C, Q> ControllerCmdSync<Q> for ControllerRef<'_, C>
where
    C: ControllerCmdSync<Q>,
    Q: SyncCmd + ?Sized,
{
    fn exec(
        &self,
        cmd: &Q,
    ) -> impl Future<Output = Result<Q::Return, bt_hci::cmd::Error<Self::Error>>> {
        self.0.exec(cmd)
    }
}

impl<C, Q> ControllerCmdAsync<Q> for ControllerRef<'_, C>
where
    C: ControllerCmdAsync<Q>,
    Q: AsyncCmd + ?Sized,
{
    fn exec(&self, cmd: &Q) -> impl Future<Output = Result<(), bt_hci::cmd::Error<Self::Error>>> {
        self.0.exec(cmd)
    }
}

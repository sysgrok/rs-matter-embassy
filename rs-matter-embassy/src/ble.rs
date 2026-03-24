// BLE: `TroubleBtpGattPeripheral` - an implementation of the `GattPeripheral` trait from `rs-matter`.

#![allow(clippy::useless_conversion)] // https://github.com/embassy-rs/trouble/issues/248
#![allow(clippy::needless_borrows_for_generic_args)] // In latest trouble-host: ^^^^^^^^ help: change this to: `External`

use core::fmt::Debug;
use core::future::Future;
use core::mem::MaybeUninit;
use core::ops::Deref;
use core::pin::pin;

use bt_hci::cmd::{AsyncCmd, SyncCmd};
use bt_hci::controller::{ControllerCmdAsync, ControllerCmdSync};
use bt_hci::data::{AclPacket, IsoPacket, SyncPacket};
use bt_hci::ControllerToHostPacket;

use embassy_futures::select::select;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::signal::Signal;

use embedded_io::ErrorType;

use rs_matter_stack::ble::GattPeripheral;
use rs_matter_stack::matter::crypto::RngCore;
use rs_matter_stack::matter::error::{Error, ErrorCode};
use rs_matter_stack::matter::transport::network::btp::{
    AdvData, Btp, C1_CHARACTERISTIC_UUID, C2_CHARACTERISTIC_UUID, MATTER_BLE_SERVICE_UUID16,
};
use rs_matter_stack::matter::transport::network::BtAddr;
use rs_matter_stack::matter::utils::init::{init, Init};
use rs_matter_stack::matter::utils::select::Coalesce;
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

struct TroubleBtpResources {
    resources: GPHostResources,
    ind_buf: Vec<u8, MAX_MTU_SIZE>,
}

impl TroubleBtpResources {
    const fn new() -> Self {
        Self {
            resources: GPHostResources::new(),
            ind_buf: Vec::new(),
        }
    }

    fn init() -> impl Init<Self> {
        init!(Self {
            // Note: below will break if `HostResources` stops being a bunch of `MaybeUninit`s
            resources: unsafe { MaybeUninit::<GPHostResources>::uninit().assume_init() },
            ind_buf <- Vec::init(),
        })
    }
}

/// The state of the `TroubleBtpGattPeripheral` struct.
/// Isolated as a separate struct to allow for `const fn` construction
/// and static allocation.
pub struct TroubleBtpGattContext<M>
where
    M: RawMutex,
{
    resources: IfMutex<M, TroubleBtpResources>,
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
            resources: IfMutex::new(TroubleBtpResources::new()),
        }
    }

    /// Return an in-place initializer for the type.
    pub fn init() -> impl Init<Self> {
        init!(Self {
            resources <- IfMutex::init(TroubleBtpResources::init()),
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
    ble_ctl: C,
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
            ble_ctl,
            rand,
            context,
        }
    }

    /// Run the GATT peripheral.
    pub async fn run(
        &mut self,
        btp: &Btp,
        service_name: &str,
        service_adv_data: &AdvData,
    ) -> Result<(), Error> {
        info!("Starting advertising and GATT service");

        let mut resources = self.context.resources.lock().await;
        let resources = &mut *resources;

        unwrap!(resources.ind_buf.resize_default(MAX_MTU_SIZE));

        let ind_buf = &mut resources.ind_buf;
        let resources = &mut resources.resources;

        let controller = ControllerRef::new(&self.ble_ctl);

        let stack = trouble_host::new(controller, resources);

        let stack = if let Some(mut rand) = self.rand {
            // Generate a valid BLE Static Random Address
            // - Two most significant bits must be 11 (static random address)
            // - Lower 46 bits must contain at least one 0 and one 1
            let address: [u8; 6] = loop {
                let addr = rand.next_u64() & 0x3f_ff_ff_ff_ff_ff;
                if addr != 0 && addr != 0x3f_ff_ff_ff_ff_ff {
                    break (addr | 0xc0_00_00_00_00_00).to_be_bytes()[2..]
                        .try_into()
                        .unwrap();
                }
            };

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

        let mut ble_task = pin!(Self::run_ble(runner));
        let mut peripheral_task = pin!(Self::run_peripheral(
            btp,
            service_name,
            service_adv_data,
            &server,
            &mut peripheral,
            ind_buf
        ));

        select(&mut ble_task, &mut peripheral_task)
            .coalesce()
            .await?;

        Ok(())
    }

    async fn run_ble(
        mut runner: Runner<'_, impl Controller, impl PacketPool>,
    ) -> Result<(), Error> {
        loop {
            runner.run().await.map_err(to_matter_err)?;
        }
    }

    async fn run_peripheral(
        btp: &Btp,
        service_name: &str,
        service_adv_data: &AdvData,
        server: &Server<'_>,
        peripheral: &mut Peripheral<'_, impl Controller, DefaultPacketPool>,
        ind_buf: &mut [u8],
    ) -> Result<(), Error> {
        loop {
            let conn = Self::advertise(service_name, service_adv_data, peripheral)
                .await
                .map_err(to_matter_err)?;

            let conn = conn
                .with_attribute_server(server.deref())
                .map_err(to_matter_err)?;

            btp.reset();

            let ind_ack = Signal::new();

            let events = Self::handle_events(server, &conn, &ind_ack, btp);
            let indications = Self::handle_indications(server, &conn, ind_buf, &ind_ack, btp);

            select(events, indications).coalesce().await?;
        }
    }

    /// Create an advertiser to use to connect to a BLE Central, and wait for it to connect.
    async fn advertise<'p, CC: Controller>(
        service_name: &str,
        service_adv_data: &AdvData,
        peripheral: &mut Peripheral<'p, CC, DefaultPacketPool>,
    ) -> Result<Connection<'p, DefaultPacketPool>, BleHostError<CC::Error>> {
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

    /// Stream events until the connection closes or the other peer unsibscribes from char C2.
    async fn handle_events(
        server: &Server<'_>,
        conn: &GattConnection<'_, '_, DefaultPacketPool>,
        ind_ack: &Signal<M, ()>,
        btp: &Btp,
    ) -> Result<(), Error> {
        fn to_bt_addr(addr: &BdAddr) -> BtAddr {
            let raw = addr.raw();
            BtAddr([raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]])
        }

        let mut subscribed = false;

        loop {
            match conn.next().await {
                GattConnectionEvent::Disconnected { reason } => {
                    info!("GATT: Disconnect: {:?}", reason);
                    break;
                }
                GattConnectionEvent::Gatt { event } => match event.payload().incoming() {
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

                            btp.process_incoming(
                                Some(conn.raw().att_mtu()),
                                to_bt_addr(&conn.raw().peer_address()),
                                bytes,
                            )
                            .map_err(to_matter_err)?;

                            Self::write_reply(event).await?;
                        } else if Some(handle) == server.matter_service.c2.cccd_handle {
                            let subscription_req = bytes[0] != 0;

                            trace!("GATT: Write to C2 CCC descriptor: {:?}", bytes);

                            Self::write_reply(event).await?;

                            if subscription_req {
                                if !subscribed {
                                    info!("GATT: Peer subscribed");
                                    subscribed = true;
                                    ind_ack.signal(());
                                }
                            } else if subscribed {
                                info!("GATT: Peer unsubscribed");
                                break;
                            }
                        } else {
                            Self::accept(event).await?;
                        }
                    }
                    AttClient::Confirmation(AttCfm::ConfirmIndication) => {
                        trace!("GATT: Confirm indication");
                        ind_ack.signal(());
                    }
                    _ => Self::accept(event).await?,
                },
                _ => (),
            }
        }

        info!("GATT: Events task finished");

        Ok(())
    }

    /// Handle outgoing data from Btp as indications
    async fn handle_indications(
        server: &Server<'_>,
        conn: &GattConnection<'_, '_, DefaultPacketPool>,
        ind_buf: &mut [u8],
        ind_ack: &Signal<M, ()>,
        btp: &Btp,
    ) -> Result<(), Error> {
        // Wait until `handle_events` indicates to us
        // that the peer did subscribe to char C2
        ind_ack.wait().await;

        loop {
            let len = btp.process_outgoing(Some(conn.raw().att_mtu()), ind_buf)?;
            if len > 0 {
                let data = &ind_buf[..len];

                GattData::send_unsolicited(
                    conn.raw(),
                    AttUns::Indicate {
                        handle: server.matter_service.c2.handle,
                        data,
                    },
                )
                .await
                .map_err(to_matter_err)?;

                trace!("GATT: Indicate {} len {}", Bytes(data), len);

                ind_ack.wait().await;
            } else {
                btp.wait_outgoing().await;
            }
        }
    }

    async fn write_reply(event: GattEvent<'_, '_, DefaultPacketPool>) -> Result<(), Error> {
        event
            .into_payload()
            .reply(AttRsp::Write)
            .await
            .map_err(to_matter_err)
    }

    async fn accept(event: GattEvent<'_, '_, DefaultPacketPool>) -> Result<(), Error> {
        match event.accept() {
            Ok(reply) => {
                reply.send().await;
            }
            Err(e) => {
                warn!("GATT: Error accepting event: {:?}", e);
            }
        }

        Ok(())
    }
}

impl<M, R, C> GattPeripheral for TroubleBtpGattPeripheral<'_, M, R, C>
where
    M: RawMutex,
    R: RngCore + Copy,
    C: Controller,
{
    async fn run(
        &mut self,
        btp: &Btp,
        service_name: &str,
        adv_data: &AdvData,
    ) -> Result<(), Error> {
        TroubleBtpGattPeripheral::run(self, btp, service_name, adv_data)
            .await
            .map_err(|_| {
                error!("Running TroubleBtpGattPeripheral failed");
                ErrorCode::BtpError
            })?;

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

fn to_matter_err<E: Debug>(err: E) -> Error {
    error!("BLE error: {:?}", debug2format!(err)); // TODO: defmt
    ErrorCode::BtpError.into()
}

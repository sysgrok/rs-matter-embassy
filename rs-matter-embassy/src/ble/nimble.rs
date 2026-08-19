//! BLE: `NimbleBtpGattPeripheral` - an implementation of the `GattPeripheral` trait from
//! `rs-matter`, on top of the NimBLE C host, as exposed by the `nimble-rs` crate.
//!
//! An alternative to the `trouble-host` backend in the sibling `trouble` module. Both drive a
//! plain `bt-hci` controller, so the choice is purely which host stack ends up in the binary.

use core::cell::Cell;

use embassy_futures::select::{select, select3};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use nimble_rs::gap::{conn_find, BleAdvParams, GapEvent};
use nimble_rs::gatt::att_mtu;
use nimble_rs::gatt::server::{BleGattRegister, GattServices, GattsEvent};
use nimble_rs::{gatt_services, Ble, BleError, BleUuid, HostEvent};

use rs_matter_stack::ble::GattPeripheral;
use rs_matter_stack::matter::crypto::RngCore;
use rs_matter_stack::matter::error::{Error, ErrorCode};
use rs_matter_stack::matter::transport::network::btp::{
    AdvData, Btp, C1_CHARACTERISTIC_UUID, C2_CHARACTERISTIC_UUID, C3_CHARACTERISTIC_UUID,
    MATTER_BLE_SERVICE_UUID16,
};
use rs_matter_stack::matter::transport::network::BtAddr;
use rs_matter_stack::matter::utils::cell::RefCell;
use rs_matter_stack::matter::utils::init::{init, Init};
use rs_matter_stack::matter::utils::select::Coalesce;
use rs_matter_stack::matter::utils::storage::Vec;
use rs_matter_stack::matter::utils::sync::blocking::Mutex;
use rs_matter_stack::matter::utils::sync::Signal;

use super::ControllerRef;

/// The `bt-hci` controller contract of the selected BLE backend.
pub use nimble_rs::Controller;

/// The BTP GATT context of the selected BLE backend.
pub type BtpGattContext = NimbleBtpGattContext;

/// The BTP GATT peripheral of the selected BLE backend.
pub type BtpGattPeripheral<'a, R, C> = NimbleBtpGattPeripheral<'a, R, C>;

/// A size which is enough to accommodate the maximum BTP payload plus the GATT header.
/// NimBLE's preferred ATT MTU is 256, so a C1 write can be up to 253 bytes.
const MAX_MTU_SIZE: usize = 512;

/// The maximum size of the BTP advertising payload (AD1 Flags + AD2 UUID16 & Service Data).
const MAX_ADV_DATA_SIZE: usize = 32;

// Raw NimBLE constants. `nimble-rs` does not re-export its `-sys` crate, so the handful of values
// the BTP peripheral needs are spelled out here.
/// `BLE_GAP_CONN_MODE_UND` - connectable, undirected advertising.
const BLE_GAP_CONN_MODE_UND: u8 = 2;
/// `BLE_GAP_DISC_MODE_GEN` - general discoverable mode.
const BLE_GAP_DISC_MODE_GEN: u8 = 2;
/// `BLE_ATT_ERR_INSUFFICIENT_RES` - the ATT error returned when a write cannot be buffered.
const BLE_ATT_ERR_INSUFFICIENT_RES: u8 = 0x11;
/// `BLE_HS_EDONE` - the status a completed (peer-confirmed) indication reports.
const BLE_HS_EDONE: i32 = 14;

// The Matter BTP service UUIDs, as `const BleUuid`s (NimBLE stores 128-bit UUIDs LSB-first; the
// `BleUuid` constructors handle that).
const SVC_UUID: BleUuid = BleUuid::uuid16(MATTER_BLE_SERVICE_UUID16);
const C1_UUID: BleUuid = BleUuid::uuid128(C1_CHARACTERISTIC_UUID);
const C2_UUID: BleUuid = BleUuid::uuid128(C2_CHARACTERISTIC_UUID);
const C3_UUID: BleUuid = BleUuid::uuid128(C3_CHARACTERISTIC_UUID);

// The Matter BTP GATT service, defined statically at compile time (in flash). `C1` is written by
// the peer, `C2` is indicated to the peer, `C3` is read by the peer.
gatt_services!(SERVICES {
    primary(SVC_UUID) {
        chr(C1_UUID, Write);
        chr(C2_UUID, Indicate);
        chr(C3_UUID, Read);
    }
});

/// The GATT service table type, as the `S` type parameter of the `Ble` host it is registered with.
/// The `2` is the one Matter BTP service plus the table's null terminator - if `SERVICES` above
/// ever grows another service, this alias no longer matches and the compiler says so.
type Services = &'static GattServices<2>;

/// The context of the currently running peripheral, as an address (`0` when none is running).
///
/// The event hooks are `'static` (they capture nothing), so they reach the context through this
/// global rather than by borrowing it. Only one BTP peripheral can run at a time, which the `Ble`
/// singleton (and `NimbleBtpGattContext::reset`) enforces.
///
/// A critical-section-guarded `Cell` rather than an `AtomicPtr`: `core::sync::atomic` is absent on
/// the atomic-less baremetal targets this crate supports (e.g. `riscv32imc` for the `esp32c3`),
/// and a critical section is required infrastructure here anyway.
static CONTEXT: Mutex<Cell<usize>, CriticalSectionRawMutex> = Mutex::new(Cell::new(0));

fn set_context(context: Option<&NimbleBtpGattContext>) {
    let addr = context.map_or(0, |context| context as *const _ as usize);
    CONTEXT.lock(|cell| cell.set(addr));
}

fn context() -> Option<&'static NimbleBtpGattContext> {
    let context = CONTEXT.lock(|cell| cell.get()) as *const NimbleBtpGattContext;

    // SAFETY: the address is published by `run` and unpublished before `run` returns, and the
    // context outlives the peripheral.
    unsafe { context.as_ref() }
}

/// The state of the connection to the (single) peer we are talking to.
#[derive(Debug, Clone)]
struct Connection {
    peer: BtAddr,
    conn_handle: u16,
    subscribed: bool,
    mtu: Option<u16>,
}

struct State {
    connection: Option<Connection>,
    conn_gen: usize,
    /// The value attribute handle of `C2`, learned from the registration event; indications are
    /// addressed to it. `0` until the `Ble` host reports it.
    c2_val_handle: u16,
    in_data: Vec<u8, MAX_MTU_SIZE>,
    out_data: Vec<u8, MAX_MTU_SIZE>,
    /// Set by the host-sync / disconnect hooks to request (re)advertising; consumed by the outgoing
    /// pump (advertising is synchronous, so it needs no future of its own).
    need_advertise: bool,
    /// The raw advertising payload, kept so that advertising can be restarted on disconnect
    adv_data: Vec<u8, MAX_ADV_DATA_SIZE>,
}

impl State {
    #[inline(always)]
    const fn new() -> Self {
        Self {
            connection: None,
            conn_gen: 0,
            c2_val_handle: 0,
            in_data: Vec::new(),
            out_data: Vec::new(),
            need_advertise: false,
            adv_data: Vec::new(),
        }
    }

    fn init() -> impl Init<Self> {
        init!(Self {
            connection: None,
            conn_gen: 0,
            c2_val_handle: 0,
            in_data <- Vec::init(),
            out_data <- Vec::init(),
            need_advertise: false,
            adv_data <- Vec::init(),
        })
    }
}

/// The `'static` state of the `NimbleBtpGattPeripheral` struct.
/// Isolated as a separate struct to allow for `const fn` construction
/// and static allocation.
pub struct NimbleBtpGattContext {
    state: Mutex<RefCell<State>, CriticalSectionRawMutex>,
    /// Whether the current outgoing indication is still awaiting the peer's acknowledgment. Kept
    /// *outside* `state` (and its `RefCell`) on purpose: NimBLE can deliver `NotifyComplete`
    /// synchronously from within `ble.indicate()`, which runs while `process_outgoing` holds
    /// `state.borrow_mut()`. A separate (nestable) critical-section cell lets that re-entrant hook
    /// clear it without re-borrowing `state` (which would panic with `BorrowMutError`).
    out_nack: Mutex<Cell<bool>, CriticalSectionRawMutex>,
    /// A signal used to awake the `process_incoming()` loop as there might be incoming data (c1 writes) to process.
    notify_process_incoming: Signal<Option<()>, CriticalSectionRawMutex>,
    /// A signal used to awake the `process_outgoing()` loop as there might be outgoing data (c2
    /// indications) to process, or (re)advertising to (re)start.
    notify_process_outgoing: Signal<Option<()>, CriticalSectionRawMutex>,
}

impl NimbleBtpGattContext {
    /// Create a new instance.
    #[allow(clippy::large_stack_frames)]
    #[inline(always)]
    pub const fn new() -> Self {
        Self {
            state: Mutex::new(RefCell::new(State::new())),
            out_nack: Mutex::new(Cell::new(false)),
            notify_process_incoming: Signal::new(None),
            notify_process_outgoing: Signal::new(None),
        }
    }

    /// Return an in-place initializer for `NimbleBtpGattContext`.
    #[allow(clippy::large_stack_frames)]
    pub fn init() -> impl Init<Self> {
        init!(Self {
            state <- Mutex::init(RefCell::init(State::init())),
            out_nack: Mutex::new(Cell::new(false)),
            notify_process_incoming <- Signal::init(None),
            notify_process_outgoing <- Signal::init(None),
        })
    }

    pub(crate) fn reset(&self) {
        self.state.lock(|state| {
            let mut state = state.borrow_mut();

            state.connection = None;
            state.in_data.clear();
            state.out_data.clear();
            state.adv_data.clear();
            state.need_advertise = false;
            // Re-reported by the `Register` events of the next `Ble` instance
            state.c2_val_handle = 0;
        });

        self.set_out_nack(false);

        self.notify_process_incoming.modify(|state| {
            *state = None;
            (false, ())
        });

        self.notify_process_outgoing.modify(|state| {
            *state = None;
            (false, ())
        });
    }

    fn out_nack(&self) -> bool {
        self.out_nack.lock(|out_nack| out_nack.get())
    }

    fn set_out_nack(&self, value: bool) {
        self.out_nack.lock(|out_nack| out_nack.set(value));
    }
}

impl Default for NimbleBtpGattContext {
    // TODO
    #[allow(clippy::large_stack_frames)]
    fn default() -> Self {
        Self::new()
    }
}

/// A GATT peripheral implementation for the BTP protocol in `rs-matter` via `nimble-rs`.
/// Implements the `GattPeripheral` trait.
pub struct NimbleBtpGattPeripheral<'a, R, C> {
    // TODO: Ideally this should be the controller itself, but this is not possible
    // until `bt-hci` is updated with `impl<C: Controller>` Controller for &C {}`
    ble_ctl: C,
    /// Unused: NimBLE derives its own identity address (the controller's public one, or a random
    /// static one it generates and keeps). Present only so that the two backends share a
    /// constructor signature.
    _rand: Option<R>,
    context: &'a NimbleBtpGattContext,
}

impl<'a, R, C> NimbleBtpGattPeripheral<'a, R, C>
where
    R: RngCore + Copy,
    C: Controller,
{
    /// Create a new instance.
    pub const fn new(ble_ctl: C, rand: Option<R>, context: &'a NimbleBtpGattContext) -> Self {
        Self {
            ble_ctl,
            _rand: rand,
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
        self.context.reset();

        self.context.state.lock(|state| {
            let mut state = state.borrow_mut();

            for byte in service_adv_data.iter() {
                state
                    .adv_data
                    .push(byte)
                    .map_err(|_| Error::new(ErrorCode::NoSpace))?;
            }

            Ok::<_, Error>(())
        })?;

        // Publish the context so that the `'static` event hooks can reach it. Unpublished when
        // `run` returns; any hook racing in afterwards sees `None` and no-ops.
        set_context(Some(self.context));
        let _guard = scopeguard::guard((), |_| set_context(None));

        // Initialize the host and register the static Matter BTP service table (in flash). The host
        // is not driven yet - that happens in `Ble::run` below, after the hooks are set. Dropped
        // (and hence deinitialized) before `_guard`, when this future completes.
        let ble: Ble<'_, Services> =
            Ble::new_with_services(&SERVICES).map_err(to_matter_err_ble)?;

        // The hooks are plain `fn` items: they capture nothing and reach the peripheral state
        // through the global `CONTEXT`, so no `unsafe` borrowing is needed.
        ble.host_subscribe(&on_host_event);
        ble.gap_subscribe(&on_gap_event);
        ble.gatts_subscribe(&on_gatts_event);

        ble.set_device_name(service_name)
            .map_err(to_matter_err_ble)?;

        info!(
            "BTP service registered, device name set to `{}`",
            service_name
        );

        // Three concurrent pumps: the NimBLE host itself, incoming C1 writes, and outgoing C2
        // indications - the last of which also drives (re)advertising synchronously, so that needs
        // no future of its own. This keeps the `run` future (which rs-matter-stack bump-allocates)
        // small.
        select3(
            Self::run_host(&ble, &self.ble_ctl),
            self.process_incoming(btp),
            self.process_outgoing(btp, &ble),
        )
        .coalesce()
        .await
    }

    /// Drive the NimBLE host over the BLE controller. Only ever returns an error.
    async fn run_host(ble: &Ble<'_, Services>, ble_ctl: &C) -> Result<(), Error> {
        let err = ble
            .run(ControllerRef::new(ble_ctl))
            .await
            .expect_err("The NimBLE host cannot complete successfully");

        Err(to_matter_err_ble(err))
    }

    /// Process incoming writes on characteristic `C1` from a remote peer.
    ///
    /// While it might seem that this can be done directly from the GATTS hook, this is not
    /// generally possible because `Btp` might not be `Sync`, while the hook has to be.
    async fn process_incoming(&self, btp: &Btp) -> Result<(), Error> {
        let mut generation = None;

        loop {
            let processed = self.context.state.lock(|state| {
                let mut state = state.borrow_mut();

                // Copy the connection out so the `in_data` borrows below are unambiguous.
                let conn = state.connection.as_ref().map(|c| (c.mtu, c.peer));

                if let Some((mtu, peer)) = conn {
                    if generation != Some(state.conn_gen) {
                        btp.reset();
                        generation = Some(state.conn_gen);
                    }

                    if !state.in_data.is_empty() {
                        btp.process_incoming(mtu, peer, &state.in_data)?;

                        // Unlike e.g. Bluedroid, NimBLE has already acknowledged the write.
                        state.in_data.clear();

                        return Ok::<_, Error>(true);
                    }
                }

                Ok(false)
            })?;

            if !processed {
                self.context.notify_process_incoming.wait_signalled().await;
            }
        }
    }

    /// Indicate new data on characteristic `C2` to a remote peer, and drive (re)advertising.
    async fn process_outgoing(&self, btp: &Btp, ble: &Ble<'_, Services>) -> Result<(), Error> {
        loop {
            // (Re)advertise if the host-sync / disconnect hooks requested it. Synchronous, so it
            // adds no state to this future.
            if self
                .context
                .state
                .lock(|s| core::mem::take(&mut s.borrow_mut().need_advertise))
            {
                if let Err(e) = self.advertise(ble) {
                    error!("Cannot start advertising: {:?}", debug2format!(e));
                }
            }

            let processed = self.context.state.lock(|state| {
                let mut state = state.borrow_mut();
                let state = &mut *state;

                let (mtu, conn_handle) = match state.connection.as_ref() {
                    // Peer is connected and subscribed to indications.
                    Some(conn) if conn.subscribed => (conn.mtu, conn.conn_handle),
                    _ => return Ok::<_, Error>(false),
                };

                if self.context.out_nack() {
                    // The previous indication has not been acknowledged by the peer yet.
                    return Ok(false);
                }

                let c2_handle = state.c2_val_handle;
                if c2_handle == 0 {
                    return Ok(false);
                }

                unwrap!(state.out_data.resize_default(MAX_MTU_SIZE));

                let len = btp.process_outgoing(mtu, &mut state.out_data)?;
                if len == 0 {
                    return Ok(false);
                }

                // Mark unacked *before* the send: NimBLE may deliver `NotifyComplete` synchronously
                // from within `indicate`, and that hook clears `out_nack` - which must win, so it has
                // to be set first. Because `out_nack` lives outside `state`, that re-entrant hook
                // does not touch the `state` borrow we are still holding here, so `indicate` can be
                // called in place - no copy of `out_data` out of the borrow, no bigger future.
                self.context.set_out_nack(true);

                match ble.indicate(conn_handle, c2_handle, &state.out_data[..len]) {
                    Ok(()) => {
                        trace!("Indicated {} bytes", len);
                        Ok(true)
                    }
                    Err(e) => {
                        // The send failed: clear the flag so a later attempt is not blocked forever
                        // by a phantom in-flight indication.
                        self.context.set_out_nack(false);
                        Err(to_matter_err_ble(e))
                    }
                }
            })?;

            if !processed {
                select(
                    btp.wait_outgoing(),
                    self.context.notify_process_outgoing.wait_signalled(),
                )
                .coalesce()
                .await;
            }
        }
    }

    /// Configure and (re)start connectable, undirected advertising. Synchronous - deliberately not a
    /// separate async task, so the bump-allocated `run` future stays small.
    fn advertise(&self, ble: &Ble<'_, Services>) -> Result<(), Error> {
        // Infers (and, if need be, generates) the identity address to advertise with: the
        // controller's public address where it has one, a random static one otherwise.
        let (_, own_addr_type) = ble.address().map_err(to_matter_err_ble)?;

        let adv_data = self
            .context
            .state
            .lock(|state| state.borrow().adv_data.clone());

        ble.adv_set_data(&adv_data).map_err(to_matter_err_ble)?;

        let params = BleAdvParams {
            conn_mode: BLE_GAP_CONN_MODE_UND,
            disc_mode: BLE_GAP_DISC_MODE_GEN,
            ..Default::default()
        };

        ble.adv_start(own_addr_type, &params)
            .map_err(to_matter_err_ble)?;

        info!("Advertising started");

        Ok(())
    }
}

impl<R, C> GattPeripheral for NimbleBtpGattPeripheral<'_, R, C>
where
    R: RngCore + Copy,
    C: Controller,
{
    async fn run(
        &mut self,
        btp: &Btp,
        service_name: &str,
        adv_data: &AdvData,
    ) -> Result<(), Error> {
        NimbleBtpGattPeripheral::run(self, btp, service_name, adv_data)
            .await
            .map_err(|_| {
                error!("Running NimbleBtpGattPeripheral failed");
                ErrorCode::BtpError
            })?;

        Ok(())
    }
}

/// Host lifecycle: advertise once in sync, and again after every stack reset.
fn on_host_event(event: HostEvent) {
    if matches!(event, HostEvent::Sync) {
        if let Some(context) = context() {
            context.state.lock(|s| s.borrow_mut().need_advertise = true);
            context.notify_process_outgoing.signal(());
        }
    }
}

/// GAP: connection lifecycle. (`Subscribe` / `NotifyComplete` are demuxed to the GATTS hook.)
fn on_gap_event(event: GapEvent<'_>) -> i32 {
    let Some(context) = context() else {
        return 0;
    };

    match event {
        GapEvent::Connect {
            conn_handle,
            status,
        } => {
            if status.is_err() {
                warn!("BLE connection failed: {:?}", debug2format!(status));
                // The attempt failed, so we have to advertise again.
                context.state.lock(|s| s.borrow_mut().need_advertise = true);
                context.notify_process_outgoing.signal(());
                return 0;
            }

            let peer = conn_find(conn_handle)
                .map(|desc| BtAddr(desc.peer_addr().val()))
                .unwrap_or(BtAddr([0; 6]));
            let mtu = att_mtu(conn_handle).ok();

            context.state.lock(|state| {
                let mut state = state.borrow_mut();

                state.conn_gen = state.conn_gen.wrapping_add(1);
                state.in_data.clear();
                state.out_data.clear();
                state.connection = Some(Connection {
                    peer,
                    conn_handle,
                    subscribed: false,
                    mtu,
                });
            });

            context.set_out_nack(false);

            info!("BLE connected, handle: {}", conn_handle);

            context.notify_process_incoming.signal(());
        }
        GapEvent::Disconnect { reason, .. } => {
            context.state.lock(|state| {
                let mut state = state.borrow_mut();

                state.connection = None;
                state.in_data.clear();
                state.out_data.clear();
                // Nobody is connected anymore, so become discoverable again.
                state.need_advertise = true;
            });

            context.set_out_nack(false);

            info!("BLE disconnected, reason: {:?}", debug2format!(reason));

            context.notify_process_incoming.signal(());
            context.notify_process_outgoing.signal(());
        }
        GapEvent::Mtu { conn_handle, value } => {
            context.state.lock(|state| {
                let mut state = state.borrow_mut();

                if let Some(conn) = state.connection.as_mut() {
                    if conn.conn_handle == conn_handle {
                        conn.mtu = Some(value);
                    }
                }
            });

            trace!("MTU negotiated: {}", value);
        }
        _ => {}
    }

    0
}

/// GATT server: learn the `C2` handle, ingest `C1` writes, answer `C3` reads (empty), and
/// track `C2` subscriptions and indication completions.
fn on_gatts_event(event: GattsEvent<'_>) -> u8 {
    let Some(context) = context() else {
        return 0;
    };

    match event {
        GattsEvent::Register(reg) => {
            if let BleGattRegister::Characteristic {
                uuid, val_handle, ..
            } = reg
            {
                if uuid == C2_UUID {
                    context
                        .state
                        .lock(|state| state.borrow_mut().c2_val_handle = val_handle);
                }
            }
        }
        // A `C1` write carries an incoming BTP packet.
        GattsEvent::Write { data, .. } => {
            let result = context.state.lock(|state| {
                let mut state = state.borrow_mut();

                if !state.in_data.is_empty() {
                    // The previous write has not been processed yet.
                    return Err(());
                }

                state.in_data.resize_default(MAX_MTU_SIZE).map_err(|_| ())?;

                let len = data.read(&mut state.in_data).map_err(|_| ())?;
                state.in_data.truncate(len);

                Ok(())
            });

            if result.is_err() {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }

            context.notify_process_incoming.signal(());
        }
        // A `C3` read: answered with an empty payload (no additional commissioning data).
        GattsEvent::Read { .. } => {}
        // A `C2` subscription change: gate outgoing indications on it. Teardown and bond
        // restore report the resulting state too, so `cur_indicate` is all we need.
        GattsEvent::SubscriptionChanged {
            conn_handle,
            attr_handle,
            cur_indicate,
            ..
        } => {
            let ours = context.state.lock(|state| {
                let mut state = state.borrow_mut();

                if state.c2_val_handle != attr_handle {
                    return false;
                }

                if let Some(conn) = state.connection.as_mut() {
                    if conn.conn_handle == conn_handle {
                        conn.subscribed = cur_indicate;
                    }
                }

                true
            });

            if ours {
                info!(
                    "Peer {} to `C2`",
                    if cur_indicate {
                        "subscribed"
                    } else {
                        "unsubscribed"
                    }
                );

                context.notify_process_outgoing.signal(());
            }
        }
        // An indication completed (peer confirmed, or it failed); the next one may go out.
        GattsEvent::NotifyComplete {
            indication, status, ..
        } => {
            if indication && (status == BLE_HS_EDONE || status != 0) {
                // `out_nack` lives *outside* `state`, so this can run re-entrantly from inside
                // `ble.indicate()` (which is called while `process_outgoing` holds
                // `state.borrow_mut()`) without re-borrowing `state`.
                context.set_out_nack(false);

                context.notify_process_outgoing.signal(());
            }
        }
    }

    0
}

fn to_matter_err_ble(e: BleError) -> Error {
    error!("BLE error: {:?}", debug2format!(e));
    ErrorCode::BtpError.into()
}

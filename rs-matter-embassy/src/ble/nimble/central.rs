//! A GATT **client** (central / Commissioner-side) BTP transport on top of the type-safe NimBLE
//! GATT-client API.
//!
//! This is the analogue of the peripheral in the parent module for the Matter *commissioner* role:
//! given the BLE address of a commissionable device (as an mDNS/BLE browse would yield), it
//! connects as a GATT central, discovers the Matter BTP service and its `C1`/`C2` characteristics,
//! subscribes to `C2` indications, and drives the `Btp` engine as the **initiator** - writing
//! outgoing BTP segments to `C1` as acknowledged Write Requests and feeding `C2` indications back
//! in. It models rs-matter's `bluer::run_central`; scanning/browsing for the address is a separate
//! concern not handled here.
//!
//! Unlike the peripheral, this is not part of any `MatterStack` assembly - a Matter stack is an
//! Accessory-side notion. It is a standalone utility for `rs-matter` users taking the Controller
//! role, which is why it sits behind its own `nimble-central` feature.

use core::cell::Cell;

use embassy_futures::select::select;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use nimble_rs::gap::GapEvent;
use nimble_rs::gatt::client::GattcEvent;
use nimble_rs::{Ble, BleAddr, HostEvent};

use rs_matter_stack::matter::error::{Error, ErrorCode};
use rs_matter_stack::matter::transport::network::btp::Btp;
use rs_matter_stack::matter::transport::network::BtAddr;
use rs_matter_stack::matter::utils::cell::RefCell;
use rs_matter_stack::matter::utils::init::{init, Init};
use rs_matter_stack::matter::utils::select::Coalesce;
use rs_matter_stack::matter::utils::storage::Vec;
use rs_matter_stack::matter::utils::sync::blocking::Mutex;
use rs_matter_stack::matter::utils::sync::Signal;

use super::{to_matter_err_ble, Controller, C1_UUID, C2_UUID, MAX_MTU_SIZE, SVC_UUID};
use crate::ble::ControllerRef;

/// The peer address type to connect with. Commissionable ESP devices advertise with a public
/// address; a full commissioner would take this from the browse/scan result.
const PEER_ADDR_TYPE: u8 = 0; // BLE_ADDR_PUBLIC

/// The context of the currently running client, as an address (`0` when none is running).
///
/// The central counterpart of the peripheral's `CONTEXT`; see there for why this is a
/// critical-section-guarded `Cell` rather than an `AtomicPtr`.
static CLIENT_CONTEXT: Mutex<Cell<usize>, CriticalSectionRawMutex> = Mutex::new(Cell::new(0));

fn set_client_context(context: Option<&NimbleBtpGattClientContext>) {
    let addr = context.map_or(0, |context| context as *const _ as usize);
    CLIENT_CONTEXT.lock(|cell| cell.set(addr));
}

fn client_context() -> Option<&'static NimbleBtpGattClientContext> {
    let context = CLIENT_CONTEXT.lock(|cell| cell.get()) as *const NimbleBtpGattClientContext;

    // SAFETY: published by `run`, unpublished before `run` returns; the context outlives the client.
    unsafe { context.as_ref() }
}

/// The central-side session state, driven by the GAP / GATT-client event hooks.
struct State {
    /// Set once the host is in sync with the controller.
    synced: bool,
    /// The connection handle, once connected.
    conn_handle: Option<u16>,
    /// Set if the connection attempt failed (or the link dropped).
    failed: bool,
    mtu: Option<u16>,
    peer: BtAddr,
    /// The Matter service's attribute-handle range, learned from service discovery.
    matter_range: Option<(u16, u16)>,
    services_done: bool,
    /// The value handles of `C1` (write) and `C2` (indicate), learned from char discovery.
    c1_val: Option<u16>,
    c2_val: Option<u16>,
    chars_done: bool,
    /// Set once the `C2` CCCD write (indication subscribe) completes.
    subscribed: bool,
    /// An incoming `C2` indication, awaiting delivery to the BTP engine.
    in_data: Vec<u8, MAX_MTU_SIZE>,
    out_data: Vec<u8, MAX_MTU_SIZE>,
    /// Whether a `C1` Write Request is in flight (awaiting its response = BTP flow control).
    out_inflight: bool,
}

impl State {
    const fn new() -> Self {
        Self {
            synced: false,
            conn_handle: None,
            failed: false,
            mtu: None,
            peer: BtAddr([0; 6]),
            matter_range: None,
            services_done: false,
            c1_val: None,
            c2_val: None,
            chars_done: false,
            subscribed: false,
            in_data: Vec::new(),
            out_data: Vec::new(),
            out_inflight: false,
        }
    }

    fn init() -> impl Init<Self> {
        init!(Self {
            synced: false,
            conn_handle: None,
            failed: false,
            mtu: None,
            peer: BtAddr([0; 6]),
            matter_range: None,
            services_done: false,
            c1_val: None,
            c2_val: None,
            chars_done: false,
            subscribed: false,
            in_data <- Vec::init(),
            out_data <- Vec::init(),
            out_inflight: false,
        })
    }
}

/// The `'static` backing state of [`NimbleBtpGattClient`], split out for `const`/static allocation.
pub struct NimbleBtpGattClientContext {
    state: Mutex<RefCell<State>, CriticalSectionRawMutex>,
    /// Fired on any discovery/connection progress; `run` waits on it between the setup steps.
    notify_progress: Signal<Option<()>, CriticalSectionRawMutex>,
    /// Fired when a `C2` indication arrives (there may be incoming data to pump).
    notify_in: Signal<Option<()>, CriticalSectionRawMutex>,
    /// Fired when a `C1` write completes (there may be outgoing data to pump).
    notify_out: Signal<Option<()>, CriticalSectionRawMutex>,
}

impl NimbleBtpGattClientContext {
    /// Create a new instance.
    #[inline(always)]
    pub const fn new() -> Self {
        Self {
            state: Mutex::new(RefCell::new(State::new())),
            notify_progress: Signal::new(None),
            notify_in: Signal::new(None),
            notify_out: Signal::new(None),
        }
    }

    /// Return an in-place initializer.
    pub fn init() -> impl Init<Self> {
        init!(Self {
            state <- Mutex::init(RefCell::init(State::init())),
            notify_progress <- Signal::init(None),
            notify_in <- Signal::init(None),
            notify_out <- Signal::init(None),
        })
    }

    pub(crate) fn reset(&self) {
        self.state.lock(|state| *state.borrow_mut() = State::new());

        for signal in [&self.notify_progress, &self.notify_in, &self.notify_out] {
            signal.modify(|state| {
                *state = None;
                (false, ())
            });
        }
    }
}

impl Default for NimbleBtpGattClientContext {
    fn default() -> Self {
        Self::new()
    }
}

/// A GATT client (Matter Commissioner-side BTP transport) on top of the NimBLE host.
pub struct NimbleBtpGattClient<'a, C> {
    ble_ctl: C,
    context: &'a NimbleBtpGattClientContext,
}

impl<'a, C> NimbleBtpGattClient<'a, C>
where
    C: Controller,
{
    /// Create a new instance. A central has no GATT server, so the host carries no service table.
    pub const fn new(ble_ctl: C, context: &'a NimbleBtpGattClientContext) -> Self {
        Self { ble_ctl, context }
    }

    /// Connect to the commissionable device at `addr` and run the BTP transport against it as
    /// the GATT central / BTP initiator until the session ends.
    ///
    /// **Build-only for now**: the BTP engine cannot be put into initiator mode against the
    /// rs-matter version this crate pins - see the `TODO` in the body.
    ///
    /// As with rs-matter's `run_central`, this deliberately does **not** `btp.reset()` - the
    /// caller resets and queues the first PASE SDU before driving Matter traffic.
    pub async fn run(&mut self, btp: &Btp, addr: BtAddr) -> Result<(), Error> {
        self.context.reset();

        self.context
            .state
            .lock(|state| state.borrow_mut().peer = addr);

        set_client_context(Some(self.context));
        let _guard = scopeguard::guard((), |_| set_client_context(None));

        // No service table: a central has no GATT server. Dropped (and hence deinitialized) before
        // `_guard`, when this future completes.
        let ble: Ble<'_, ()> = Ble::new().map_err(to_matter_err_ble)?;

        ble.host_subscribe(&on_host_event);
        ble.gap_subscribe(&on_gap_event);
        ble.gattc_subscribe(&on_gattc_event);

        // The session sequence only makes progress while the host is being driven, so the two run
        // concurrently - unlike the ESP-IDF original, where starting the host spawned an RTOS task.
        select(
            Self::run_host(&ble, &self.ble_ctl),
            self.run_session(&ble, btp, addr),
        )
        .coalesce()
        .await
    }

    /// Drive the NimBLE host over the BLE controller. Only ever returns an error.
    async fn run_host(ble: &Ble<'_, ()>, ble_ctl: &C) -> Result<(), Error> {
        let err = ble
            .run(ControllerRef::new(ble_ctl))
            .await
            .expect_err("The NimBLE host cannot complete successfully");

        Err(to_matter_err_ble(err))
    }

    /// Connect, discover, subscribe, then pump BTP in both directions.
    async fn run_session(&self, ble: &Ble<'_, ()>, btp: &Btp, addr: BtAddr) -> Result<(), Error> {
        // 1) Wait for the host to synchronize, then connect. `address` infers (and, if need be,
        //    generates) the identity address to connect with.
        self.wait_state(|s| s.synced).await;

        let (_, own_addr_type) = ble.address().map_err(to_matter_err_ble)?;

        let peer = BleAddr::new(PEER_ADDR_TYPE, addr.0);
        ble.connect(own_addr_type, &peer)
            .map_err(to_matter_err_ble)?;

        info!("Connecting to commissionable device {}", addr);

        self.wait_state(|s| s.conn_handle.is_some() || s.failed)
            .await;
        let conn = self
            .connected_handle()
            .ok_or_else(|| Error::new(ErrorCode::BtpError))?;

        // 2) Discover the Matter service and its C1/C2 characteristics.
        ble.discover_services(conn).map_err(to_matter_err_ble)?;
        self.wait_state(|s| s.services_done || s.failed).await;

        let (start, end) = self
            .context
            .state
            .lock(|s| s.borrow().matter_range)
            .ok_or_else(|| {
                error!("Matter BTP service not found on peer");
                Error::new(ErrorCode::BtpError)
            })?;

        ble.discover_characteristics(conn, start, end)
            .map_err(to_matter_err_ble)?;
        self.wait_state(|s| s.chars_done || s.failed).await;

        let (c1_val, c2_val) = self.context.state.lock(|s| {
            let s = s.borrow();
            (s.c1_val, s.c2_val)
        });
        let (c1_val, c2_val) = match (c1_val, c2_val) {
            (Some(c1), Some(c2)) => (c1, c2),
            _ => {
                error!("Matter C1/C2 characteristics not found on peer");
                return Err(Error::new(ErrorCode::BtpError));
            }
        };

        info!(
            "Discovered Matter C1 (handle {}) / C2 (handle {})",
            c1_val, c2_val
        );

        // 3) Subscribe to C2 indications by writing its CCCD (value-handle + 1). NimBLE lays the
        //    CCCD out directly after the value attribute.
        ble.write(conn, c2_val + 1, &[0x02, 0x00])
            .map_err(to_matter_err_ble)?;
        self.wait_state(|s| s.subscribed || s.failed).await;

        if self.context.state.lock(|s| s.borrow().failed) {
            return Err(Error::new(ErrorCode::BtpError));
        }

        info!("Subscribed to C2; driving BTP as initiator");

        // 4) We are the initiator: pump C2 indications -> BTP and BTP -> C1 writes.
        //
        // TODO: `btp.set_initiator(true)` belongs here, and without it this client does not
        // actually work: the BTP engine stays in responder mode, so it waits for a handshake
        // request instead of sending one, and the session never opens. `Btp::set_initiator` is
        // public only in rs-matter after 0.2 - call it here once this crate moves to 0.3.
        // Until then the central is build-only.

        select(
            self.process_incoming(btp),
            self.process_outgoing(btp, ble, conn, c1_val),
        )
        .coalesce()
        .await
    }

    fn connected_handle(&self) -> Option<u16> {
        self.context.state.lock(|s| s.borrow().conn_handle)
    }

    /// Await until `pred` holds over the session state, waking on any progress signal.
    async fn wait_state(&self, pred: impl Fn(&State) -> bool) {
        loop {
            if self.context.state.lock(|s| pred(&s.borrow())) {
                return;
            }

            self.context.notify_progress.wait_signalled().await;
        }
    }

    /// Feed received `C2` indications into the BTP engine.
    async fn process_incoming(&self, btp: &Btp) -> Result<(), Error> {
        loop {
            let processed = self.context.state.lock(|state| {
                let mut state = state.borrow_mut();

                if state.failed {
                    return Err(Error::new(ErrorCode::BtpError));
                }

                if state.in_data.is_empty() {
                    return Ok(false);
                }

                let mtu = state.mtu;
                let peer = state.peer;

                // An empty payload is never a valid BTP frame; non-empty only reaches here.
                btp.process_incoming(mtu, peer, &state.in_data)?;
                state.in_data.clear();

                Ok::<_, Error>(true)
            })?;

            if !processed {
                self.context.notify_in.wait_signalled().await;
            }
        }
    }

    /// Drive BTP output as acknowledged `C1` Write Requests (one segment in flight at a time,
    /// which is the client-to-server half of BTP flow control).
    async fn process_outgoing(
        &self,
        btp: &Btp,
        ble: &Ble<'_, ()>,
        conn: u16,
        c1_val: u16,
    ) -> Result<(), Error> {
        loop {
            let processed = self.context.state.lock(|state| {
                let mut state = state.borrow_mut();

                if state.failed {
                    return Err(Error::new(ErrorCode::BtpError));
                }

                if state.out_inflight {
                    // Awaiting the previous Write Response.
                    return Ok(false);
                }

                let mtu = state.mtu;
                unwrap!(state.out_data.resize_default(MAX_MTU_SIZE));

                let len = btp.process_outgoing(mtu, &mut state.out_data)?;
                if len > 0 {
                    ble.write(conn, c1_val, &state.out_data[..len])
                        .map_err(to_matter_err_ble)?;
                    state.out_inflight = true;

                    trace!("Wrote {} bytes to C1", len);

                    Ok(true)
                } else {
                    Ok::<_, Error>(false)
                }
            })?;

            if !processed {
                select(
                    btp.wait_outgoing(),
                    self.context.notify_out.wait_signalled(),
                )
                .coalesce()
                .await;
            }
        }
    }
}

/// Host lifecycle: record the sync that gates the connect.
fn on_host_event(event: HostEvent) {
    if matches!(event, HostEvent::Sync) {
        if let Some(context) = client_context() {
            context.state.lock(|s| s.borrow_mut().synced = true);
            context.notify_progress.signal(());
        }
    }
}

/// GAP: connection lifecycle. (`Notify` is demuxed to the GATTC hook.)
fn on_gap_event(event: GapEvent<'_>) -> i32 {
    let Some(context) = client_context() else {
        return 0;
    };

    match event {
        GapEvent::Connect {
            conn_handle,
            status,
        } => {
            context.state.lock(|s| {
                let mut s = s.borrow_mut();
                if status.is_ok() {
                    s.conn_handle = Some(conn_handle);
                } else {
                    s.failed = true;
                }
            });
            context.notify_progress.signal(());
        }
        GapEvent::Disconnect { .. } => {
            context.state.lock(|s| s.borrow_mut().failed = true);
            context.notify_progress.signal(());
            context.notify_in.signal(());
            context.notify_out.signal(());
        }
        GapEvent::Mtu { conn_handle, value } => {
            context.state.lock(|s| {
                let mut s = s.borrow_mut();
                if s.conn_handle == Some(conn_handle) {
                    s.mtu = Some(value);
                }
            });
        }
        _ => {}
    }

    0
}

/// GATT client: discovery results, write completions, and received `C2` indications.
fn on_gattc_event(event: GattcEvent<'_>) {
    let Some(context) = client_context() else {
        return;
    };

    match event {
        // Service discovery: collect the Matter service range, signal on completion.
        GattcEvent::Service { service, .. } => match service {
            Some(service) => {
                if service.uuid == SVC_UUID {
                    context.state.lock(|s| {
                        s.borrow_mut().matter_range =
                            Some((service.start_handle, service.end_handle));
                    });
                }
            }
            None => {
                context.state.lock(|s| s.borrow_mut().services_done = true);
                context.notify_progress.signal(());
            }
        },
        // Characteristic discovery: capture C1/C2 value handles, signal on completion.
        GattcEvent::Characteristic { chr, .. } => match chr {
            Some(chr) => context.state.lock(|s| {
                let mut s = s.borrow_mut();
                if chr.uuid == C1_UUID {
                    s.c1_val = Some(chr.val_handle);
                } else if chr.uuid == C2_UUID {
                    s.c2_val = Some(chr.val_handle);
                }
            }),
            None => {
                context.state.lock(|s| s.borrow_mut().chars_done = true);
                context.notify_progress.signal(());
            }
        },
        // A write completed: either the CCCD subscribe, or a C1 segment (flow control).
        GattcEvent::WriteComplete {
            attr_handle,
            status,
            ..
        } => {
            let (c2_val, c1_val) = context
                .state
                .lock(|s| (s.borrow().c2_val, s.borrow().c1_val));

            if Some(attr_handle) == c2_val.map(|h| h + 1) {
                context
                    .state
                    .lock(|s| s.borrow_mut().subscribed = status == 0);
                context.notify_progress.signal(());
            } else if Some(attr_handle) == c1_val {
                context.state.lock(|s| s.borrow_mut().out_inflight = false);
                context.notify_out.signal(());
            }
        }
        // A C2 indication: stash it for the incoming pump (empty payloads are not BTP).
        GattcEvent::Notify {
            attr_handle, data, ..
        } => {
            let c2_val = context.state.lock(|s| s.borrow().c2_val);

            if Some(attr_handle) == c2_val {
                context.state.lock(|s| {
                    let mut s = s.borrow_mut();
                    if s.in_data.is_empty() && s.in_data.resize_default(MAX_MTU_SIZE).is_ok() {
                        match data.read(&mut s.in_data) {
                            Ok(len) => s.in_data.truncate(len),
                            Err(_) => s.in_data.clear(),
                        }
                    }
                });
                context.notify_in.signal(());
            }
        }
        GattcEvent::ReadComplete { .. } => {}
    }
}

//! BTP GATT peripheral support, used by Matter for commissioning over BLE.
//!
//! Which BLE host is compiled depends on the selected backend feature:
//! - `trouble` (the default): the pure-Rust `trouble-host` stack
//! - `nimble`: the NimBLE C host, via the `nimble-rs` crate - which needs a C toolchain for the
//!   target, and a C heap it makes bounded allocations from at runtime
//!
//! Both expose the same [`BtpGattContext`] and [`BtpGattPeripheral`] types (as aliases of their
//! backend-specific names), as well as a [`Controller`] trait alias for the `bt-hci` controller the
//! backend drives, so the rest of the crate does not care which one is in use.
//!
//! The two backends are mutually exclusive, as both would otherwise claim the one BLE controller.

use core::future::Future;

use bt_hci::cmd::{AsyncCmd, SyncCmd};
use bt_hci::controller::{ControllerCmdAsync, ControllerCmdSync};
use bt_hci::data::{AclPacket, IsoPacket, SyncPacket};
use bt_hci::ControllerToHostPacket;

use embedded_io::ErrorType;

#[cfg(feature = "trouble")]
mod trouble;
#[cfg(feature = "trouble")]
pub use trouble::*;

#[cfg(feature = "nimble")]
mod nimble;
#[cfg(feature = "nimble")]
pub use nimble::*;

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

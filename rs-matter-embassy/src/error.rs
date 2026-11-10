use rs_matter::error::{Error, ErrorCode};

use sequential_storage::Error as SError;

/// Converts a `sequential_storage` error to an `rs-matter` error
pub fn to_persist_error<E>(_err: SError<E>) -> Error {
    // TODO: The `rs-matter` error code is too generic
    // TODO: Capture the backtrace and the original error
    ErrorCode::StdIoError.into()
}

/// Converts an error of the IP stack (`embassy-net` or `openthread`) to an `rs-matter` error
#[cfg(any(feature = "embassy-net", feature = "openthread"))]
pub(crate) fn to_net_error<E: core::fmt::Debug>(err: E) -> Error {
    warn!("Network error: {:?}", debug2format!(err));

    ErrorCode::StdIoError.into()
}

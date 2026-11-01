use rs_matter::error::{Error, ErrorCode};

use sequential_storage::Error as SError;

/// Converts a `sequential_storage` error to an `rs-matter` error
pub fn to_persist_error<E>(_err: SError<E>) -> Error {
    // TODO: The `rs-matter` error code is too generic
    // TODO: Capture the backtrace and the original error
    ErrorCode::StdIoError.into()
}

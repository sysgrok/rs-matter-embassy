//! Persistence: `EmbassyPersist` - an implementation of the `Persist` trait that uses the `sequential_storage::map` API

use core::ops::Range;

use embedded_storage_async::nor_flash::MultiwriteNorFlash;

use rs_matter_stack::matter::error::Error;
use rs_matter_stack::persist::{KvBlobStore, MatterPersist};

use sequential_storage::cache::NoCache;

use crate::error::to_persist_error;
use crate::fmt::Bytes;

pub type EmbassyPersist<'a, S, N> = MatterPersist<'a, EmbassyKvBlobStore<S>, N>;

/// A `KvBlobStore`` implementation that uses the `sequential_storage::map` API
/// on top of NOR Flash.
pub struct EmbassyKvBlobStore<S> {
    flash: S,
    flash_range: Range<u32>,
    cache: NoCache,
}

impl<S> EmbassyKvBlobStore<S>
where
    S: MultiwriteNorFlash,
{
    /// Create a new KV blob store instance.
    pub fn new(flash: S, flash_range: Range<u32>) -> Self {
        Self {
            flash,
            flash_range,
            cache: NoCache::new(),
        }
    }

    async fn load<F>(&mut self, key: u16, buf: &mut [u8], cb: F) -> Result<(), Error>
    where
        F: FnOnce(Option<&[u8]>) -> Result<(), Error>,
    {
        let data: Option<&[u8]> = sequential_storage::map::fetch_item(
            &mut self.flash,
            self.flash_range.clone(),
            &mut self.cache,
            buf,
            &key,
        )
        .await
        .map_err(to_persist_error)?;

        cb(data)?;

        debug!(
            "Blob {}: loaded {:?} bytes",
            key,
            data.map(|data| data.len()),
        );
        trace!(
            "Blob {} load details: loaded {:?} bytes, data: {:?}",
            key,
            data.map(|data| data.len()),
            data.map(Bytes)
        );

        Ok(())
    }

    async fn store<F>(&mut self, key: u16, buf: &mut [u8], cb: F) -> Result<(), Error>
    where
        F: FnOnce(&mut [u8]) -> Result<usize, Error>,
    {
        // Not ideal, but both `rs-matter-stack` and `sequential-storage` need a buffer.
        let (matter_buf, seqs_buf) = buf.split_at_mut(buf.len() / 2);

        let len = cb(matter_buf)?;
        let data = &matter_buf[..len];

        sequential_storage::map::store_item(
            &mut self.flash,
            self.flash_range.clone(),
            &mut self.cache,
            seqs_buf,
            &key,
            &data,
        )
        .await
        .map_err(to_persist_error)?;

        debug!("Blob {}: stored {} bytes", key, len);
        trace!(
            "Blob {} store details: stored {} bytes, data: {:?}",
            key,
            len,
            data
        );

        Ok(())
    }

    async fn remove(&mut self, key: u16, buf: &mut [u8]) -> Result<(), Error> {
        sequential_storage::map::remove_item(
            &mut self.flash,
            self.flash_range.clone(),
            &mut self.cache,
            buf,
            &key,
        )
        .await
        .map_err(to_persist_error)?;

        debug!("Blob {}: removed", key);

        Ok(())
    }
}

impl<S> KvBlobStore for EmbassyKvBlobStore<S>
where
    S: MultiwriteNorFlash,
{
    async fn load<F>(&mut self, key: u16, buf: &mut [u8], f: F) -> Result<(), Error>
    where
        F: FnOnce(Option<&[u8]>) -> Result<(), Error>,
    {
        EmbassyKvBlobStore::load(self, key, buf, f).await
    }

    async fn store<F>(&mut self, key: u16, buf: &mut [u8], f: F) -> Result<(), Error>
    where
        F: FnOnce(&mut [u8]) -> Result<usize, Error>,
    {
        EmbassyKvBlobStore::store(self, key, buf, f).await
    }

    async fn remove(&mut self, key: u16, buf: &mut [u8]) -> Result<(), Error> {
        EmbassyKvBlobStore::remove(self, key, buf).await
    }
}

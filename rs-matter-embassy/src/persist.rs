//! Persistence: `SeqMapKvBlobStore` - an implementation of the `KvBlobStore` trait that uses the `sequential_storage::map` API

use core::ops::Range;

use embedded_storage_async::nor_flash::MultiwriteNorFlash;

use rs_matter_stack::matter::error::Error;
use rs_matter_stack::matter::persist::KvBlobStore;

use sequential_storage::cache::NoCache;

use crate::error::to_persist_error;
use crate::fmt::Bytes;

/// A `KvBlobStore`` implementation that uses the `sequential_storage::map` API
/// on top of NOR Flash.
pub struct SeqMapKvBlobStore<S> {
    flash: S,
    flash_range: Range<u32>,
    cache: NoCache,
}

impl<S> SeqMapKvBlobStore<S>
where
    S: MultiwriteNorFlash,
{
    /// Create a new KV blob store instance.
    pub const fn new(flash: S, flash_range: Range<u32>) -> Self {
        Self {
            flash,
            flash_range,
            cache: NoCache::new(),
        }
    }

    async fn load(&mut self, key: u16, buf: &mut [u8]) -> Result<Option<usize>, Error> {
        let data: Option<&[u8]> = sequential_storage::map::fetch_item(
            &mut self.flash,
            self.flash_range.clone(),
            &mut self.cache,
            buf,
            &key,
        )
        .await
        .map_err(to_persist_error)?;

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

        Ok(data.map(|data| data.len()))
    }

    async fn store(&mut self, key: u16, data: &[u8], buf: &mut [u8]) -> Result<(), Error> {
        sequential_storage::map::store_item(
            &mut self.flash,
            self.flash_range.clone(),
            &mut self.cache,
            buf,
            &key,
            &data,
        )
        .await
        .map_err(to_persist_error)?;

        debug!("Blob {}: stored {} bytes", key, data.len());
        trace!(
            "Blob {} store details: stored {} bytes, data: {:?}",
            key,
            data.len(),
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

impl<S> KvBlobStore for SeqMapKvBlobStore<S>
where
    S: MultiwriteNorFlash,
{
    async fn load(&mut self, key: u16, buf: &mut [u8]) -> Result<Option<usize>, Error> {
        SeqMapKvBlobStore::load(self, key, buf).await
    }

    async fn store(&mut self, key: u16, data: &[u8], buf: &mut [u8]) -> Result<(), Error> {
        SeqMapKvBlobStore::store(self, key, data, buf).await
    }

    async fn remove(&mut self, key: u16, buf: &mut [u8]) -> Result<(), Error> {
        SeqMapKvBlobStore::remove(self, key, buf).await
    }
}

//! Persistence: `SeqMapKvBlobStore` - an implementation of the `KvBlobStore` trait that uses the `sequential_storage::map` API

use core::ops::Range;

use embedded_storage_async::nor_flash::MultiwriteNorFlash;

use rs_matter_stack::matter::error::Error;
use rs_matter_stack::matter::fabric::MAX_FABRICS;
use rs_matter_stack::matter::persist::KvBlobStore;
use rs_matter_stack::matter::utils::futures::block_on;
use rs_matter_stack::matter::utils::storage::Vec;

use sequential_storage::cache::key_pointers::ArrayKeyPointers;
use sequential_storage::cache::page_states::CalculatedPageStates;
use sequential_storage::cache::{Cache, Uncached};
use sequential_storage::map::{MapConfig, MapStorage};

use crate::error::to_persist_error;
use crate::fmt::Bytes;

/// How many keys `SeqMapKvBlobStore` tracks: the fabrics, plus room for the
/// singleton, subscription and vendor keys
const MAX_KEYS: usize = MAX_FABRICS + 64;

/// How many key locations the `sequential_storage` cache remembers
const MAX_KEY_POINTERS: usize = 32;

/// A `KvBlobStore` implementation that uses the `sequential_storage::map` API
/// on top of NOR Flash.
///
/// Keeps the keys present in the flash in RAM (learned with one scan on first access),
/// so that loading or removing a missing key does not scan the flash.
pub struct SeqMapKvBlobStore<S: MultiwriteNorFlash> {
    storage: Storage<S>,
    keys: Keys,
}

impl<S> SeqMapKvBlobStore<S>
where
    S: MultiwriteNorFlash,
{
    /// Create a new KV blob store instance.
    ///
    /// Panics if `flash_range` is not aligned to the flash pages or is shorter than two pages.
    pub fn new(flash: S, flash_range: Range<u32>) -> Self {
        let pages = (flash_range.end - flash_range.start) as usize / S::ERASE_SIZE;

        Self {
            storage: MapStorage::new(
                flash,
                MapConfig::new(flash_range),
                Cache::new(
                    CalculatedPageStates::new(pages),
                    Uncached,
                    ArrayKeyPointers::new(),
                ),
            ),
            keys: Keys::new(),
        }
    }

    async fn load<'a>(&mut self, key: u16, buf: &'a mut [u8]) -> Result<Option<&'a [u8]>, Error> {
        let data: Option<&[u8]> = if self.keys.may_contain(&mut self.storage, key, buf).await {
            self.storage
                .fetch_item(buf, &key)
                .await
                .map_err(to_persist_error)?
        } else {
            None
        };

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

        Ok(data)
    }

    async fn store(&mut self, key: u16, data: &[u8], buf: &mut [u8]) -> Result<(), Error> {
        // Before the write, so that the tracked keys stay a superset even if it fails
        self.keys.insert(key);

        self.storage
            .store_item(buf, &key, &data)
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
        if self.keys.may_contain(&mut self.storage, key, buf).await {
            self.storage
                .remove_item(buf, &key)
                .await
                .map_err(to_persist_error)?;

            self.keys.remove(key);
        }

        debug!("Blob {}: removed", key);

        Ok(())
    }
}

impl<S> KvBlobStore for SeqMapKvBlobStore<S>
where
    S: MultiwriteNorFlash,
{
    fn load<'a>(&mut self, key: u16, buf: &'a mut [u8]) -> Result<Option<&'a [u8]>, Error> {
        block_on(SeqMapKvBlobStore::load(self, key, buf))
    }

    fn store(&mut self, key: u16, data: &[u8], buf: &mut [u8]) -> Result<(), Error> {
        block_on(SeqMapKvBlobStore::store(self, key, data, buf))
    }

    fn remove(&mut self, key: u16, buf: &mut [u8]) -> Result<(), Error> {
        block_on(SeqMapKvBlobStore::remove(self, key, buf))
    }
}

type Storage<S> = MapStorage<
    u16,
    S,
    Cache<CalculatedPageStates, Uncached, ArrayKeyPointers<u16, MAX_KEY_POINTERS>, u16>,
>;

/// The keys present in the flash, kept sorted.
///
/// Always a superset of them, so a key not tracked here is surely missing.
struct Keys {
    state: KeysState,
    keys: Vec<u16, MAX_KEYS>,
}

impl Keys {
    const fn new() -> Self {
        Self {
            state: KeysState::Unknown,
            keys: Vec::new(),
        }
    }

    /// Whether `key` might be in `storage`; scans `storage` for its keys on first use
    async fn may_contain<S: MultiwriteNorFlash>(
        &mut self,
        storage: &mut Storage<S>,
        key: u16,
        buf: &mut [u8],
    ) -> bool {
        if matches!(self.state, KeysState::Unknown) {
            self.scan(storage, buf).await;
        }

        !matches!(self.state, KeysState::Tracked) || self.keys.binary_search(&key).is_ok()
    }

    fn insert(&mut self, key: u16) {
        if matches!(self.state, KeysState::Tracked) && !self.add(key) {
            warn!("More than {} keys in storage; not tracking them", MAX_KEYS);
            self.state = KeysState::Untracked;
        }
    }

    fn remove(&mut self, key: u16) {
        if matches!(self.state, KeysState::Tracked) {
            if let Ok(index) = self.keys.binary_search(&key) {
                self.keys.remove(index);
            }
        }
    }

    async fn scan<S: MultiwriteNorFlash>(&mut self, storage: &mut Storage<S>, buf: &mut [u8]) {
        self.keys.clear();

        self.state = match self.collect(storage, buf).await {
            Ok(true) => {
                debug!("Found {} keys in storage", self.keys.len());
                KeysState::Tracked
            }
            Ok(false) => {
                warn!("More than {} keys in storage; not tracking them", MAX_KEYS);
                KeysState::Untracked
            }
            Err(_) => {
                warn!("Scanning the keys in storage failed; not tracking them");
                KeysState::Untracked
            }
        };
    }

    /// Add the keys of all items in `storage`; `false` if they do not fit
    async fn collect<S: MultiwriteNorFlash>(
        &mut self,
        storage: &mut Storage<S>,
        buf: &mut [u8],
    ) -> Result<bool, sequential_storage::Error<S::Error>> {
        let mut items = storage.fetch_all_items(buf).await?;

        // Older copies of a key come up too; `add` skips them
        while let Some((key, _)) = items.next::<&[u8]>(buf).await? {
            if !self.add(key) {
                return Ok(false);
            }
        }

        Ok(true)
    }

    /// Add `key`; `false` if there is no room for it
    fn add(&mut self, key: u16) -> bool {
        match self.keys.binary_search(&key) {
            Ok(_) => true,
            Err(index) => self.keys.insert(index, key).is_ok(),
        }
    }
}

enum KeysState {
    /// The flash is not scanned yet
    Unknown,
    /// All keys in the flash are tracked
    Tracked,
    /// Too many keys, or the scan failed; every lookup goes to the flash
    Untracked,
}

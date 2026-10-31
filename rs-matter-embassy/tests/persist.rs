//! Tests for `SeqMapKvBlobStore`, on a RAM NOR flash shaped like the default ESP-IDF NVS partition

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::ops::Range;
use std::rc::Rc;

use embedded_storage_async::nor_flash::{
    ErrorType, MultiwriteNorFlash, NorFlash, NorFlashErrorKind, ReadNorFlash,
};

use rs_matter_embassy::matter::persist::KvBlobStore;
use rs_matter_embassy::matter::utils::futures::block_on;
use rs_matter_embassy::persist::SeqMapKvBlobStore;

use sequential_storage::cache::Cache;
use sequential_storage::map::{MapConfig, MapStorage};

const PAGES: usize = 6;
const PAGE_SIZE: usize = 4096;
const WORD_SIZE: usize = 4;
const FLASH_RANGE: Range<u32> = 0..(PAGES * PAGE_SIZE) as u32;

/// A RAM NOR flash that counts its reads; clones share the same memory
#[derive(Clone)]
struct Flash(Rc<RefCell<Mem>>);

impl Flash {
    fn new() -> Self {
        Self(Rc::new(RefCell::new(Mem {
            data: vec![0xff; PAGES * PAGE_SIZE],
            reads: Reads::default(),
        })))
    }

    fn reads(&self) -> Reads {
        self.0.borrow().reads
    }

    fn store(&self) -> SeqMapKvBlobStore<Flash> {
        SeqMapKvBlobStore::new(self.clone(), FLASH_RANGE)
    }
}

impl ErrorType for Flash {
    type Error = NorFlashErrorKind;
}

impl ReadNorFlash for Flash {
    const READ_SIZE: usize = WORD_SIZE;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let mut mem = self.0.borrow_mut();
        let range = mem.range(offset, bytes.len(), Self::READ_SIZE)?;

        bytes.copy_from_slice(&mem.data[range]);
        mem.reads.count += 1;
        mem.reads.bytes += bytes.len();

        Ok(())
    }

    fn capacity(&self) -> usize {
        PAGES * PAGE_SIZE
    }
}

impl NorFlash for Flash {
    const WRITE_SIZE: usize = WORD_SIZE;
    const ERASE_SIZE: usize = PAGE_SIZE;

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        let mut mem = self.0.borrow_mut();
        let range = mem.range(from, to.saturating_sub(from) as usize, Self::ERASE_SIZE)?;

        mem.data[range].fill(0xff);

        Ok(())
    }

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        let mut mem = self.0.borrow_mut();
        let range = mem.range(offset, bytes.len(), Self::WRITE_SIZE)?;

        for (old, new) in mem.data[range].iter_mut().zip(bytes) {
            // NOR flash can only clear bits
            assert_eq!(
                *old & new,
                *new,
                "Write over non-erased bits at {offset:#x}"
            );
            *old = *new;
        }

        Ok(())
    }
}

impl MultiwriteNorFlash for Flash {}

struct Mem {
    data: Vec<u8>,
    reads: Reads,
}

impl Mem {
    fn range(
        &self,
        offset: u32,
        len: usize,
        align: usize,
    ) -> Result<Range<usize>, NorFlashErrorKind> {
        let offset = offset as usize;

        if !offset.is_multiple_of(align) || !len.is_multiple_of(align) {
            Err(NorFlashErrorKind::NotAligned)
        } else if offset + len > self.data.len() {
            Err(NorFlashErrorKind::OutOfBounds)
        } else {
            Ok(offset..offset + len)
        }
    }
}

#[derive(Debug, Default, Clone, Copy)]
struct Reads {
    count: usize,
    bytes: usize,
}

impl Reads {
    fn since(self, before: Reads) -> Reads {
        Reads {
            count: self.count - before.count,
            bytes: self.bytes - before.bytes,
        }
    }
}

/// xorshift64
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }

    fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }

    fn bytes(&mut self, len: usize) -> Vec<u8> {
        (0..len).map(|_| self.next() as u8).collect()
    }
}

/// Live keys well within what the store tracks
#[test]
fn tracked_keys() {
    let keys = [1, 2, 3, 77, 200, 254]
        .into_iter()
        .chain(256..=272)
        .chain(0x800..0x804)
        .chain([0x1000, 0x1001, 0xffff])
        .collect::<Vec<_>>();

    for seed in 1..=4 {
        model_check(seed, &keys, 200, 30);
    }
}

/// More live keys than the store tracks, so that it falls back to plain lookups
#[test]
fn untracked_keys() {
    let keys = (1..=200).collect::<Vec<_>>();

    for seed in 1..=2 {
        model_check(seed, &keys, 8, 10);
    }
}

/// A failed store must not hide the key from later loads
#[test]
fn failed_store() {
    let flash = Flash::new();
    let mut store = flash.store();
    let mut buf = [0; 4096];
    let mut rng = Rng(1);

    // Track the keys
    assert_eq!(store.load(1, &mut buf).unwrap(), None);

    let mut stored = BTreeMap::new();
    let failed = (1..)
        .find(|&key| {
            let data = rng.bytes(1000);
            let ok = store.store(key, &data, &mut buf).is_ok();
            if ok {
                stored.insert(key, data);
            }
            !ok
        })
        .unwrap();

    assert_eq!(store.load(failed, &mut buf).unwrap(), None);

    for (key, data) in &stored {
        assert_eq!(store.load(*key, &mut buf).unwrap(), Some(&data[..]));
    }
}

/// Once the keys are tracked, missing keys cost no flash reads
#[test]
fn missing_keys_are_free() {
    let flash = commissioned();
    let keys = boot_keys();
    let mut buf = [0; 4096];

    let before = flash.reads();
    let mut base = MapStorage::<u16, _, _>::new(
        flash.clone(),
        MapConfig::new(FLASH_RANGE),
        Cache::new_uncached(),
    );
    let base_present = keys
        .iter()
        .filter(|key| {
            block_on(base.fetch_item::<&[u8]>(&mut buf, key))
                .unwrap()
                .is_some()
        })
        .count();
    let uncached = flash.reads().since(before);
    drop(base);

    let mut store = flash.store();

    let before = flash.reads();
    let present = keys
        .iter()
        .filter(|&&key| store.load(key, &mut buf).unwrap().is_some())
        .count();
    let cold = flash.reads().since(before);

    assert_eq!(present, base_present);
    assert!(
        cold.bytes * 10 < uncached.bytes,
        "Boot: {cold:?}, without caching: {uncached:?}"
    );

    let missing = keys
        .iter()
        .copied()
        .filter(|&key| store.load(key, &mut buf).unwrap().is_none())
        .collect::<Vec<_>>();

    let before = flash.reads();
    for &key in &missing {
        assert_eq!(store.load(key, &mut buf).unwrap(), None);
        store.remove(key, &mut buf).unwrap();
    }
    assert_eq!(flash.reads().since(before).count, 0);

    // Present keys are found through the cached key locations
    let before = flash.reads();
    for &key in &keys {
        store.load(key, &mut buf).unwrap();
    }
    let warm = flash.reads().since(before);
    assert!(warm.count <= 4 * present, "Warm: {warm:?}");
}

/// Random stores, removes and loads checked against a model, across reboots
fn model_check(seed: u64, keys: &[u16], max_len: usize, boots: usize) {
    let flash = Flash::new();
    let mut model = BTreeMap::<u16, Vec<u8>>::new();
    let mut rng = Rng(seed.wrapping_mul(0x9e37_79b9_7f4a_7c15));
    let mut buf = [0; 4096];

    let mut probe = (0..=0x120).chain(keys.iter().copied()).collect::<Vec<_>>();
    probe.sort();
    probe.dedup();

    for _ in 0..boots {
        let mut store = flash.store();

        check(&mut store, &model, &probe, &mut buf);

        for _ in 0..rng.below(1000) {
            let key = keys[rng.below(keys.len())];

            match rng.below(100) {
                0..45 => {
                    let len = 1 + rng.below(max_len);
                    let data = rng.bytes(len);
                    store.store(key, &data, &mut buf).unwrap();
                    model.insert(key, data);
                }
                45..60 => {
                    store.remove(key, &mut buf).unwrap();
                    model.remove(&key);
                }
                _ => check(&mut store, &model, &[key], &mut buf),
            }
        }

        check(&mut store, &model, &probe, &mut buf);
    }
}

fn check(
    store: &mut SeqMapKvBlobStore<Flash>,
    model: &BTreeMap<u16, Vec<u8>>,
    keys: &[u16],
    buf: &mut [u8],
) {
    for &key in keys {
        let data = store.load(key, buf).unwrap();
        assert_eq!(data, model.get(&key).map(Vec::as_slice), "Key {key}");
    }
}

/// A flash like that of a commissioned device: one fabric, some singletons, a subscription,
/// the OpenThread settings, and old copies of the keys written often
fn commissioned() -> Flash {
    let flash = Flash::new();
    let mut store = flash.store();
    let mut buf = [0; 4096];
    let mut rng = Rng(7);

    let mut put = |key, len| {
        let data = rng.bytes(len);
        store.store(key, &data, &mut buf).unwrap();
    };

    put(1, 1100);
    for (i, key) in (256..=272).enumerate() {
        if i % 3 != 2 {
            put(key, 8 + (i * 37) % 200);
        }
    }
    put(0x800, 220);
    put(0x1000, 300);
    for _ in 0..150 {
        put(268, 4);
        put(270, 8);
        put(266, 120);
    }

    flash
}

/// What a boot loads: every fabric index, the singletons, the subscriptions and the OpenThread settings
fn boot_keys() -> Vec<u16> {
    (1..=255)
        .chain(256..=272)
        .chain(0x800..0x803)
        .chain([0x1000])
        .collect()
}

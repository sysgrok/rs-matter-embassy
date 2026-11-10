//! Tests for picking the network of a `MatterStack` over `EmbassyAnyNetwork`

#![cfg(all(feature = "embassy-net", feature = "openthread"))]

use rs_matter_embassy::matter::dm::clusters::net_comm::{NetworkType, Networks};
use rs_matter_embassy::matter::dm::devices::test::{TEST_DEV_ATT, TEST_DEV_COMM, TEST_DEV_DET};
use rs_matter_embassy::matter::dm::networks::any::AnyNetworks;
use std::collections::BTreeMap;

use rs_matter_embassy::matter::error::Error;
use rs_matter_embassy::matter::persist::{KvBlobStore, NETWORKS_KEY};
use rs_matter_embassy::matter::utils::futures::block_on;
use rs_matter_embassy::matter::utils::init::InitMaybeUninit;
use rs_matter_embassy::stack::MatterStack;
use rs_matter_embassy::wireless::{select_net_type, EmbassyAnyNetwork};

/// A `KvBlobStore` in RAM
#[derive(Default)]
struct MemStore(BTreeMap<u16, Vec<u8>>);

impl KvBlobStore for MemStore {
    fn load<'a>(&mut self, key: u16, buf: &'a mut [u8]) -> Result<Option<&'a [u8]>, Error> {
        Ok(self.0.get(&key).map(|data| {
            buf[..data.len()].copy_from_slice(data);
            &buf[..data.len()]
        }))
    }

    fn store(&mut self, key: u16, data: &[u8], _buf: &mut [u8]) -> Result<(), Error> {
        self.0.insert(key, data.to_vec());
        Ok(())
    }

    fn remove(&mut self, key: u16, _buf: &mut [u8]) -> Result<(), Error> {
        self.0.remove(&key);
        Ok(())
    }
}

#[test]
fn select_net_type_sets_the_store_and_the_context() {
    let stack =
        Box::leak(Box::<MatterStack<'static, 0, EmbassyAnyNetwork>>::new_uninit()).init_with(
            MatterStack::init(&TEST_DEV_DET, TEST_DEV_COMM, &TEST_DEV_ATT),
        );

    let mut store = MemStore::default();
    let mut buf = [0; 1024];

    // Nothing saved: the default
    let net_type = block_on(select_net_type(
        stack,
        &mut store,
        &mut buf,
        NetworkType::Thread,
    ))
    .unwrap();

    assert_eq!(net_type, NetworkType::Thread);
    assert!(stack.context().net_context().ot().is_some());

    // Saved Wifi networks win over the default
    let mut data = [0; 256];
    let len = AnyNetworks::<2>::new(NetworkType::Wifi)
        .save(&mut data)
        .unwrap()
        .unwrap();

    store.store(NETWORKS_KEY, &data[..len], &mut buf).unwrap();

    let net_type = block_on(select_net_type(
        stack,
        &mut store,
        &mut buf,
        NetworkType::Thread,
    ))
    .unwrap();

    assert_eq!(net_type, NetworkType::Wifi);
    assert!(stack.context().net_context().enet().is_some());
    assert_eq!(
        block_on(stack.networks(|networks| networks.net_type())),
        Some(NetworkType::Wifi)
    );
}

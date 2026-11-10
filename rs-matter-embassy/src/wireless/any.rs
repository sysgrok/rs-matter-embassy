//! An Embassy Matter stack for a node that runs Ethernet, Wifi or Thread, whichever it
//! picks at runtime.

use core::marker::PhantomData;
use core::ptr::addr_of_mut;

use crate::eth::EmbassyNetContext;
use crate::matter::dm::clusters::net_comm::NetworkType;
use crate::matter::dm::networks::any::AnyNetworks;
use crate::matter::error::Error;
use crate::matter::persist::KvBlobStore;
use crate::matter::utils::init::{init_from_closure, Init};
use crate::stack::network::{AnyStackNetwork, Embedding, StackNetwork};
use crate::stack::MatterStack;

use super::{EmbassyGatt, OtNetContext};

/// The network of a `MatterStack` that runs Ethernet, Wifi or Thread, whichever the node
/// picks at runtime; with BLE during commissioning on Wifi and Thread. It is the networks
/// store `S`, and the contexts of the BLE controller and of the network
/// ([`EmbassyAnyNetContext`]), which the stack holds so that they can be statically allocated.
///
/// Pick the network with [`select_net_type`] before the stack is shared, then build the driver
/// of that network with its context from [`EmbassyAnyNetContext`], and run the stack with
/// `run`, `run_no_coex` or `run_oper` over an `AnyNetDriver`.
pub struct EmbassyAnyNetwork<S = AnyNetworks>(PhantomData<fn() -> S>);

impl<S> StackNetwork for EmbassyAnyNetwork<S>
where
    S: AnyStackNetwork<Context = ()>,
{
    type Store = S::Store;
    type Context = EmbassyGatt<EmbassyAnyNetContext>;
}

impl<S> AnyStackNetwork for EmbassyAnyNetwork<S> where S: AnyStackNetwork<Context = ()> {}

/// The network context of an [`EmbassyAnyNetwork`]: the `embassy-net` one of Ethernet and
/// Wifi, or the `openthread` one of Thread, in the same memory.
// Holds one context or the other, so the variants differ in size by design.
#[allow(clippy::large_enum_variant)]
#[repr(u8)]
pub enum EmbassyAnyNetContext {
    Enet(EmbassyNetContext) = 0,
    Ot(OtNetContext) = 1,
}

impl EmbassyAnyNetContext {
    // The discriminants of the variants.
    const ENET: u8 = 0;
    const OT: u8 = 1;

    /// Create a new instance, holding the `embassy-net` context.
    pub const fn new() -> Self {
        Self::Enet(EmbassyNetContext::new())
    }

    /// Return an in-place initializer for an instance holding the context of `net_type`.
    pub fn init_for(net_type: NetworkType) -> impl Init<Self> {
        // SAFETY: each branch initializes a whole variant (see `init_variant`).
        unsafe {
            init_from_closure(move |slot: *mut Self| {
                if net_type == NetworkType::Thread {
                    Self::init_variant(slot, Self::OT, OtNetContext::init());
                } else {
                    Self::init_variant(slot, Self::ENET, EmbassyNetContext::init());
                }

                Ok(())
            })
        }
    }

    /// Make it hold the context of `net_type`, unless it does already.
    pub fn set_net_type(&mut self, net_type: NetworkType) {
        if (net_type == NetworkType::Thread) == matches!(self, Self::Ot(_)) {
            return;
        }

        let slot: *mut Self = self;

        // SAFETY: the old context is dropped, and the new one initialized in its place.
        // Neither can panic.
        unsafe {
            core::ptr::drop_in_place(slot);

            Self::init_for(net_type)
                .__init(slot)
                .unwrap_or_else(|never| match never {});
        }
    }

    /// The `embassy-net` context of Ethernet and Wifi, if it holds that one.
    pub fn enet(&self) -> Option<&EmbassyNetContext> {
        match self {
            Self::Enet(context) => Some(context),
            Self::Ot(_) => None,
        }
    }

    /// The `openthread` context of Thread, if it holds that one.
    pub fn ot(&self) -> Option<&OtNetContext> {
        match self {
            Self::Ot(context) => Some(context),
            Self::Enet(_) => None,
        }
    }

    /// Write `tag` and initialize the context with `init` after it, in `slot`.
    ///
    /// # Safety
    ///
    /// `slot` must be valid for writes, and `tag` must be the variant's whose context `init`
    /// initializes. A `#[repr(u8)]` enum is laid out as its variants, each a `#[repr(C)]`
    /// struct of the tag followed by the fields, so this initializes that variant.
    unsafe fn init_variant<T, I>(slot: *mut Self, tag: u8, init: I)
    where
        I: Init<T>,
    {
        #[repr(C)]
        struct Variant<T> {
            tag: u8,
            context: T,
        }

        let variant = slot.cast::<Variant<T>>();

        addr_of_mut!((*variant).tag).write(tag);

        init.__init(addr_of_mut!((*variant).context))
            .unwrap_or_else(|never| match never {});
    }
}

impl Default for EmbassyAnyNetContext {
    fn default() -> Self {
        Self::new()
    }
}

impl Embedding for EmbassyAnyNetContext {
    const INIT: Self = Self::new();

    fn init() -> impl Init<Self> {
        Self::init_for(NetworkType::Ethernet)
    }
}

/// Pick the network to run on `stack`: the one its saved networks are for, or `default` if
/// none are saved, as the node was never commissioned or was reset since. The networks store
/// and the network context of the stack are set to it, so build the driver for the network
/// returned.
///
/// Call it before the stack is shared, as it needs it exclusively.
///
/// # Arguments
/// - `stack` - the stack
/// - `store` - the BLOB store the networks are saved in
/// - `buf` - a scratch buffer for loading the saved networks
/// - `default` - the network to run if none are saved
pub async fn select_net_type<const B: usize, const N: usize, S, E, K>(
    stack: &mut MatterStack<'_, B, S, E>,
    store: K,
    buf: &mut [u8],
    default: NetworkType,
) -> Result<NetworkType, Error>
where
    S: StackNetwork<Store = AnyNetworks<N>, Context = EmbassyGatt<EmbassyAnyNetContext>>,
    E: Embedding,
    K: KvBlobStore,
{
    let net_type = stack.select_net_type(store, buf, default).await?;

    stack.context_mut().net_context_mut().set_net_type(net_type);

    Ok(net_type)
}

//! RNG utilities:
//! - A reseeding CSPRNG ([`ReseedingRng`], [`reseeding_csprng`]) on top of `rand_chacha`,
//!   a good default for the `rs-matter` crypto backends, especially on baremetal
//! - A (temporary) adaptor between `rand_core` 0.9 and `rand_core` 0.10 RNGs ([`RngAdaptor`])

pub use rand_chacha::*;
pub use reseeding::ReseedingRng;

use rand_core010::{TryCryptoRng, TryRng};

mod reseeding;

/// TEMPORARY: an adaptor between the `rand_core` 0.9 and the `rand_core` 0.10 traits, in both directions:
/// - A `rand_core` 0.9 (T)RNG becomes a `rand_core` 0.10 [`TryRng`] / [`TryCryptoRng`] (and thus - when
///   infallible - `Rng` / `CryptoRng`), as used by `rs-matter` and by this crate.
///   Needed for HALs still implementing only `rand_core` 0.6 / 0.9 for their (T)RNG peripherals - as of this
///   writing `embassy-nrf` 0.10 and `embassy-rp` 0.10 (`esp-hal` 1.1 and `embassy-nrf` 0.11 implement 0.10 natively).
///   Wrap such a TRNG in `RngAdaptor` before handing it to [`reseeding_csprng`] or to the `rs-matter` crypto backends.
/// - A `rand_core` 0.10 `Rng` / `CryptoRng` (e.g. the one `rs-matter`'s `Crypto::rand` returns) becomes a
///   `rand_core` 0.9 `RngCore` / `CryptoRng`, for crates still consuming `rand_core` 0.9 RNGs
///   (e.g. `nrf-sdc`, or `openthread` releases before it moved to `rand_core` 0.10).
///
/// This adaptor - and the `rand_core` 0.9 dependency it needs - will be removed once the crates above
/// are on `rand_core` 0.10.
pub struct RngAdaptor<T>(T);

impl<T> RngAdaptor<T> {
    /// Create a new `RngAdaptor` instance wrapping the provided `rand_core` 0.9 RNG.
    pub const fn new(rng: T) -> Self {
        Self(rng)
    }

    /// Unwrap the adaptor, returning the wrapped RNG.
    pub fn into_inner(self) -> T {
        self.0
    }
}

impl<T> TryRng for RngAdaptor<T>
where
    T: ::rand_core::TryRngCore,
    T::Error: core::error::Error,
{
    type Error = T::Error;

    fn try_next_u32(&mut self) -> Result<u32, Self::Error> {
        self.0.try_next_u32()
    }

    fn try_next_u64(&mut self) -> Result<u64, Self::Error> {
        self.0.try_next_u64()
    }

    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), Self::Error> {
        self.0.try_fill_bytes(dest)
    }
}

impl<T> TryCryptoRng for RngAdaptor<T>
where
    T: ::rand_core::TryCryptoRng,
    T::Error: core::error::Error,
{
}

impl<T> ::rand_core::RngCore for RngAdaptor<T>
where
    T: rand_core010::Rng,
{
    fn next_u32(&mut self) -> u32 {
        self.0.next_u32()
    }

    fn next_u64(&mut self) -> u64 {
        self.0.next_u64()
    }

    fn fill_bytes(&mut self, dest: &mut [u8]) {
        self.0.fill_bytes(dest)
    }
}

impl<T> ::rand_core::CryptoRng for RngAdaptor<T> where T: rand_core010::CryptoRng {}

/// Create a reseeding CSPRNG using ChaCha12 as the underlying PRNG.
/// A good default as an argument to the various `rs-matter` crypto backends, especially in baremetal environments.
///
/// # Arguments
/// - `trng`: The true random number generator to use for reseeding. If the TRNG only implements
///   `rand_core` 0.9, wrap it in [`RngAdaptor`] first.
/// - `reseed_threshold`: The number of bytes to generate before reseeding; 0 means "never reseed".
///
/// # Returns
/// The reseeding RNG, or an error if the underlying TRNG failed to provide the initial seed.
pub fn reseeding_csprng<T: TryRng>(
    trng: T,
    reseed_threshold: u64,
) -> Result<ReseedingRng<ChaCha12Rng, T>, T::Error> {
    ReseedingRng::new(reseed_threshold, trng)
}

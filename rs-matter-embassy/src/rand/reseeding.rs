// Copyright 2018 Developers of the Rand project.
// Copyright 2013 The Rust Project Developers.
//
// Licensed under the Apache License, Version 2.0 <LICENSE-APACHE or
// https://www.apache.org/licenses/LICENSE-2.0> or the MIT license
// <LICENSE-MIT or https://opensource.org/licenses/MIT>, at your
// option. This file may not be copied, modified, or distributed
// except according to those terms.

//! A wrapper around another PRNG that reseeds it after it
//! generates a certain number of random bytes.
//!
//! Derived from the `ReseedingRng` of the `rand` project (retired there in `rand` 0.10),
//! re-implemented on top of the plain `rand_core` 0.10 traits rather than its block API
//! so that it can wrap any `SeedableRng` PRNG.

use rand_core010::{SeedableRng, TryCryptoRng, TryRng};

/// A wrapper around any PRNG that implements [`SeedableRng`] and [`TryRng`],
/// that adds the ability to reseed it.
///
/// `ReseedingRng` reseeds the underlying PRNG in the following cases:
///
/// - On a manual call to [`reseed()`].
/// - After `clone()`, the clone will be reseeded on first use.
/// - After a specified number of bytes has been generated (the threshold).
///
/// When the threshold is reached and the reseeding operation fails, a warning is logged
/// and the PRNG keeps generating from its current state; the reseeding is retried
/// after the next threshold worth of bytes.
///
/// # Error handling
///
/// Although unlikely, reseeding the wrapped PRNG can fail. `ReseedingRng` will never panic
/// along this code path; instead, it logs the error and continues with the not-reseeded PRNG.
/// If you need to reseed the PRNG and be sure it happened, call [`reseed()`] directly.
///
/// [`reseed()`]: ReseedingRng::reseed
#[derive(Debug)]
pub struct ReseedingRng<R, Rsdr> {
    inner: R,
    reseeder: Rsdr,
    threshold: i64,
    bytes_until_reseed: i64,
}

impl<R, Rsdr> ReseedingRng<R, Rsdr>
where
    R: SeedableRng,
    Rsdr: TryRng,
{
    /// Create a new `ReseedingRng` from an existing PRNG, combined with a
    /// reseeder (a.k.a. TRNG), and the threshold - the number of bytes after which to reseed the PRNG.
    ///
    /// `threshold` of 0 means "never reseed".
    pub fn new(threshold: u64, mut reseeder: Rsdr) -> Result<Self, Rsdr::Error> {
        let threshold = if threshold == 0 || threshold > i64::MAX as u64 {
            i64::MAX
        } else {
            threshold as i64
        };

        let inner = R::try_from_rng(&mut reseeder)?;

        Ok(Self {
            inner,
            reseeder,
            threshold,
            bytes_until_reseed: threshold,
        })
    }

    /// Immediately reseed the PRNG, and reset the bytes counter
    pub fn reseed(&mut self) -> Result<(), Rsdr::Error> {
        self.inner = R::try_from_rng(&mut self.reseeder)?;
        self.bytes_until_reseed = self.threshold;

        Ok(())
    }

    /// Account for `num_bytes` about to be generated, reseeding first if the threshold was reached
    #[inline(always)]
    fn before_generate(&mut self, num_bytes: usize) {
        if self.bytes_until_reseed <= 0 {
            self.reseed_at_threshold();
        }

        self.bytes_until_reseed -= num_bytes as i64;
    }

    #[inline(never)]
    fn reseed_at_threshold(&mut self) {
        trace!("Reseeding RNG (periodic reseed)");

        if let Err(e) = self.reseed() {
            warn!("Reseeding RNG failed: {}", display2format!(e));
            let _ = e;

            // Retry after the next threshold worth of bytes
            self.bytes_until_reseed = self.threshold;
        }
    }
}

impl<R, Rsdr> TryRng for ReseedingRng<R, Rsdr>
where
    R: SeedableRng + TryRng,
    Rsdr: TryRng,
{
    type Error = R::Error;

    #[inline(always)]
    fn try_next_u32(&mut self) -> Result<u32, Self::Error> {
        self.before_generate(4);
        self.inner.try_next_u32()
    }

    #[inline(always)]
    fn try_next_u64(&mut self) -> Result<u64, Self::Error> {
        self.before_generate(8);
        self.inner.try_next_u64()
    }

    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), Self::Error> {
        self.before_generate(dest.len());
        self.inner.try_fill_bytes(dest)
    }
}

impl<R, Rsdr> TryCryptoRng for ReseedingRng<R, Rsdr>
where
    R: SeedableRng + TryCryptoRng,
    Rsdr: TryCryptoRng,
{
}

impl<R, Rsdr> Clone for ReseedingRng<R, Rsdr>
where
    R: SeedableRng + Clone,
    Rsdr: TryRng + Clone,
{
    fn clone(&self) -> Self {
        Self {
            inner: self.inner.clone(),
            reseeder: self.reseeder.clone(),
            threshold: self.threshold,
            bytes_until_reseed: 0, // reseed the clone on first use
        }
    }
}

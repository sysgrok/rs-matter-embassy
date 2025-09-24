/// `rand` function for the esp chips family.
#[cfg(feature = "esp")]
pub mod esp {
    use core::cell::RefCell;

    use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};

    // ... To erase generics, `Matter` takes a rand `fn` rather than a trait or a closure,
    // so we need to store the `esp_hal::Rng` in a global variable
    static RAND: Mutex<CriticalSectionRawMutex, RefCell<Option<esp_hal::rng::Rng>>> =
        Mutex::new(RefCell::new(None));

    /// Initialize the esp-specific `rand` implementation
    /// Need to do this only once
    pub fn esp_init_rand(rng: esp_hal::rng::Rng) {
        RAND.lock(|r| *r.borrow_mut() = Some(rng));
    }

    /// Generate random bytes using the esp-specific `rand` implementation
    // TODO: Not cryptographically secure?
    pub fn esp_rand(buf: &mut [u8]) {
        RAND.lock(|rng| {
            let mut rng = rng.borrow_mut();

            buf.iter_mut()
                .for_each(|byte| *byte = unwrap!(rng.as_mut()).random() as _);
        })
    }
}

#[cfg(feature = "rp")]
pub mod rp {
    use embassy_rp::clocks::RoscRng;

    // TODO: Not cryptographically secure?
    pub fn rp_rand(buf: &mut [u8]) {
        let mut rng = RoscRng;
        rng.fill_bytes(buf);
    }
}

#[cfg(feature = "nrf")]
pub mod nrf {
    use core::cell::RefCell;

    use embassy_nrf::mode::Blocking;
    use embassy_nrf::rng::Rng;

    use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};

    static RAND: Mutex<CriticalSectionRawMutex, RefCell<Option<Rng<'_, Blocking>>>> =
        Mutex::new(RefCell::new(None));

    /// Initialize the nrf-specific `rand` implementation
    /// Need to do this only once
    pub fn nrf_init_rand(rng: Rng<'static, Blocking>) {
        RAND.lock(|r| *r.borrow_mut() = Some(rng));
    }

    // TODO: Not cryptographically secure?
    pub fn nrf_rand(buf: &mut [u8]) {
        RAND.lock(|rng| {
            let mut rng = rng.borrow_mut();
            unwrap!(rng.as_mut()).blocking_fill_bytes(buf);
        })
    }
}

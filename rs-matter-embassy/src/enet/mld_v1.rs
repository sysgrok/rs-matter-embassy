cfg_if::cfg_if! {
    if #[cfg(feature = "mld-v1-compat")] {
        use core::task::Context;

        use embassy_net::driver::{
            Capabilities, Driver as EnetDriver, HardwareAddress as DriverHardwareAddress,
            LinkState, TxToken,
        };

        /// Conditional driver alias for [`Mldv1ConvertingDriver`].
        #[doc(hidden)]
        pub type Driver<D> = Mldv1ConvertingDriver<D>;

        /// Wrap `driver` in [`Mldv1ConvertingDriver`] when `mld-v1-compat` is enabled.
        pub const fn wrap<D>(driver: D) -> Driver<D> {
            Mldv1ConvertingDriver::new(driver)
        }

        /// `embassy_net::driver::Driver` wrapper that rewrites outgoing single-record
        /// MLDv2 Membership Reports (ICMPv6 type 143) into RFC 2710 MLDv1 Membership
        /// Reports (ICMPv6 type 131) for Wi-Fi APs whose snooping bridge only
        /// inspects MLDv1.
        pub struct Mldv1ConvertingDriver<D> {
            inner: D,
        }

        impl<D> Mldv1ConvertingDriver<D> {
            /// Wrap `inner` with outgoing MLDv2 -> MLDv1 conversion.
            pub const fn new(inner: D) -> Self {
                Self { inner }
            }
        }

        impl<D: EnetDriver> EnetDriver for Mldv1ConvertingDriver<D> {
            type RxToken<'a>
                = D::RxToken<'a>
            where
                Self: 'a;
            type TxToken<'a>
                = Mldv1ConvertingTxToken<D::TxToken<'a>>
            where
                Self: 'a;

            fn receive(&mut self, cx: &mut Context) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
                self.inner
                    .receive(cx)
                    .map(|(rx, tx)| (rx, Mldv1ConvertingTxToken { inner: tx }))
            }

            fn transmit(&mut self, cx: &mut Context) -> Option<Self::TxToken<'_>> {
                self.inner
                    .transmit(cx)
                    .map(|tx| Mldv1ConvertingTxToken { inner: tx })
            }

            fn link_state(&mut self, cx: &mut Context) -> LinkState {
                self.inner.link_state(cx)
            }

            fn capabilities(&self) -> Capabilities {
                self.inner.capabilities()
            }

            fn hardware_address(&self) -> DriverHardwareAddress {
                self.inner.hardware_address()
            }
        }

        /// TX token wrapper that applies [`convert_mldv2_to_mldv1`] in-place before
        /// handing the frame to the underlying driver.
        pub struct Mldv1ConvertingTxToken<T> {
            inner: T,
        }

        impl<T: TxToken> TxToken for Mldv1ConvertingTxToken<T> {
            fn consume<R, F>(self, len: usize, f: F) -> R
            where
                F: FnOnce(&mut [u8]) -> R,
            {
                self.inner.consume(len, |buf| {
                    let r = f(buf);
                    convert_mldv2_to_mldv1(buf);
                    r
                })
            }
        }

        /// Rewrite an outgoing single-record MLDv2 Report (RFC 3810, ICMPv6 type 143)
        /// into an MLDv1 Membership Report (RFC 2710, ICMPv6 type 131) in-place.
        ///
        /// Leaves all other frames untouched. Trailing bytes `[86..90]` are zeroed;
        /// IPv6 payload length is updated to 32 (`HbH(8) + MLDv1(24)`).
        fn convert_mldv2_to_mldv1(buf: &mut [u8]) {
            if buf.len() < 90 || buf[12] != 0x86 || buf[13] != 0xDD || buf[20] != 0 || buf[54] != 58 {
                return;
            }
            if buf[62] != 143 {
                return;
            }
            // Only rewrite single-record reports with 0 sources (MODE_IS_EXCLUDE=2 / CHANGE_TO_EXCLUDE=4).
            let nr_records = u16::from_be_bytes([buf[68], buf[69]]);
            let num_srcs = u16::from_be_bytes([buf[72], buf[73]]);
            if nr_records != 1 || num_srcs != 0 || (buf[70] != 2 && buf[70] != 4) {
                return;
            }

            let mut mcast_addr = [0u8; 16];
            mcast_addr.copy_from_slice(&buf[74..90]);

            // Destination MAC: 33:33 + low 32 bits of the multicast IPv6 address
            buf[0] = 0x33;
            buf[1] = 0x33;
            buf[2..6].copy_from_slice(&mcast_addr[12..16]);

            // Destination IPv6 = the multicast group itself
            buf[38..54].copy_from_slice(&mcast_addr);

            // IPv6 payload length = HbH(8) + ICMPv6 MLDv1(24) = 32
            buf[18] = 0;
            buf[19] = 32;

            // ICMPv6 MLDv1 Membership Report header
            buf[62] = 131;
            buf[63] = 0;
            buf[64] = 0;
            buf[65] = 0;
            buf[66] = 0;
            buf[67] = 0;
            buf[68] = 0;
            buf[69] = 0;
            buf[70..86].copy_from_slice(&mcast_addr);
            buf[86..90].fill(0);

            let cksum = icmpv6_checksum(buf, 62, 24);
            buf[64] = (cksum >> 8) as u8;
            buf[65] = (cksum & 0xFF) as u8;

            info!("MLDv2->MLDv1: rewrote outgoing Report");
        }

        /// Compute the ICMPv6 checksum (RFC 4443 §2.3) over an Ethernet frame whose
        /// IPv6 source/destination addresses sit at `[22..38]` / `[38..54]` and whose
        /// ICMPv6 message starts at `icmp_offset` with length `icmp_len`.
        fn icmpv6_checksum(buf: &[u8], icmp_offset: usize, icmp_len: usize) -> u16 {
            let mut sum: u32 = 0;

            let mut i = 22;
            while i < 54 {
                sum += u16::from_be_bytes([buf[i], buf[i + 1]]) as u32;
                i += 2;
            }
            sum += icmp_len as u32;
            sum += 58_u32; // next header = ICMPv6
            i = icmp_offset;
            while i < icmp_offset + icmp_len {
                sum += u16::from_be_bytes([buf[i], buf[i + 1]]) as u32;
                i += 2;
            }

            while sum >> 16 != 0 {
                sum = (sum & 0xFFFF) + (sum >> 16);
            }
            !(sum as u16)
        }
    } else {
        /// Identity driver alias when `mld-v1-compat` is disabled.
        #[doc(hidden)]
        pub type Driver<D> = D;

        /// Identity pass-through when `mld-v1-compat` is disabled.
        #[inline(always)]
        pub const fn wrap<D>(driver: D) -> Driver<D> {
            driver
        }
    }
}

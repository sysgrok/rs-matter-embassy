//! Host tests for scan configuration resolution and Matter SSID precedence.

#![cfg(feature = "embassy-net")]

use rs_matter_embassy::matter::error::Error;
use rs_matter_embassy::wifi::scan::resolve_scan_config;

#[derive(Clone, Debug, PartialEq, Eq)]
struct MockScanConfig {
    ssid: Option<Vec<u8>>,
    active_min_ms: u32,
    channel: Option<u8>,
}

impl Default for MockScanConfig {
    fn default() -> Self {
        Self {
            ssid: None,
            active_min_ms: 0,
            channel: None,
        }
    }
}

#[test]
fn absent_config_uses_driver_default_when_no_matter_ssid() {
    let resolved = resolve_scan_config(None, None, MockScanConfig::default, |mut cfg, ssid| {
        cfg.ssid = Some(ssid.to_vec());
        Ok::<_, Error>(cfg)
    })
    .unwrap();

    assert_eq!(resolved, MockScanConfig::default());
}

#[test]
fn non_empty_matter_ssid_overrides_only_ssid_preserving_other_fields() {
    let configured = MockScanConfig {
        ssid: Some(b"ConfiguredSsid".to_vec()),
        active_min_ms: 30,
        channel: Some(6),
    };

    let resolved = resolve_scan_config(
        Some(configured),
        Some(b"MatterRequested"),
        MockScanConfig::default,
        |mut cfg, ssid| {
            cfg.ssid = Some(ssid.to_vec());
            Ok::<_, Error>(cfg)
        },
    )
    .unwrap();

    assert_eq!(
        resolved,
        MockScanConfig {
            ssid: Some(b"MatterRequested".to_vec()),
            active_min_ms: 30,
            channel: Some(6),
        }
    );
}

#[test]
fn absent_or_empty_matter_ssid_preserves_configured_ssid_filter() {
    let configured = MockScanConfig {
        ssid: Some(b"ConfiguredSsid".to_vec()),
        active_min_ms: 50,
        channel: Some(11),
    };

    let resolved_none = resolve_scan_config(
        Some(configured.clone()),
        None,
        MockScanConfig::default,
        |mut cfg, ssid| {
            cfg.ssid = Some(ssid.to_vec());
            Ok::<_, Error>(cfg)
        },
    )
    .unwrap();
    assert_eq!(resolved_none, configured);

    let resolved_empty = resolve_scan_config(
        Some(configured.clone()),
        Some(b""),
        MockScanConfig::default,
        |mut cfg, ssid| {
            cfg.ssid = Some(ssid.to_vec());
            Ok::<_, Error>(cfg)
        },
    )
    .unwrap();
    assert_eq!(resolved_empty, configured);
}

#[test]
fn propagates_ssid_conversion_error() {
    let err: Result<MockScanConfig, Error> = resolve_scan_config(
        None,
        Some(b"TooLong"),
        MockScanConfig::default,
        |_cfg, _ssid| Err(rs_matter_embassy::matter::error::ErrorCode::InvalidData.into()),
    );
    assert!(err.is_err());
}

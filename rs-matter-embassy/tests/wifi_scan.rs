//! Tests for configurable Wi-Fi scan result sorting, hidden-SSID filtering, deduplication, and capping.

#![cfg(feature = "embassy-net")]

use rs_matter_embassy::matter::error::{Error, ErrorCode};
use rs_matter_embassy::wifi::scan::{report_scan_results, WifiScanResultConfig};

#[derive(Clone, Debug, PartialEq, Eq)]
struct MockAp {
    ssid: &'static str,
    bssid: [u8; 6],
    rssi: i8,
}

fn ap(ssid: &'static str, bssid_last: u8, rssi: i8) -> MockAp {
    MockAp {
        ssid,
        bssid: [0x00, 0x11, 0x22, 0x33, 0x44, bssid_last],
        rssi,
    }
}

#[test]
fn default_config_preserves_driver_order_duplicates_and_hidden_ssids() {
    let mut aps = [
        ap("Weak", 1, -85),
        ap("", 2, -40),
        ap("Dup", 3, -70),
        ap("Dup", 4, -50),
        ap("Strong", 5, -35),
    ];
    let expected = aps.clone();
    let mut reported = Vec::new();

    let count = report_scan_results(
        &mut aps,
        None,
        &WifiScanResultConfig::default(),
        |a| a.ssid,
        |a| a.rssi,
        |a, _idx| {
            reported.push(a.clone());
            Ok(())
        },
    )
    .unwrap();

    assert_eq!(count, 5);
    assert_eq!(reported, expected);
}

#[test]
fn sort_by_rssi_orders_descending_without_dedup_or_hidden_filter() {
    let mut aps = [
        ap("Weak", 1, -85),
        ap("", 2, -30),
        ap("Dup", 3, -70),
        ap("Dup", 4, -50),
    ];
    let mut reported = Vec::new();
    let config = WifiScanResultConfig {
        sort_by_rssi: true,
        ..WifiScanResultConfig::default()
    };

    let count = report_scan_results(
        &mut aps,
        None,
        &config,
        |a| a.ssid,
        |a| a.rssi,
        |a, _idx| {
            reported.push(a.clone());
            Ok(())
        },
    )
    .unwrap();

    assert_eq!(count, 4);
    assert_eq!(
        reported,
        vec![
            ap("", 2, -30),
            ap("Dup", 4, -50),
            ap("Dup", 3, -70),
            ap("Weak", 1, -85),
        ]
    );
}

#[test]
fn deduplicate_without_sorting_keeps_strongest_bssid_at_first_seen_position() {
    let mut aps = [
        ap("MeshNet", 1, -72),
        ap("OtherNet", 2, -60),
        ap("MeshNet", 3, -48),
        ap("MeshNet", 4, -81),
        ap("ThirdNet", 5, -35),
    ];
    let mut reported = Vec::new();
    let config = WifiScanResultConfig {
        deduplicate_ssids: true,
        ..WifiScanResultConfig::default()
    };

    let count = report_scan_results(
        &mut aps,
        None,
        &config,
        |a| a.ssid,
        |a| a.rssi,
        |a, _idx| {
            reported.push(a.clone());
            Ok(())
        },
    )
    .unwrap();

    assert_eq!(count, 3);
    assert_eq!(
        reported,
        vec![
            ap("MeshNet", 3, -48),
            ap("OtherNet", 2, -60),
            ap("ThirdNet", 5, -35),
        ]
    );
}

#[test]
fn filter_hidden_ssids_applies_only_on_open_scans() {
    let mut aps = [ap("", 1, -30), ap("Visible", 2, -55), ap("", 3, -40)];
    let mut reported = Vec::new();
    let config = WifiScanResultConfig {
        filter_hidden_ssids: true,
        ..WifiScanResultConfig::default()
    };

    let count = report_scan_results(
        &mut aps,
        None,
        &config,
        |a| a.ssid,
        |a| a.rssi,
        |a, _idx| {
            reported.push(a.ssid);
            Ok(())
        },
    )
    .unwrap();

    assert_eq!(count, 1);
    assert_eq!(reported, vec!["Visible"]);

    // Empty slice is also an open scan
    let mut aps_empty_slice = [ap("", 1, -30), ap("Visible", 2, -55)];
    let mut reported_empty_slice = Vec::new();
    let count_empty = report_scan_results(
        &mut aps_empty_slice,
        Some(b""),
        &config,
        |a| a.ssid,
        |a| a.rssi,
        |a, _idx| {
            reported_empty_slice.push(a.ssid);
            Ok(())
        },
    )
    .unwrap();
    assert_eq!(count_empty, 1);
    assert_eq!(reported_empty_slice, vec!["Visible"]);
}

#[test]
fn max_results_applies_after_filtering_and_deduplication() {
    let mut aps = [
        ap("", 1, -20),
        ap("MeshNet", 2, -70),
        ap("MeshNet", 3, -45),
        ap("NetB", 4, -55),
        ap("NetC", 5, -60),
    ];
    let mut reported = Vec::new();
    let config = WifiScanResultConfig {
        sort_by_rssi: true,
        deduplicate_ssids: true,
        filter_hidden_ssids: true,
        max_results: Some(2),
    };

    let count = report_scan_results(
        &mut aps,
        None,
        &config,
        |a| a.ssid,
        |a| a.rssi,
        |a, _idx| {
            reported.push(a.clone());
            Ok(())
        },
    )
    .unwrap();

    assert_eq!(count, 2);
    assert_eq!(reported, vec![ap("MeshNet", 3, -45), ap("NetB", 4, -55)]);
}

#[test]
fn propagates_callback_error() {
    let mut aps = [ap("NetA", 1, -40), ap("NetB", 2, -50)];
    let err: Result<usize, Error> = report_scan_results(
        &mut aps,
        None,
        &WifiScanResultConfig::default(),
        |a| a.ssid,
        |a| a.rssi,
        |_a, _idx| Err(ErrorCode::BufferTooSmall.into()),
    );

    assert!(err.is_err());
}

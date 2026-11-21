use crate::matter::error::Error;

/// Configuration for post-scan Matter Wi-Fi network result processing.
///
/// All options are disabled or unset by default so callers preserve driver order,
/// duplicate BSSIDs, hidden SSIDs, and full result lists unless they opt in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct WifiScanResultConfig {
    /// Sort reported results by descending RSSI (strongest signal first).
    pub sort_by_rssi: bool,
    /// Deduplicate multiple BSSIDs advertising the same SSID, retaining the strongest RSSI entry.
    ///
    /// When `sort_by_rssi` is `false`, the retained entry keeps the position where its SSID first
    /// appeared in the driver results.
    pub deduplicate_ssids: bool,
    /// Omit results with empty SSIDs when Matter performs an open scan.
    pub filter_hidden_ssids: bool,
    /// Optional cap applied to the final reported list after filtering, deduplication, and sorting.
    pub max_results: Option<usize>,
}

impl WifiScanResultConfig {
    /// Create a default configuration with all transformations disabled.
    pub const fn new() -> Self {
        Self {
            sort_by_rssi: false,
            deduplicate_ssids: false,
            filter_hidden_ssids: false,
            max_results: None,
        }
    }

    /// Enable or disable descending RSSI sorting.
    pub const fn with_sort_by_rssi(mut self, sort_by_rssi: bool) -> Self {
        self.sort_by_rssi = sort_by_rssi;
        self
    }

    /// Enable or disable SSID deduplication (retaining the strongest BSSID per SSID).
    pub const fn with_deduplicate_ssids(mut self, deduplicate_ssids: bool) -> Self {
        self.deduplicate_ssids = deduplicate_ssids;
        self
    }

    /// Enable or disable filtering empty SSIDs during open scans.
    pub const fn with_filter_hidden_ssids(mut self, filter_hidden_ssids: bool) -> Self {
        self.filter_hidden_ssids = filter_hidden_ssids;
        self
    }

    /// Set a cap on the final number of reported scan results.
    pub const fn with_max_results(mut self, max_results: usize) -> Self {
        self.max_results = Some(max_results);
        self
    }
}

/// Resolve the driver scan configuration for a Matter scan request.
///
/// Starts from `configured` (or `default_config()` when absent) and overrides only the SSID
/// filter when `network` contains a non-empty SSID.
pub fn resolve_scan_config<C, E>(
    configured: Option<C>,
    network: Option<&[u8]>,
    default_config: impl FnOnce() -> C,
    apply_ssid: impl FnOnce(C, &[u8]) -> Result<C, E>,
) -> Result<C, E> {
    let config = configured.unwrap_or_else(default_config);
    if let Some(ssid) = network.filter(|n| !n.is_empty()) {
        apply_ssid(config, ssid)
    } else {
        Ok(config)
    }
}

/// Process raw driver scan results according to `config` and emit each retained entry.
pub fn report_scan_results<T, F>(
    aps: &mut [T],
    network: Option<&[u8]>,
    config: &WifiScanResultConfig,
    mut ssid_of: impl FnMut(&T) -> &str,
    mut rssi_of: impl FnMut(&T) -> i8,
    mut emit: F,
) -> Result<usize, Error>
where
    F: FnMut(&T, usize) -> Result<(), Error>,
{
    let open_scan = network.is_none_or(|n| n.is_empty());
    let mut write_idx = 0;

    for i in 0..aps.len() {
        let ssid = ssid_of(&aps[i]);

        if open_scan && config.filter_hidden_ssids && ssid.is_empty() {
            continue;
        }

        if config.deduplicate_ssids {
            let existing = (0..write_idx).find(|&j| ssid_of(&aps[j]) == ssid);
            if let Some(j) = existing {
                if rssi_of(&aps[i]) > rssi_of(&aps[j]) {
                    aps.swap(j, i);
                }
                continue;
            }
        }

        aps.swap(write_idx, i);
        write_idx += 1;
    }

    let retained = &mut aps[..write_idx];

    if config.sort_by_rssi {
        retained.sort_unstable_by_key(|a| core::cmp::Reverse(rssi_of(a)));
    }

    let limit = config
        .max_results
        .unwrap_or(retained.len())
        .min(retained.len());
    for (idx, ap) in retained[..limit].iter().enumerate() {
        emit(ap, idx + 1)?;
    }

    Ok(limit)
}

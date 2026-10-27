# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [?.??.?] - ????-??-??
* `SeqMapKvBlobStore` tracks the keys in flash, so a missing key (e.g. an unused fabric index) costs no flash scan anymore; on `sequential-storage` 8
  * Breaking: `SeqMapKvBlobStore::new` is no longer `const`; MSRV is 1.87
* Fix: a Thread ICD's child timeout follows its slowest polling interval as the ICD sets it, not as it was when Thread came up, so that the parent keeps a LIT child between its polls
* Fix: `OtMdns` serves resolves and browses only while the SRP client has a server, so that a resolve (e.g. an ICD Check-In) right after boot isn't sent before the node can reach it
* Thread Network Diagnostics: the MAC and MLE counters (advertise with `thread_diag(mac_counts, mle_counts)`); the hardware address of the Thread interface is its extended address
* Wi-Fi Network Diagnostics: BSSID, security type, channel and RSSI with `esp-radio`; RSSI with `cyw43`
* Ethernet Network Diagnostics over `embassy-net`: carrier detect and packet counts (advertise with `eth_diag(carrier_detect, packet_counts)`)
  * Breaking: the Ethernet `run_preex` takes an `EthDiag`, as per `rs-matter-stack`
* The Wi-Fi and Thread stacks take the networks store type as a parameter, `S`, after `E` (`EmbassyWifiMatterStack<'a, B, E, S>`, `EmbassyThreadMatterStack<'a, B, E, S>`), as per `rs-matter-stack`; by default a `WirelessNetworks` with `MAX_WIRELESS_NETWORKS` networks

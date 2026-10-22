# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [?.??.?] - ????-??-??
* Thread Network Diagnostics: the MAC and MLE counters (advertise with `thread_diag(mac_counts, mle_counts)`); the hardware address of the Thread interface is its extended address
* Wi-Fi Network Diagnostics: BSSID, security type, channel and RSSI with `esp-radio`; RSSI with `cyw43`
* Ethernet Network Diagnostics over `embassy-net`: carrier detect and packet counts (advertise with `eth_diag(carrier_detect, packet_counts)`)
  * Breaking: the Ethernet `run_preex` takes an `EthDiag`, as per `rs-matter-stack`

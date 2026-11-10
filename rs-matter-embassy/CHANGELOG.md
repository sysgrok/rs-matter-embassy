# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [?.??.?] - ????-??-??
* Breaking: no `edge-nal`, `edge-nal-embassy` or `edge-nal-openthread`; `EnetStack` and `OtNetStack` implement the `NetStack` of `rs_matter::stack` themselves. The `nal` module and the `edge-nal-embassy` re-export in `enet` are gone
  * `embassy-net` gets only the features the stack needs (UDP, IPv4, IPv6, multicast, DHCPv4, Ethernet); an app that needs e.g. TCP or DNS turns them on itself
  * `EnetMatterUdpBuffers::init`, for building the buffers in place
* Fix: a Thread node joins the IPv6 multicast groups the stack asks for (Matter groups), instead of ignoring them
* New: `EmbassyAnyNetwork`, for a node that runs Ethernet, Wifi or Thread, picked at runtime with `wireless::select_net_type`; its `EmbassyAnyNetContext` holds the `embassy-net` or the `openthread` context in the same memory. Drive it with an `AnyNetDriver` of the Embassy drivers. `EmbassyGatt::net_context_mut`
* The stacks that commission over BLE need `BTP_BUMP_SIZE` more in their bump arena, as the examples show, since `rs_matter::stack` keeps the BTP transport there
* Breaking: no stack aliases; a stack is a plain `MatterStack` over `EmbassyEthNetwork`, `EmbassyWifiNetwork`, `EmbassyThreadNetwork` or `EmbassyAnyNetwork` (a networks store plus the contexts the drivers need), as in `MatterStack<'a, B, EmbassyWifiNetwork, E>`. Each takes a store of its network type. `EmbassyEthMatterStack`, `EmbassyWifiMatterStack`, `EmbassyThreadMatterStack`, `EmbassyWirelessMatterStack`, `EmbassyWirelessBle`, `EmbassyEth` and `EmbassyNet` are gone, and `EmbassyGatt` no longer holds the user data
* Breaking: on the `stack` of `rs-matter`; the driver traits are gone, as `EmbassyWifi`, `EmbassyThread` and `EmbassyEthernet` implement the drivers of the stack for each radio, and `*Driver` is `*Radio`
* Breaking: update to the `rs-matter` split into crates; `root_handler` takes the `Matter` instance, as per `rs-matter-stack`
* Raise MSRV to 1.92, as per `rs-matter`
* Use `rs-matter`'s `utils::futures`, `utils::timers` and `utils::storage` instead of depending on `embassy-futures`, `embassy-time` and `heapless`
* `SeqMapKvBlobStore` tracks the keys in flash, so a missing key (e.g. an unused fabric index) costs no flash scan anymore; on `sequential-storage` 8
  * Breaking: `SeqMapKvBlobStore::new` is no longer `const`; MSRV is 1.87
* Fix: a Thread ICD's child timeout follows its slowest polling interval as the ICD sets it, not as it was when Thread came up, so that the parent keeps a LIT child between its polls
* Fix: the Thread SRP host name comes from the Thread extended address, not the factory EUI-64, so that the SRP server does not refuse a node whose flash was erased for up to 14 days
* Fix: `OtMdns` serves resolves and browses only while the SRP client has a server, so that a resolve (e.g. an ICD Check-In) right after boot isn't sent before the node can reach it
* Thread Network Diagnostics: the MAC and MLE counters (advertise with `thread_diag(mac_counts, mle_counts)`); the hardware address of the Thread interface is its extended address
* Wi-Fi Network Diagnostics: BSSID, security type, channel and RSSI with `esp-radio`; RSSI with `cyw43`
* Ethernet Network Diagnostics over `embassy-net`: carrier detect and packet counts (advertise with `eth_diag(carrier_detect, packet_counts)`)
  * Breaking: the Ethernet `run_preex` takes an `EthDiag`, as per `rs-matter-stack`
* The Wi-Fi and Thread networks take the networks store type as a parameter, `S` (`EmbassyWifiNetwork<S>`, `EmbassyThreadNetwork<S>`), as per `rs-matter-stack`; by default a `WirelessNetworks` with `DEFAULT_MAX_NETWORKS` networks

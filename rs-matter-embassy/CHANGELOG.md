# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [?.??.?] - ????-??-??
* (Breaking) Update to the latest `rs-matter-stack`, which no longer has a bump allocator: the
  `B` (bump size) const generic of `EmbassyEthMatterStack`, `EmbassyWirelessMatterStack`,
  `EmbassyWifiMatterStack` and `EmbassyThreadMatterStack` is gone, as is the `BUMP_SIZE` tuning
  in the examples. `EmbassyWifiMatterStack<BUMP_SIZE, ()>` becomes `EmbassyWifiMatterStack<()>`, etc.

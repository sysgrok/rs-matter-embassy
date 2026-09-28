# Intermittently Connected Devices (battery-powered Matter over Thread)

This page explains what a "sleepy" Matter device is, which of the two Matter profiles
(SIT or LIT) fits which product, how each maps onto the ESP32-C6's sleep modes, and what
`rs-matter`, `rs-matter-stack` and `rs-matter-embassy` do for you. Two examples go with it:

- [`trv_battery_thread`](../examples/esp/src/bin/trv_battery_thread.rs) - a Short Idle Time
  ICD, the light-sleep model: a battery radiator valve (a heating-only Thermostat) that must
  apply a setpoint within seconds of a controller writing it;
- [`temp_sensor_battery_thread`](../examples/esp/src/bin/temp_sensor_battery_thread.rs) - a
  Long Idle Time ICD, the deep-sleep model: a temperature sensor that takes one reading per
  wake-up.

The two are deliberately the same physical quantity: one device must *react* to what
controllers send it, the other only *reports*. That difference, not the battery, is what picks
the profile.

## Three things that are all called "sleep"

Battery operation on a Thread device involves three separate mechanisms that are easy to
conflate:

1. **The Thread role.** A *Sleepy End Device* (SED) tells its parent router that it keeps its
   receiver off and will *poll* for queued frames. The parent buffers everything for the child
   until the next data poll. This is a promise to the network, not a power state: without it
   the parent forwards frames immediately, gets no acknowledgement from a sleeping child and
   eventually evicts it. Every other mechanism builds on this one.
2. **The radio.** Between polls the 802.15.4 receiver is off. On the ESP32-C6 the receiver
   alone draws about 74 mA, so this is the largest single saving - but only while the CPU is
   awake for other reasons. With the CPU asleep it is moot, since the whole modem is off anyway.
3. **The MCU.** What the CPU does between polls is what decides battery life:

   | ESP32-C6 state (datasheet, typical)              | Current  |
   |--------------------------------------------------|----------|
   | CPU idle, radio off                              | ~17 mA   |
   | Light sleep (CPU and radio off, RAM retained)    | 35-180 µA |
   | Deep sleep (only the low-power timer and memory) | 7 µA     |

   Real boards land higher than the datasheet: regulator quiescent current, pull-ups, and the
   radio wake-up cost per poll. Measured light-sleep floors on C6 boards are 50-300 µA.

## SIT versus LIT: the Matter side

Matter calls a sleepy device an *Intermittently Connected Device* (ICD) and defines two
profiles through the **ICD Management** cluster on the root endpoint:

| | Short Idle Time (SIT) | Long Idle Time (LIT) |
|---|---|---|
| Slowest polling interval (`SII`) | at most 15 s | minutes (once per `IdleModeDuration`) |
| Reachability | any controller can reach it within 15 s | only after a Check-In, or during its active window |
| Controller support needed | none beyond SED support | the controller *registers* as a Check-In client |
| Typical product | actuators a user expects to react: radiator valves, locks, blinds | slow sensors: soil moisture, temperature, air quality |
| Sleep model on the ESP32-C6 | light sleep between polls | deep sleep between wake-ups |

A LIT-capable device without any registered client **operates as SIT** (it polls at least
every 15 s) so that plain controllers can commission and use it. The `OperatingMode`
attribute and the `ICD` DNS-SD TXT key tell the world which one it is right now.

The state machine both profiles share:

- **Active mode** after boot (`ActiveModeDuration`), after any Matter message
  (`ActiveModeThreshold`), on a `StayActiveRequest`, on a user trigger, and for as long as a
  commissioning window is open. The device polls fast (`SAI`) and must not sleep deeply.
- **Idle mode** otherwise, for at most `IdleModeDuration`. The device polls slowly (`SII`) and
  may sleep as deeply as it likes.
- On every idle → active transition (and on boot), a LIT sends a **Check-In** message to each
  registered client whose subscription is gone, so the client can re-subscribe while the
  device is awake.

## Why light sleep for SIT and deep sleep for LIT

Every wake-up costs energy, and the two profiles wake up on very different schedules.

A SIT device polls every few seconds. A light-sleep wake-up is a few milliseconds: the radio
comes back, a data request goes out, the parent's acknowledgement (and any queued frame) comes
in, the chip sleeps again. The Thread attachment, the Matter sessions and the subscriptions all
survive because RAM is retained. Even so, the wake-ups dominate the average current: a 15 s
poll cycle measured at about 230 µA average on a C6 board is 50 µA of sleep floor and 180 µA
worth of wake-ups. Going to deep sleep would cut the floor to 7 µA but turn each wake-up into
a reboot - hundreds of milliseconds at tens of milliamps - which is far more than the floor it
saves. Light sleep wins.

A LIT device wakes up minutes apart. Here the sleep floor dominates, and a reboot per wake-up
is affordable: bootloader, radio, OpenThread re-attaching to its parent, Matter resuming its
sessions, a Check-In, a few seconds of active window, and back to sleep. This only works if
*everything* is persisted: fabrics, ICD registrations and Check-In counter, CASE resumption
records, subscriptions, and the OpenThread attachment state. `rs-matter-embassy` persists the
latter so that OpenThread re-attaches with a single `Child Update Request` exchange instead of
a full attach.

## What the crates do

**rs-matter** owns the protocol:

- `Icd` holds the ICD Management cluster state - registrations, Check-In counter,
  `StayActiveRequest` deadline - *and* the active/idle power mode state machine.
  `IcdMgmtHandler` serves the cluster on the root endpoint; its `run` hook drives the state
  machine, feeds it the transport's activity and the commissioning window, and sends the
  Check-Ins.
- The polling intervals are the `SAI` (active) and `SII` (idle) the device advertises in its
  `BasicInfoConfig`, so the network driver and the controllers agree by construction. The
  `SII` is capped to 15 s while operating as SIT.
- Everything a consumer needs is on `Icd`: `net_params()` (power mode, operating mode, the
  polling interval to use now), `wait_net_changed()` for the network driver,
  `wait_power_changed()` / `wait_idle()` / `wait_active()` for the application, `idle_until()`,
  `request_active()` for a user trigger. The two `wait_*_changed` notifications are
  single-waiter: one task per role.

**rs-matter-embassy** owns the mechanics:

- `EmbassyThread::with_icd(&icd)` makes the OpenThread node a Sleepy End Device (receiver
  off when idle, MTD) and maps the ICD polling interval onto the OpenThread data-poll period,
  sizing the child timeout and the child-supervision check timeout from it.
- Concurrent commissioning keeps working, for Thread and Wifi alike: the coex drivers hand
  the stack a `BleDriver` instead of a BLE controller, and the controller is created only while
  a commissioning window is advertised over BLE and torn down afterwards (`esp-radio`
  de-initializes its BLE stack on drop), so BLE costs nothing once commissioned and is never
  brought up by an already-commissioned device on wake-up. Where the controller is persistent
  by nature (nRF SoftDevice Controller, CYW43) the stack still only advertises during a window.
- The OpenThread persister keeps the active dataset, the network and parent info, the SLAAC
  key and the SRP state across reboots.

What is Thread-specific is only the last step, the mapping of a polling interval onto the
OpenThread data-poll period. A Wifi ICD would map the same `net_params()` onto the station's
power-save mode and listen interval (esp-radio's `PowerSaveMode` plus DTIM skipping), which
is not implemented yet. An Ethernet ICD is not a thing: a wired link cannot poll, and a wired
device is mains-powered anyway.

**The application** owns the sleep itself, because only it knows the hardware:

- SIT: start `esp-rtos` with its automatic light-sleep idle hook and do nothing else - the
  ICD state machine and the poll period take care of the rest. Enable CASE session resumption
  (`case-resumption`) so a returning controller needs one round trip, not a full handshake.
- LIT: wait for idle mode (`icd.wait_idle()`), let the persistence tasks flush, and deep-sleep
  until the next poll is due, with the user trigger armed as a GPIO wake-up source. Enable
  CASE resumption *and* persistent subscriptions (`persistent-subscriptions`): every wake-up is
  a reboot, and with both the device resumes its sessions and its reports on its own.

## Current limitations

- **Light sleep does not engage on the ESP32-C6 yet.** `esp-radio` holds a wake lock for as
  long as the radio is initialized, which keeps the `esp-rtos` idle hook from sleeping. The
  SIT example is complete on the protocol side and will sleep once `esp-radio` releases the
  lock while the 802.15.4 receiver is off; nothing in the example needs to change.
- **A deep-sleep wake-up re-registers with SRP** (one update per wake-up) and re-establishes
  subscriptions through resumed CASE sessions. Both are correct, both cost a round trip; an
  SRP lease check before re-registering would trim the former.
- **The BOOT button cannot wake the C6 from deep sleep.** Only GPIO0..7 have the low-power
  path a deep-sleep wake-up needs; the LIT example uses GPIO4 (pulled up, wake on low).

# Examples

<img src="https://github.com/ivmarkov/rs-matter-embassy/blob/master/examples/acme.jpg" alt="ACME" width="300" height="670">

The examples are tested and _should_ work on the nrf52, rPI Pico W, esp32, esp32s3, esp32c3 and esp32c6.

With that said, it is still early days for all of `rs-matter`, `trouble` (the bare-metal BLE stack in use) 
and `openthread` (the OpenThread Rust wrappers) so you might face issues during the initial commissioning.

Please [report](https://github.com/ivmarkov/rs-matter-embassy/issues) those!

Also, currently the persistance is (temporarily) switched off, so if you stop/restart the MCU, you'll have to go over
the commissioning process again, by first removing your device from your Matter Controller.

## Matter Controller

You need one of:
* **Google**:
  * Google Home/Nest or other Google Matter Controller
  * The Google Home app on your phone
* **Alexa**:
  * Alexa Echo Hub, Echo Dot or other Amazon Matter Controller
  * The Alexa app on your phone
  * Note that Alexa will not work with the Thread examples yet, as no MCU is supported
  * with BLE+Thread coex, and Alexa requires that
* **Apple**:
  * Apple TV or other Apple Matter Controller
  * An iPhone with the Apple Home app

Once you build and flash the firmware, follow the instructions in the phone app.

To start the commissioning process, all apps will ask you to take a screenshot of the QR code which is printed by the firmware when it starts.
Once you do that, you should see a bunch of logs for the commissioning process.

NOTE: Since the firmware is not certified, the app will warn you about that. Disregard and let it proceed anyway.

Upon successful commissioning, you should end up with a Light device which you can turn on/off, and which will also turn on/off by itself every 5 secs.

## How to build and flash

### rPI Pico and Pico W

(The stock Pico only supports Ethernet using the `light_eth` example and W5500)

```sh
cd rp
cargo +nightly build

# Replace `light_wifi` with `light_eth` below to flash the Ethernet example
probe-rs run --chip rp2040 target/thumbv6m-none-eabi/debug/light_wifi
```

### Espressif MCUs

#### esp32

```sh
# Wifi credentials should be valid only if you plan to run the `light_eth` "ethernet" example.
# The `light_wifi` example gets your Wifi settings from the Matter Controller automatically.
export WIFI_SSID=foo
export WIFI_PASS=bar

cargo install espup
espup update

cd esp
cargo +esp build --target xtensa-esp32-none-elf --no-default-features --features esp32,wifi

# Replace `light_wifi` with `light_eth` below to flash the "Ethernet" example
# Replace `light_wifi` with `light_thread` below to flash the Ethernet example (you'll need an esp32c6 or esp32h2)
espflash flash target/xtensa-esp32-none-elf/debug/light_wifi --baud 1500000
espflash monitor --elf target/xtensa-esp32-none-elf/debug/light_wifi
```

#### esp32s3

```sh
# Wifi credentials should be valid only if you plan to run the `light_eth` "ethernet" example.
# The `light_wifi` example gets your Wifi settings from the Matter Controller automatically.
export WIFI_SSID=foo
export WIFI_PASS=bar

cargo install espup
espup update

cd esp
cargo +esp build --target xtensa-esp32s3-none-elf --no-default-features --features esp32s3,wifi

# Replace `light_wifi` with `light_eth` below to flash the "Ethernet" example
espflash flash target/xtensa-esp32s3-none-elf/debug/light_wifi --baud 1500000
espflash monitor --elf target/xtensa-esp32s3-none-elf/debug/light_wifi
```

#### esp32c3

```sh
# Wifi credentials should be valid only if you plan to run the `light_eth` "ethernet" example.
# The `light_wifi` example gets your Wifi settings from the Matter Controller automatically.
export WIFI_SSID=foo
export WIFI_PASS=bar

cd esp
cargo +nightly build --target riscv32imc-unknown-none-elf --no-default-features --features esp32c3,wifi

# Replace `light_wifi` with `light_eth` below to flash the "Ethernet" example
espflash flash target/riscv32imc-unknown-none-elf/debug/light_wifi --baud 1500000
espflash monitor --elf target/riscv32imc-unknown-none-elf/debug/light_wifi
```

#### esp32c6

```sh
# Wifi credentials should be valid only if you plan to run the `light_eth` "ethernet" example.
# The `light_wifi` example gets your Wifi settings from the Matter Controller automatically.
export WIFI_SSID=foo
export WIFI_PASS=bar

cd esp
cargo +nightly build --target riscv32imac-unknown-none-elf --no-default-features --features esp32c6,wifi

# Replace `light_wifi` with `light_eth` below to flash the "Ethernet" example
# Replace `light_wifi` with `light_thread` below to flash the Ethernet example (you'll need an esp32c6 or esp32h2)
espflash flash target/riscv32imac-unknown-none-elf/debug/light_wifi --baud 1500000
espflash monitor --elf target/riscv32imac-unknown-none-elf/debug/light_wifi
```

### Nordic nRF52840 DK

```sh
# Thread dataset should be valid only if you plan to run the `light_eth` "ethernet" example.
# Get the Thread dataset from a commissioned device (`ot cli dataset active -x`) 
# or from your Thread Border Router and set the environment variable
export THREAD_DATASET="000003..."

cd nrf

# Replace `light_thread` with `light_eth` below to flash the Ethernet example
cargo run --bin light_thread
```

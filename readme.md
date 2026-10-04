# Environment Node (TML-EN1)

**English** | [中文](README.zh-CN.md)



[Quick Start](#quick-start) · [Hardware](#hardware) · [Zigbee Interface](#zigbee-interface) · [Known Limitations](#known-limitations)



> TML-EN1 is a DIY Zigbee environmental sensor node for smart home systems. It measures temperature, humidity, and barometric pressure, then reports the readings to Zigbee2MQTT or Home Assistant for use in home automation. The device is optimized for continuous low-power operation, so the battery rarely needs replacing.



![TML-EN1 assembled device](docs/device.jpeg)



## Note

This is an experimental project with rough edges. You may need to modify the code or the hardware to make it work in your setup.



## Introduction

TML-EN1 (Thingsmaking Link - Environment Node 1) is based on the ESP32-H2 SoC. It collects measurements from its environmental sensors and reports them over Zigbee for use in home automation. The design is optimized for continuous low-power operation without frequent maintenance or battery replacement.

The node uses only standard Zigbee Cluster Library (ZCL) clusters. Compatible Zigbee coordinators, such as Home Assistant ZHA and Zigbee2MQTT, receive all sensor readings directly, with no custom protocol or proprietary gateway.



## Demo

![Zigbee2MQTT device page: live values of the four standard clusters](docs/z2m-exposes.png)

*Zigbee2MQTT device page: live temperature, humidity, pressure, battery, and link quality.*

![Home Assistant device page: recognised as Thingsmaking / TML-EN1](docs/ha-device.png)

*Home Assistant device page: the coordinator generates the definition automatically, with the manufacturer and model recognised correctly.*

![Home Assistant history: 44 hours of continuous data](docs/ha-history.png)

*Home Assistant history: a continuous 44-hour temperature trace (October 3 to 4) with no missing data.*



## Hardware

### Bill of Materials

| Category | Model / Specification | Quantity | Remarks |
|---|---|---|---|
| MCU | ESP32-H2 Supermini | 1 | ESP32-H2 Supermini development board with integrated Li-ion charging management |
| Sensor | BME280 | 1 | I2C interface, default address `0x76` |
| Battery | 18650 Li-ion battery | 1 | With matching battery holder and 1S Li-ion protection board |
| Voltage sensing | Divider resistors | 2 | 10 kΩ |
| Voltage sensing | ADC input filter capacitor | 1 | 100 nF ceramic capacitor |
| Enclosure / Mount | 〔To be added later〕 | | |



### Pinout and Wiring

| Function | Pin | Parameters |
|---|---|---|
| I2C SDA | GPIO10 | Master mode, internal pull-up enabled |
| I2C SCL | GPIO11 | 100 kHz |
| Battery voltage sensing | GPIO1 | ADC1 channel 0, 12 dB attenuation, curve-fitting calibration |



### Power Supply and Consumption

A single 18650 Li-ion battery powers the device. The battery voltage is fed into the ADC through a resistor divider. The remaining battery percentage is calculated by piecewise linear interpolation over a discharge curve.

The firmware enables the following power-saving configurations:

| Mechanism | Configuration |
|---|---|
| Dynamic frequency scaling | Maximum 96 MHz, minimum 32 MHz |
| Automatic light sleep | Enabled |
| FreeRTOS tickless idle | Enabled |
| 802.15.4 radio sleep | Enabled |
| Flash power-down during sleep | Enabled |

The Zigbee stack disables idle reception (`rx_on_when_idle = false`), so the node wakes the radio only when it needs to transmit or receive.



## Zigbee Interface

### Device Identity

| Parameter | Value |
|---|---|
| Manufacturer Name | `Thingsmaking` |
| Model Identifier | `TML-EN1` |
| Endpoint | 1 |
| Application Profile ID | Home Automation (`0x0104`) |
| Device Type | Temperature Sensor (`0x0302`) |
| Power Source | Battery (Basic cluster `PowerSource`) |

> Note: ZCL string attributes carry a length prefix. In the firmware they are written as `"\x0C""Thingsmaking"` and `"\x07""TML-EN1"`, where `\x0C` and `\x07` are the byte lengths of the two strings.



### Endpoints and Clusters

Endpoint 1 implements the following server clusters:

| Cluster | Cluster ID | Attribute | Data Type | Unit / Resolution | Value Range |
|---|---|---|---|---|---|
| Basic | `0x0000` | ManufacturerName / ModelIdentifier / PowerSource | String / Enum | — | — |
| Power Configuration | `0x0001` | BatteryPercentageRemaining | `U8` | 0.5 % | 0–200 |
| Temperature Measurement | `0x0402` | MeasuredValue | `S16` | 0.01 °C | −1000–5000 (−10–50 °C) |
| Pressure Measurement | `0x0403` | MeasuredValue | `S16` | 0.1 kPa | 300–1100 (30–110 kPa) |
| Relative Humidity Measurement | `0x0405` | MeasuredValue | `U16` | 0.01 % | 0–10000 (0–100 %) |

After joining the network, the node creates a binding to the coordinator (short address `0x0000`) for each of the four measurement clusters, then sends periodic attribute reports according to the reporting configuration.



### Reporting Strategy

All four measurements share the same reporting parameters:

| Attribute | Min Interval | Max Interval | Reportable Change |
|---|---|---|---|
| Temperature | 60 s | 600 s | 20 (0.2 °C) |
| Humidity | 60 s | 600 s | 300 (3 %RH) |
| Pressure | 60 s | 600 s | 1 (0.1 kPa) |
| Battery | 60 s | 600 s | 4 (2 %) |

Sampling runs on a 60-second tick: a Zigbee stack timer starts the acquisition task every 60 seconds, and the task deletes itself once it finishes. The max interval guarantees that data is refreshed at least every 10 minutes even when the environment stays constant.



### Network Behavior

| Item | Configuration |
|---|---|
| Device Role | Zigbee End Device (ZED) |
| Preferred Channel | 11 |
| End Device Aging Timeout | 64 minutes |
| Keep-alive | 60 seconds |
| Commissioning Strategy | Factory-fresh devices start network steering automatically and retry every 1 second on failure; already-joined devices restore their reporting configuration after a reboot |



## Quick Start

### Requirements

| Requirement | Version |
|---|---|
| ESP-IDF | 6.0.0 |
| Target Chip | esp32h2 |
| Flash Size | 2 MB |
| Dependencies | `espressif/bme280`, `espressif/esp-zigbee-lib ~1.6.0`, `espressif/esp-zboss-lib ~1.6.0` |

`sdkconfig.defaults` provides the base configuration, and the IDF Component Manager downloads the dependencies during the build.



### Build and Flash

```bash
# In the ESP-IDF environment
idf.py set-target esp32h2
idf.py build
idf.py -p <PORT> flash monitor
```



### Partition Table

The project uses a custom partition table (`partitions.csv`):

| Name | Type | Offset | Size | Purpose |
|---|---|---|---|---|
| `nvs` | data / nvs | `0x9000` | 24 KB | Non-volatile storage |
| `phy_init` | data / phy | `0xf000` | 4 KB | RF calibration data |
| `factory` | app / factory | `0x10000` | 900 KB | Application |
| `zb_storage` | data / fat | `0xf1000` | 16 KB | Zigbee stack storage (network parameters, etc.) |
| `zb_fct` | data / fat | `0xf5000` | 1 KB | Zigbee stack factory data |

The Zigbee stack owns `zb_storage` and `zb_fct`; deleting them or shrinking them prevents the stack from starting. OTA updates are not supported yet.



## Usage

### Joining the Network

1. Open the network for joining on the coordinator (in Home Assistant ZHA, select "Add device"; in Zigbee2MQTT, use `Permit join`).
2. Power on the node. The serial console prints `System starting...`, followed by the peripheral initialization messages.
3. A node that is not yet on a network starts network steering automatically. A `Joined network successfully.` line means the join succeeded, followed by `Automatic reporting configured.` and `Binding configured.`.



### Normal Operation

The node prints one line of readings every 60 seconds:

```
Readings: Temp: 22.31 C, Humidity: 45.60 %, Pressure: 101.32 kPa, Bat: 87 %
```

If the log stops updating, the sampling task is not running. Check the network join status first (look for `Joined network successfully.`), then check whether every peripheral initialized successfully at boot.



### Replacing the Battery

Power the device down, replace the battery, and power it back on. The network parameters persist in the `zb_storage` partition, so the node resumes reporting after the reboot without joining the network again.



## Known Limitations

- **No OTA updates.** The firmware must be re-flashed over USB or UART.
- **No custom device definition (quirk / external converter).** The coordinator auto-generates a usable definition and reporting works, but entity naming, icons, and units follow that generated result and cannot be customised.
- **The battery percentage is an approximation.**
- **No fallback when a sensor read fails.** `ESP_ERROR_CHECK` in the sampling path triggers an assertion and reboots the MCU if the sensor does not respond, so the node cannot keep reporting battery level while the sensor is faulty.
- **No button for factory reset or re-pairing.** Resetting to factory defaults or joining a different network requires erasing the flash manually with `idf.py`.



## Related Projects and Acknowledgments

- The sensor driver is the official Espressif component [`espressif/bme280`](https://components.espressif.com/components/espressif/bme280); the initialization routine in this repository is adapted from it to use Forced Mode.
- The Zigbee stack is `esp-zigbee-lib` with `esp-zboss-lib`.



## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for the full text.

You are free to use, modify, and distribute the code in this repository, provided that you keep the original copyright notice and license text.

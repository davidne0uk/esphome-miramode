# esphome-miramode

ESPHome external component for controlling Mira Mode digital showers via BLE.

## Alexa, turn on the shower!

This project brings native Mira Mode support to [ESPHome](https://esphome.io),
allowing an ESP32 to act as a BLE gateway and expose the shower as first-class
entities in Home Assistant — outlet switches, a temperature sensor, a target
temperature control, and a pairing button — which can then be integrated with
Alexa or Google Home.

Multiple showers are supported; each `miramode:` entry maintains its own BLE
connection and independently stored credentials.

*Disclaimer: this project contains only the results of personal experiments,
use at your own risk!*

## Requirements

1. A Mira Mode shower or bath fill. Tested models (from python-miramode):
    * *Dual Outlet Shower - 1874.203*
    * *Dual Outlet Bath Fill - 1874.205*
2. An ESP32 board supported by ESPHome (e.g. `esp32dev`)
3. [ESPHome](https://esphome.io) 2025.5 or later with ESP-IDF framework

## Installation

### Option 1 — Git source (recommended)

Add to your ESPHome device YAML:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/davidne0uk/esphome-miramode
      ref: v0.1.0
    components: [miramode]
```

### Option 2 — Local source

Clone this repository and copy the `components/miramode` directory alongside
your ESPHome YAML files, then reference it:

```yaml
external_components:
  - source:
      type: local
      path: components
```

## Configuration

```yaml
esp32:
  board: esp32dev
  framework:
    type: esp-idf

esp32_ble_tracker:

ble_client:
  - mac_address: "AA:BB:CC:DD:EE:FF"
    id: shower_ble

miramode:
  - id: my_shower
    name: "My Shower"
    client_name: "ESPHome"       # Name shown during pairing
    ble_client_id: shower_ble
    update_interval: 30s
    outlet1:
      name: "Shower Outlet 1"
    outlet2:
      name: "Shower Outlet 2"
    actual_temperature:
      name: "Shower Actual Temperature"
    target_temperature:
      name: "Shower Target Temperature"
      min_value: 20.0
      max_value: 48.0
      step: 0.5
    pair_button:
      name: "Pair Shower"
```

See [`example/example.yaml`](example/example.yaml) for a complete two-shower
configuration.

## Pairing

The shower must be paired before it will respond to commands:

1. Put the shower into pairing mode by pressing and holding the outlet button
   for 5 seconds until the LED flashes.
2. Press the **Pair** button entity in Home Assistant.
3. The ESP32 will exchange credentials with the shower and store them to flash.
   Pairing persists across reboots.

## Finding your shower's MAC address

Use any BLE scanner app (e.g. nRF Connect) and look for devices advertising
the service UUID `bccb0001-ca66-11e5-88a4-0002a5d5c51b`, or devices with a
name starting with *Mira*.

## Entities

| Entity | Type | Description |
|--------|------|-------------|
| Outlet 1 | Switch | Turn outlet 1 on/off |
| Outlet 2 | Switch | Turn outlet 2 on/off |
| Actual Temperature | Sensor | Current water temperature (°C) |
| Target Temperature | Number | Desired water temperature (°C) |
| Pair | Button | Initiate BLE pairing with the shower |

## Acknowledgements

This project is based on the protocol implementation from
[python-miramode](https://github.com/alexpilotti/python-miramode) by
Alessandro Pilotti — without that work this component would not exist.

Many thanks also to Nigel Hannam for his excellent work in documenting the BLE
protocol: https://github.com/nhannam/shower-controller-documentation

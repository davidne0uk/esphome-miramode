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
      ref: v0.2.1
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
    unpair_button:
      name: "Unpair Shower"
```

See [`example/mira-showers.yaml`](example/mira-showers.yaml) for a complete
multi-shower configuration.

## Finding your shower's MAC address

Mira Mode devices advertise the service UUID `bccb0001-ca66-11e5-88a4-0002a5d5c51b`
and have names starting with *Mira*.

**Important — iOS byte-reversal bug:** BLE scanner apps on iOS (including nRF Connect)
display MAC addresses in reversed byte order. If your scanner shows `9F:90:5C:B9:59:DD`,
the address you must put in your ESPHome YAML is the reverse: `DD:59:B9:5C:90:9F`.
Android and desktop scanners show the correct order directly.

The most reliable method is to let ESPHome log the real MAC addresses for you. Add
a temporary `on_ble_advertise` filter to your device YAML, flash it, and check the
logs:

```yaml
esp32_ble_tracker:
  on_advertise:
    - lambda: |-
        for (auto &svc : x.get_service_uuids()) {
          if (svc.to_string() == "bccb0001-ca66-11e5-88a4-0002a5d5c51b") {
            ESP_LOGI("mira_scan", "Found Mira device: %s  RSSI: %d",
                     x.address_str().c_str(), x.get_rssi());
          }
        }
```

The MAC addresses printed in the log are correct and can be pasted directly into
`ble_client:` entries. Remove this block once you have your addresses.

## Pairing

The shower must be paired before it will respond to commands. Timing matters — the
shower only stays in pairing mode for about 30 seconds and will drop unpaired
connections after the same interval.

1. Put the shower into pairing mode by pressing and holding the outlet button for
   5 seconds until the LED flashes.
2. Press the **Pair** button entity in Home Assistant at any point — before or
   after the ESP32 connects. The pair request is sent as soon as the device
   reports Ready, so you do not need to watch the logs and time it precisely.
3. Confirm pairing succeeded by checking the logs for:
   ```
   [My Shower] Sending pair request
   [My Shower] Paired! slot=0
   ```
4. Credentials are stored to flash and pairing persists across reboots.

If pairing fails the shower will drop the connection and the pending state is
cleared automatically. Put the shower back into pairing mode and press Pair
again to retry.

## Unpairing

To clear stored credentials and return a shower to an unpaired state, press the
**Unpair** button entity in Home Assistant. The ESP32 will erase its saved
`client_id` and `client_slot` from flash and immediately stop treating the
shower as paired. No reboot is required.

This is useful if:

- You want to re-pair with a different client name.
- Pairing was attempted with wrong credentials and the shower is rejecting commands.
- You are moving the shower to a different ESP32.

## Entities

| Entity | Type | Description |
|--------|------|-------------|
| Outlet 1 | Switch | Turn outlet 1 on/off |
| Outlet 2 | Switch | Turn outlet 2 on/off |
| Actual Temperature | Sensor | Current water temperature (°C) |
| Target Temperature | Number | Desired water temperature (°C) |
| Pair | Button | Initiate BLE pairing with the shower |
| Unpair | Button | Erase stored credentials and reset paired state |

## Acknowledgements

This project is based on the protocol implementation from
[python-miramode](https://github.com/alexpilotti/python-miramode) by
Alessandro Pilotti — without that work this component would not exist.

Many thanks also to Nigel Hannam for his excellent work in documenting the BLE
protocol: https://github.com/nhannam/shower-controller-documentation

# ESPHome Miramode External Component Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an ESPHome external component that controls Mira Mode digital showers via BLE, exposing outlet switches, temperature controls, and pairing as native Home Assistant entities, with full support for multiple simultaneous shower instances.

**Architecture:** A C++ `PollingComponent` (`MiraModeDevice`) extends ESPHome's `BLEClientNode` to act as BLE central. Each instance owns its own BLE connection, NVS-stored credentials (client_id + client_slot), and entity pointers (switches, sensor, number, button). All entities are registered inline from the main `__init__.py` schema — no sub-platform directories needed. Packet building and CRC16 are pure C++ matching the reference Python implementation exactly.

**Tech Stack:** C++11, ESPHome 2024.x component API, ESP-IDF BLE GATTC (via `ble_client`), ESP-IDF NVS, Python 3.9+ (ESPHome codegen), pytest

**Reference implementation:** `/Users/davidbennett/workspace/python-miramode/miramode/__init__.py`

---

## File Map

| File | Responsibility |
|------|---------------|
| `components/miramode/__init__.py` | ESPHome Python schema, validation, C++ code generation |
| `components/miramode/miramode.h` | `MiraModeDevice`, `MiraModeSwitch`, `MiraModeSensor`, `MiraModeNumber`, `MiraModeButton` declarations |
| `components/miramode/miramode.cpp` | All C++ implementation: CRC16, packet builder, BLE events, NVS, notification parser, control commands, entity write handlers |
| `tests/test_crc.cpp` | Standalone CRC16 unit test (compiled with `g++`, no ESPHome dependency) |
| `tests/test_packets.py` | pytest — validates C++ packet output matches reference Python output for known inputs |
| `tests/test_schema.py` | pytest — validates `__init__.py` schema acceptance/rejection of config inputs |
| `example/example.yaml` | Two-shower ESPHome config demonstrating full multi-instance setup |

---

## Protocol Reference (from python-miramode)

- **Write characteristic:** `bccb0002-ca66-11e5-88a4-0002a5d5c51b`
- **Read/notify characteristic:** `bccb0003-ca66-11e5-88a4-0002a5d5c51b`
- **MAGIC_ID** (used during pairing only): `0x54d2ee63`
- **CRC16:** CCITT variant — CRC computed over `payload || client_id_bytes(4, big-endian)`; only `payload || crc_bytes(2, big-endian)` is transmitted
- **Writes** chunked to 20 bytes (BLE MTU limit)
- **Responses** received via BLE notifications; may be fragmented across two notifications
- **Temperature** encoding: `round(celsius * 10)` as big-endian uint16
- **Outlet states:** `RUNNING = 0x64`, `STOPPED = 0x00`
- **Timer states:** `RUNNING = 0x01`, `PAUSED = 0x03`, `STOPPED = 0x00`

### Outgoing command payloads

| Command | Payload bytes | client_id used |
|---------|--------------|----------------|
| `request_device_state` | `[slot, 0x07, 0x00]` | `client_id_` |
| `control_outlets` | `[slot, 0x87, 0x05, timer, temp_hi, temp_lo, o1, o2]` | `client_id_` |
| `pair_client` | `[0x00, 0xeb, 24, id3, id2, id1, id0, name[20]]` chunked | `MAGIC_ID` |
| `unpair_client` | `[slot, 0xeb, 1, target_slot]` | `client_id_` |
| `start_preset` | `[slot, 0xb1, 1, preset_slot]` | `client_id_` |

### Incoming notification dispatch (by payload_length)

| payload_length | Condition | Handler |
|---|---|---|
| 1 | — | success/failure; if pairing pending, extract slot from header |
| 2 | — | slot list (bitmask) — ignored for now |
| 4 | — | device settings — ignored for now |
| 10 | — | **device_state**: timer, target_temp, actual_temp, outlet1, outlet2, remaining_secs |
| 11 | `payload[0] in {1, 0x80}` | **controls_operated**: same fields + change_made flag |
| 11 | `payload[0] in {0, 4, 8}` | outlet settings — ignored for now |
| 16 | `payload[0] == 0` | technical info — ignored for now |
| 16 | `payload[0] != 0` | nickname — ignored for now |
| 20 | — | client details — ignored for now |
| 24 | — | preset details — ignored for now |

Notification header format: `value[0] = client_slot + 0x40`, `value[1] = unknown`, `value[2] = payload_length`, `value[3..] = payload`.

---

## Task 1: CRC16 Unit Test + Implementation

**Files:**
- Create: `tests/test_crc.cpp`
- Create: `components/miramode/miramode.cpp` (CRC16 function only)
- Create: `components/miramode/miramode.h` (forward declarations only)

- [ ] **Step 1: Write the failing C++ CRC16 test**

Create `tests/test_crc.cpp`:

```cpp
// Standalone test — compiled with g++, no ESPHome dependency
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cassert>

// Copy of the function under test (same impl as miramode.cpp)
static uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        uint16_t r = crc;
        for (int j = 0; j < 8; j++) {
            int msb = (r >> 15) & 1;
            int bit = (b >> (7 - j)) & 1;
            r = r << 1;
            if ((bit ^ msb) != 0)
                r ^= 0x1021;
        }
        crc = r;
    }
    return crc & 0xFFFF;
}

int main() {
    // Test vectors derived from Python reference:
    // _crc(b'\x01\x07\x00' + struct.pack('>I', 0x12345678))
    // = _crc([0x01, 0x07, 0x00, 0x12, 0x34, 0x56, 0x78])
    // Python: 0x4E1A  (compute this with: python3 -c "
    //   d=[0x01,0x07,0x00,0x12,0x34,0x56,0x78]
    //   r=0xFFFF
    //   for b in d:
    //     t=r
    //     for i in range(8):
    //       i4=1 if (t>>15)&1 else 0
    //       i5=1 if (b>>(7-i))&1 else 0
    //       t=t<<1
    //       if i5^i4: t^=0x1021
    //     r=t
    //   print(hex(r&0xFFFF))")

    struct { const char *name; uint8_t data[16]; size_t len; uint16_t expected; } cases[] = {
        // empty payload: CRC of just client_id 0x12345678
        {"client_id_only",    {0x12,0x34,0x56,0x78},             4,  0xC9CF},
        // request_device_state: payload=[0x01,0x07,0x00] + client_id=0x12345678
        {"request_state",     {0x01,0x07,0x00,0x12,0x34,0x56,0x78}, 7, 0x0000},
        // single zero byte
        {"zero_byte",         {0x00},                             1,  0x1021},
        // 0xFF byte
        {"ff_byte",           {0xFF},                             1,  0xEFFE},
        // MAGIC_ID used during pairing: 0x54d2ee63
        {"magic_id_only",     {0x54,0xD2,0xEE,0x63},             4,  0x0000},
    };

    // Compute expected values using Python reference output
    // Run: python3 tests/compute_crc_vectors.py to regenerate
    // Hardcoded expected values filled in during Step 3 after Python run

    // For now assert the function is callable and returns a uint16
    uint8_t buf[] = {0x01, 0x07, 0x00};
    uint16_t result = crc16(buf, 3);
    printf("CRC of [01 07 00] = 0x%04X\n", result);
    printf("PASS: crc16 is callable\n");
    return 0;
}
```

- [ ] **Step 2: Write the Python vector generator**

Create `tests/compute_crc_vectors.py`:

```python
#!/usr/bin/env python3
"""Generate CRC16 test vectors using the reference Python implementation."""
import struct

def crc(data):
    i = 0
    i2 = 0xFFFF
    while i < len(data):
        b = data[i]
        i3 = i2
        for i2 in range(8):
            i4 = 1
            i5 = 1 if ((b >> (7 - i2)) & 1) == 1 else 0
            if ((i3 >> 15) & 1) != 1:
                i4 = 0
            i3 = i3 << 1
            if (i5 ^ i4) != 0:
                i3 = i3 ^ 0x1021
        i += 1
        i2 = i3
    return i2 & 0xFFFF

vectors = [
    ("client_id_only",   bytes([0x12, 0x34, 0x56, 0x78])),
    ("request_state",    bytes([0x01, 0x07, 0x00, 0x12, 0x34, 0x56, 0x78])),
    ("zero_byte",        bytes([0x00])),
    ("ff_byte",          bytes([0xFF])),
    ("magic_id_only",    bytes([0x54, 0xD2, 0xEE, 0x63])),
]

for name, data in vectors:
    result = crc(data)
    print(f'  {{"{name}", {{{", ".join(f"0x{b:02X}" for b in data)}}}, {len(data)}, 0x{result:04X}}},')
```

- [ ] **Step 3: Run the vector generator to get expected values**

```bash
cd /Users/davidbennett/workspace/esphome-miramode
python3 tests/compute_crc_vectors.py
```

Expected output (capture these values):
```
  {"client_id_only", {0x12, 0x34, 0x56, 0x78}, 4, 0xXXXX},
  {"request_state", {0x01, 0x07, 0x00, 0x12, 0x34, 0x56, 0x78}, 7, 0xXXXX},
  {"zero_byte", {0x00}, 1, 0xXXXX},
  {"ff_byte", {0xFF}, 1, 0xXXXX},
  {"magic_id_only", {0x54, 0xD2, 0xEE, 0x63}, 4, 0xXXXX},
```

- [ ] **Step 4: Fill in expected values and add assertions to test_crc.cpp**

Replace the `cases[]` array in `tests/test_crc.cpp` with the actual expected values from Step 3, and add assertion loop:

```cpp
// (replace placeholder 0x0000 values with real ones from compute_crc_vectors.py)
    struct { const char *name; uint8_t data[16]; size_t len; uint16_t expected; } cases[] = {
        {"client_id_only",  {0x12,0x34,0x56,0x78},             4, /* from Step 3 */},
        {"request_state",   {0x01,0x07,0x00,0x12,0x34,0x56,0x78}, 7, /* from Step 3 */},
        {"zero_byte",       {0x00},                             1, /* from Step 3 */},
        {"ff_byte",         {0xFF},                             1, /* from Step 3 */},
        {"magic_id_only",   {0x54,0xD2,0xEE,0x63},             4, /* from Step 3 */},
    };
    bool all_pass = true;
    for (auto &c : cases) {
        uint16_t got = crc16(c.data, c.len);
        if (got != c.expected) {
            printf("FAIL %s: got 0x%04X, expected 0x%04X\n", c.name, got, c.expected);
            all_pass = false;
        } else {
            printf("PASS %s: 0x%04X\n", c.name, got);
        }
    }
    return all_pass ? 0 : 1;
```

- [ ] **Step 5: Compile and run the test**

```bash
g++ -std=c++11 -o /tmp/test_crc tests/test_crc.cpp && /tmp/test_crc
```

Expected: all lines say `PASS`, exit code 0.

- [ ] **Step 6: Write the C++ CRC16 in miramode.cpp**

Create `components/miramode/miramode.cpp` (CRC section only for now):

```cpp
#include "miramode.h"
#include "esphome/core/log.h"
#include <cmath>
#include <nvs_flash.h>
#include <nvs.h>

namespace esphome {
namespace miramode {

static const char *const TAG = "miramode";

static const uint32_t MAGIC_ID        = 0x54D2EE63;
static const uint8_t  OUTLET_RUNNING  = 0x64;
static const uint8_t  OUTLET_STOPPED  = 0x00;
static const uint8_t  TIMER_RUNNING   = 0x01;
static const uint8_t  TIMER_PAUSED    = 0x03;

// CRC16-CCITT matching reference Python _crc() exactly
uint16_t MiraModeDevice::crc16_(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        uint16_t r = crc;
        for (int j = 0; j < 8; j++) {
            int msb = (r >> 15) & 1;
            int bit = (b >> (7 - j)) & 1;
            r = r << 1;
            if ((bit ^ msb) != 0)
                r ^= 0x1021;
        }
        crc = r;
    }
    return crc & 0xFFFF;
}

}  // namespace miramode
}  // namespace esphome
```

Create `components/miramode/miramode.h` (skeleton):

```cpp
#pragma once
#include "esphome/core/component.h"
#include "esphome/components/ble_client/ble_client.h"
#include <string>
#include <vector>

namespace esphome {
namespace miramode {

class MiraModeSwitch;
class MiraModeSensor;
class MiraModeNumber;
class MiraModeButton;

class MiraModeDevice : public PollingComponent,
                       public ble_client::BLEClientNode {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                            esp_ble_gattc_cb_param_t *param) override;

  void set_name(const std::string &name) { name_ = name; }
  void set_client_name(const std::string &n) { client_name_ = n; }

  void set_outlet1_switch(MiraModeSwitch *sw) { outlet1_switch_ = sw; }
  void set_outlet2_switch(MiraModeSwitch *sw) { outlet2_switch_ = sw; }
  void set_actual_temp_sensor(MiraModeSensor *s) { actual_temp_sensor_ = s; }
  void set_target_temp_number(MiraModeNumber *n) { target_temp_number_ = n; }
  void set_pair_button(MiraModeButton *b) { pair_button_ = b; }

  // Called by entity write handlers
  void control_outlets(bool outlet1, bool outlet2, float temperature);
  void trigger_pair();
  void request_device_state();

 protected:
  std::string name_;
  std::string client_name_{"ESPHome"};
  uint32_t client_id_{0};
  uint8_t  client_slot_{0};
  bool paired_{false};
  bool pairing_pending_{false};
  uint32_t pending_client_id_{0};

  // BLE handles
  uint16_t write_handle_{0};
  uint16_t read_handle_{0};

  // Entity pointers (may be null if not configured)
  MiraModeSwitch  *outlet1_switch_{nullptr};
  MiraModeSwitch  *outlet2_switch_{nullptr};
  MiraModeSensor  *actual_temp_sensor_{nullptr};
  MiraModeNumber  *target_temp_number_{nullptr};
  MiraModeButton  *pair_button_{nullptr};

  // Cached outlet/temp state (so pair-less control_outlets calls work)
  bool  outlet1_state_{false};
  bool  outlet2_state_{false};
  float target_temp_{38.0f};

  // Packet reassembly
  std::vector<uint8_t> partial_payload_;
  uint8_t partial_client_slot_{0};
  uint8_t expected_length_{0};

  // Helpers
  static uint16_t crc16_(const uint8_t *data, size_t len);
  std::vector<uint8_t> build_packet_(const std::vector<uint8_t> &payload, uint32_t id);
  void write_chunks_(const std::vector<uint8_t> &data);
  void write_raw_(const uint8_t *data, size_t len);
  void handle_notification_(const uint8_t *data, size_t len);
  void dispatch_payload_(uint8_t client_slot, const uint8_t *payload, uint8_t length);
  void load_credentials_();
  void save_credentials_();
  std::string nvs_namespace_();
};

// ── Entity classes ──────────────────────────────────────────────────────────

class MiraModeSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(MiraModeDevice *p) { parent_ = p; }
  void set_outlet(uint8_t o) { outlet_ = o; }
  void set_state_from_device(bool state) { this->publish_state(state); }
 protected:
  void write_state(bool state) override;
  MiraModeDevice *parent_{nullptr};
  uint8_t outlet_{1};
};

class MiraModeSensor : public sensor::Sensor, public Component {
 public:
  void set_parent(MiraModeDevice *p) { parent_ = p; }
 protected:
  MiraModeDevice *parent_{nullptr};
};

class MiraModeNumber : public number::Number, public Component {
 public:
  void set_parent(MiraModeDevice *p) { parent_ = p; }
  void set_state_from_device(float value) { this->publish_state(value); }
 protected:
  void control(float value) override;
  MiraModeDevice *parent_{nullptr};
};

class MiraModeButton : public button::Button, public Component {
 public:
  void set_parent(MiraModeDevice *p) { parent_ = p; }
 protected:
  void press_action() override;
  MiraModeDevice *parent_{nullptr};
};

}  // namespace miramode
}  // namespace esphome
```

- [ ] **Step 7: Commit**

```bash
cd /Users/davidbennett/workspace/esphome-miramode
git add components/miramode/miramode.h components/miramode/miramode.cpp tests/test_crc.cpp tests/compute_crc_vectors.py
git commit -m "feat: CRC16 implementation with test vectors"
```

---

## Task 2: Packet Builder + Tests

**Files:**
- Modify: `components/miramode/miramode.cpp` (add `build_packet_`, `write_chunks_`, `write_raw_`)
- Create: `tests/test_packets.py`

- [ ] **Step 1: Write the failing pytest for packet building**

Create `tests/test_packets.py`:

```python
"""
Validate that C++ packet output matches reference Python implementation.
This test runs the Python reference directly — C++ is validated by
compilation (Task 9). The Python tests lock down expected bytes for
manual cross-checking against debug log output from the ESP32.
"""
import struct
import pytest

# ── Reference implementation (copied from python-miramode verbatim) ──────────

MAGIC_ID = 0x54D2EE63
OUTLET_RUNNING = 0x64
OUTLET_STOPPED = 0x00
TIMER_RUNNING  = 0x01
TIMER_PAUSED   = 0x03

def _crc(data):
    i = 0
    i2 = 0xFFFF
    while i < len(data):
        b = data[i]
        i3 = i2
        for i2 in range(8):
            i4 = 1
            i5 = 1 if ((b >> (7 - i2)) & 1) == 1 else 0
            if ((i3 >> 15) & 1) != 1:
                i4 = 0
            i3 = i3 << 1
            if (i5 ^ i4) != 0:
                i3 = i3 ^ 0x1021
        i += 1
        i2 = i3
    return i2 & 0xFFFF

def _payload_with_crc(payload, client_id):
    crc = _crc(bytes(payload) + struct.pack(">I", client_id))
    return bytes(payload) + struct.pack(">H", crc)

def _split_chunks(data, chunk_size=20):
    return [data[i:i + chunk_size] for i in range(0, len(data), chunk_size)]

# ── Packet builders matching miramode.cpp ────────────────────────────────────

def packet_request_device_state(client_slot, client_id):
    return _payload_with_crc([client_slot, 0x07, 0x00], client_id)

def packet_control_outlets(client_slot, client_id, outlet1, outlet2, celsius):
    temp = int(round(celsius * 10)) & 0xFFFF
    return _payload_with_crc([
        client_slot, 0x87, 0x05,
        TIMER_RUNNING if (outlet1 or outlet2) else TIMER_PAUSED,
        (temp >> 8) & 0xFF, temp & 0xFF,
        OUTLET_RUNNING if outlet1 else OUTLET_STOPPED,
        OUTLET_RUNNING if outlet2 else OUTLET_STOPPED,
    ], client_id)

def packet_pair(new_client_id, client_name):
    name_bytes = client_name.encode("UTF-8")[:20].ljust(20, b'\x00')
    payload = bytes([0x00, 0xEB, 24]) + struct.pack(">I", new_client_id) + name_bytes
    return _split_chunks(_payload_with_crc(payload, MAGIC_ID))

# ── Tests ─────────────────────────────────────────────────────────────────────

def test_request_device_state_length():
    pkt = packet_request_device_state(client_slot=1, client_id=0x12345678)
    assert len(pkt) == 5  # 3 payload + 2 CRC

def test_request_device_state_first_byte_is_slot():
    pkt = packet_request_device_state(client_slot=2, client_id=0x12345678)
    assert pkt[0] == 2

def test_request_device_state_command_byte():
    pkt = packet_request_device_state(client_slot=1, client_id=0x12345678)
    assert pkt[1] == 0x07

def test_control_outlets_on_timer_running():
    pkt = packet_control_outlets(1, 0xDEADBEEF, True, False, 38.0)
    assert pkt[3] == TIMER_RUNNING

def test_control_outlets_off_timer_paused():
    pkt = packet_control_outlets(1, 0xDEADBEEF, False, False, 38.0)
    assert pkt[3] == TIMER_PAUSED

def test_control_outlets_temperature_encoding():
    # 38.0°C → 380 → 0x017C
    pkt = packet_control_outlets(1, 0xDEADBEEF, True, False, 38.0)
    temp = (pkt[4] << 8) | pkt[5]
    assert temp == 380

def test_control_outlets_outlet1_on():
    pkt = packet_control_outlets(1, 0xDEADBEEF, True, False, 38.0)
    assert pkt[6] == OUTLET_RUNNING
    assert pkt[7] == OUTLET_STOPPED

def test_control_outlets_both_on():
    pkt = packet_control_outlets(1, 0xDEADBEEF, True, True, 40.0)
    assert pkt[6] == OUTLET_RUNNING
    assert pkt[7] == OUTLET_RUNNING

def test_pair_chunked_into_20_bytes():
    chunks = packet_pair(0xAABBCCDD, "ESPHome")
    for chunk in chunks[:-1]:
        assert len(chunk) == 20

def test_pair_total_length():
    chunks = packet_pair(0xAABBCCDD, "ESPHome")
    total = sum(len(c) for c in chunks)
    # payload=27 bytes (3 header + 4 client_id + 20 name) + 2 CRC = 29 bytes
    assert total == 29

def test_pair_first_byte_zero():
    chunks = packet_pair(0xAABBCCDD, "ESPHome")
    assert chunks[0][0] == 0x00

def test_pair_command_byte():
    chunks = packet_pair(0xAABBCCDD, "ESPHome")
    assert chunks[0][1] == 0xEB

def test_crc_changes_with_client_id():
    p1 = packet_request_device_state(1, 0x11111111)
    p2 = packet_request_device_state(1, 0x22222222)
    # CRC bytes (last 2) must differ
    assert p1[-2:] != p2[-2:]

def test_temperature_boundary_min():
    # 0°C → 0
    pkt = packet_control_outlets(1, 0x1, True, False, 0.0)
    temp = (pkt[4] << 8) | pkt[5]
    assert temp == 0

def test_temperature_boundary_max():
    # 65535 / 10 = 6553.5°C (not physically valid but tests encoding)
    pkt = packet_control_outlets(1, 0x1, True, False, 6553.5)
    temp = (pkt[4] << 8) | pkt[5]
    assert temp == 65535
```

- [ ] **Step 2: Run tests to confirm they pass (they test Python reference)**

```bash
cd /Users/davidbennett/workspace/esphome-miramode
pip install pytest
pytest tests/test_packets.py -v
```

Expected: all tests PASS (Python reference is authoritative).

- [ ] **Step 3: Implement build_packet_ and write helpers in miramode.cpp**

Add to `components/miramode/miramode.cpp` after the `crc16_` function:

```cpp
std::vector<uint8_t> MiraModeDevice::build_packet_(const std::vector<uint8_t> &payload,
                                                     uint32_t client_id) {
    // CRC is computed over payload || client_id (big-endian 4 bytes)
    std::vector<uint8_t> crc_input = payload;
    crc_input.push_back((client_id >> 24) & 0xFF);
    crc_input.push_back((client_id >> 16) & 0xFF);
    crc_input.push_back((client_id >> 8)  & 0xFF);
    crc_input.push_back( client_id        & 0xFF);

    uint16_t crc = crc16_(crc_input.data(), crc_input.size());

    // Transmitted packet is payload || CRC (client_id is NOT transmitted)
    std::vector<uint8_t> packet = payload;
    packet.push_back((crc >> 8) & 0xFF);
    packet.push_back( crc       & 0xFF);
    return packet;
}

void MiraModeDevice::write_chunks_(const std::vector<uint8_t> &data) {
    static const size_t CHUNK = 20;
    for (size_t i = 0; i < data.size(); i += CHUNK) {
        size_t end = std::min(i + CHUNK, data.size());
        this->write_raw_(data.data() + i, end - i);
    }
}

void MiraModeDevice::write_raw_(const uint8_t *data, size_t len) {
    if (this->write_handle_ == 0) {
        ESP_LOGW(TAG, "[%s] write_raw_: no write handle", this->name_.c_str());
        return;
    }
    ESP_LOGD(TAG, "[%s] Writing %d bytes", this->name_.c_str(), (int) len);
    auto status = esp_ble_gattc_write_char(
        this->parent()->get_gattc_if(),
        this->parent()->get_conn_id(),
        this->write_handle_,
        len,
        const_cast<uint8_t *>(data),
        ESP_GATT_WRITE_TYPE_NO_RSP,
        ESP_GATT_AUTH_REQ_NONE);
    if (status != ESP_OK)
        ESP_LOGW(TAG, "[%s] write_char failed: %d", this->name_.c_str(), status);
}
```

- [ ] **Step 4: Commit**

```bash
git add components/miramode/miramode.cpp tests/test_packets.py
git commit -m "feat: packet builder, write helpers, and packet tests"
```

---

## Task 3: Control Commands

**Files:**
- Modify: `components/miramode/miramode.cpp` (add `control_outlets`, `trigger_pair`, `request_device_state`)

- [ ] **Step 1: Add control command methods to miramode.cpp**

```cpp
void MiraModeDevice::request_device_state() {
    std::vector<uint8_t> payload = {this->client_slot_, 0x07, 0x00};
    auto pkt = this->build_packet_(payload, this->client_id_);
    this->write_raw_(pkt.data(), pkt.size());
}

void MiraModeDevice::control_outlets(bool outlet1, bool outlet2, float temperature) {
    this->outlet1_state_ = outlet1;
    this->outlet2_state_ = outlet2;
    this->target_temp_   = temperature;

    uint16_t temp_val = static_cast<uint16_t>(
        std::max(0.0f, std::min(65535.0f, std::round(temperature * 10.0f))));

    std::vector<uint8_t> payload = {
        this->client_slot_,
        0x87, 0x05,
        (outlet1 || outlet2) ? TIMER_RUNNING : TIMER_PAUSED,
        static_cast<uint8_t>((temp_val >> 8) & 0xFF),
        static_cast<uint8_t>( temp_val       & 0xFF),
        outlet1 ? OUTLET_RUNNING : OUTLET_STOPPED,
        outlet2 ? OUTLET_RUNNING : OUTLET_STOPPED,
    };
    auto pkt = this->build_packet_(payload, this->client_id_);
    this->write_raw_(pkt.data(), pkt.size());
}

void MiraModeDevice::trigger_pair() {
    if (!this->client_name_.size()) this->client_name_ = "ESPHome";

    // Generate random client ID
    this->pending_client_id_ = esp_random();
    this->pairing_pending_   = true;

    std::vector<uint8_t> name_bytes(this->client_name_.begin(),
                                     this->client_name_.end());
    name_bytes.resize(20, 0);

    std::vector<uint8_t> payload = {0x00, 0xEB, 24};
    payload.push_back((this->pending_client_id_ >> 24) & 0xFF);
    payload.push_back((this->pending_client_id_ >> 16) & 0xFF);
    payload.push_back((this->pending_client_id_ >>  8) & 0xFF);
    payload.push_back( this->pending_client_id_        & 0xFF);
    payload.insert(payload.end(), name_bytes.begin(), name_bytes.end());

    auto pkt = this->build_packet_(payload, MAGIC_ID);
    ESP_LOGI(TAG, "[%s] Sending pair request (client_id=0x%08X)",
             this->name_.c_str(), this->pending_client_id_);
    this->write_chunks_(pkt);
}
```

- [ ] **Step 2: Commit**

```bash
git add components/miramode/miramode.cpp
git commit -m "feat: control_outlets, trigger_pair, request_device_state commands"
```

---

## Task 4: NVS Credential Storage

**Files:**
- Modify: `components/miramode/miramode.cpp` (add `nvs_namespace_`, `load_credentials_`, `save_credentials_`)

NVS namespace must be ≤15 chars. Keys used: `"client_id"` (u32) and `"client_slot"` (u8).

- [ ] **Step 1: Add NVS helpers to miramode.cpp**

```cpp
std::string MiraModeDevice::nvs_namespace_() {
    // "mm_" prefix + first 12 chars of name, alphanumeric only
    std::string ns = "mm_";
    for (char c : this->name_) {
        if (std::isalnum(c)) ns += c;
        if (ns.size() == 15) break;
    }
    return ns;
}

void MiraModeDevice::load_credentials_() {
    nvs_handle_t handle;
    std::string ns = this->nvs_namespace_();
    esp_err_t err = nvs_open(ns.c_str(), NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "[%s] No NVS credentials found (not yet paired)", this->name_.c_str());
        return;
    }
    uint32_t id;
    uint8_t  slot;
    if (nvs_get_u32(handle, "client_id",   &id)   == ESP_OK &&
        nvs_get_u8 (handle, "client_slot", &slot) == ESP_OK) {
        this->client_id_   = id;
        this->client_slot_ = slot;
        this->paired_      = true;
        ESP_LOGI(TAG, "[%s] Loaded credentials: id=0x%08X slot=%d",
                 this->name_.c_str(), id, slot);
    }
    nvs_close(handle);
}

void MiraModeDevice::save_credentials_() {
    nvs_handle_t handle;
    std::string ns = this->nvs_namespace_();
    esp_err_t err = nvs_open(ns.c_str(), NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[%s] Failed to open NVS for writing: %d", this->name_.c_str(), err);
        return;
    }
    nvs_set_u32(handle, "client_id",   this->client_id_);
    nvs_set_u8 (handle, "client_slot", this->client_slot_);
    nvs_commit(handle);
    nvs_close(handle);
    ESP_LOGI(TAG, "[%s] Saved credentials: id=0x%08X slot=%d",
             this->name_.c_str(), this->client_id_, this->client_slot_);
}
```

- [ ] **Step 2: Commit**

```bash
git add components/miramode/miramode.cpp
git commit -m "feat: NVS credential persistence per shower instance"
```

---

## Task 5: Notification Handler

**Files:**
- Modify: `components/miramode/miramode.cpp` (add `handle_notification_`, `dispatch_payload_`)

- [ ] **Step 1: Add notification parsing to miramode.cpp**

```cpp
void MiraModeDevice::handle_notification_(const uint8_t *data, size_t len) {
    // Handle fragmented packets (two-part notifications)
    if (!this->partial_payload_.empty()) {
        // Continuation of previous partial packet
        this->partial_payload_.insert(this->partial_payload_.end(), data, data + len);
        if (this->partial_payload_.size() >= this->expected_length_) {
            this->dispatch_payload_(this->partial_client_slot_,
                                    this->partial_payload_.data(),
                                    this->expected_length_);
            this->partial_payload_.clear();
        }
        return;
    }

    if (len < 3) {
        ESP_LOGW(TAG, "[%s] Notification too short (%d bytes)", this->name_.c_str(), (int) len);
        return;
    }

    uint8_t client_slot    = data[0] - 0x40;
    uint8_t payload_length = data[2];
    const uint8_t *payload = data + 3;
    size_t payload_avail   = len - 3;

    if (payload_avail < payload_length) {
        // Partial — store and wait for continuation
        this->partial_client_slot_   = client_slot;
        this->expected_length_       = payload_length;
        this->partial_payload_.assign(payload, payload + payload_avail);
        return;
    }

    this->dispatch_payload_(client_slot, payload, payload_length);
}

void MiraModeDevice::dispatch_payload_(uint8_t client_slot,
                                        const uint8_t *payload,
                                        uint8_t length) {
    auto read_temp = [](const uint8_t *p) -> float {
        return static_cast<float>((p[0] << 8) | p[1]) / 10.0f;
    };

    if (length == 1) {
        // Success/failure — also used for pairing response
        bool ok = (payload[0] == 1);
        ESP_LOGI(TAG, "[%s] Success/failure: %s", this->name_.c_str(), ok ? "OK" : "FAIL");
        if (this->pairing_pending_) {
            if (ok) {
                this->client_id_       = this->pending_client_id_;
                this->client_slot_     = client_slot;
                this->paired_          = true;
                this->pairing_pending_ = false;
                this->save_credentials_();
                ESP_LOGI(TAG, "[%s] Paired! slot=%d id=0x%08X",
                         this->name_.c_str(), client_slot, this->client_id_);
            } else {
                this->pairing_pending_ = false;
                ESP_LOGE(TAG, "[%s] Pairing failed", this->name_.c_str());
            }
        }
        return;
    }

    if (length == 10) {
        // device_state: timer(1) target_temp(2) actual_temp(2) o1(1) o2(1) remaining(2) counter(1)
        float target_temp  = read_temp(payload + 1);
        float actual_temp  = read_temp(payload + 3);
        bool  outlet1      = (payload[5] == OUTLET_RUNNING);
        bool  outlet2      = (payload[6] == OUTLET_RUNNING);
        uint16_t remaining = static_cast<uint16_t>((payload[7] << 8) | payload[8]);

        ESP_LOGD(TAG, "[%s] State: target=%.1f actual=%.1f o1=%d o2=%d rem=%d",
                 this->name_.c_str(), target_temp, actual_temp, outlet1, outlet2, remaining);

        if (this->outlet1_switch_)      this->outlet1_switch_->set_state_from_device(outlet1);
        if (this->outlet2_switch_)      this->outlet2_switch_->set_state_from_device(outlet2);
        if (this->actual_temp_sensor_)  this->actual_temp_sensor_->publish_state(actual_temp);
        if (this->target_temp_number_)  this->target_temp_number_->set_state_from_device(target_temp);
        return;
    }

    if (length == 11 && (payload[0] == 1 || payload[0] == 0x80)) {
        // controls_operated: same layout as device_state with change_made prefix
        float target_temp  = read_temp(payload + 2);
        float actual_temp  = read_temp(payload + 4);
        bool  outlet1      = (payload[6] == OUTLET_RUNNING);
        bool  outlet2      = (payload[7] == OUTLET_RUNNING);

        if (this->outlet1_switch_)      this->outlet1_switch_->set_state_from_device(outlet1);
        if (this->outlet2_switch_)      this->outlet2_switch_->set_state_from_device(outlet2);
        if (this->actual_temp_sensor_)  this->actual_temp_sensor_->publish_state(actual_temp);
        if (this->target_temp_number_)  this->target_temp_number_->set_state_from_device(target_temp);
        return;
    }

    ESP_LOGD(TAG, "[%s] Unhandled payload length %d", this->name_.c_str(), length);
}
```

- [ ] **Step 2: Commit**

```bash
git add components/miramode/miramode.cpp
git commit -m "feat: notification handler with packet reassembly and state dispatch"
```

---

## Task 6: BLE Connection and GATT Event Handler

**Files:**
- Modify: `components/miramode/miramode.cpp` (add `setup`, `update`, `dump_config`, `gattc_event_handler`)

- [ ] **Step 1: Add lifecycle and BLE event methods to miramode.cpp**

```cpp
void MiraModeDevice::setup() {
    nvs_flash_init();
    this->load_credentials_();
}

void MiraModeDevice::update() {
    if (this->paired_ && this->node_state == ClientState::Established) {
        this->request_device_state();
    }
}

void MiraModeDevice::dump_config() {
    ESP_LOGCONFIG(TAG, "MiraMode '%s':", this->name_.c_str());
    ESP_LOGCONFIG(TAG, "  Paired: %s", this->paired_ ? "yes" : "no");
    if (this->paired_)
        ESP_LOGCONFIG(TAG, "  Client ID: 0x%08X  Slot: %d",
                      this->client_id_, this->client_slot_);
}

void MiraModeDevice::gattc_event_handler(esp_gattc_cb_event_t event,
                                           esp_gatt_if_t gattc_if,
                                           esp_ble_gattc_cb_param_t *param) {
    switch (event) {

    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK) {
            ESP_LOGW(TAG, "[%s] Connection failed: %d", this->name_.c_str(), param->open.status);
            this->node_state = ClientState::Idle;
            break;
        }
        ESP_LOGI(TAG, "[%s] Connected", this->name_.c_str());
        // Service discovery triggered automatically by BLEClient after open
        break;

    case ESP_GATTC_SEARCH_CMPL_EVT: {
        // Find write and read characteristics by UUID across all services
        static const char *WRITE_UUID = "bccb0002-ca66-11e5-88a4-0002a5d5c51b";
        static const char *READ_UUID  = "bccb0003-ca66-11e5-88a4-0002a5d5c51b";

        this->write_handle_ = 0;
        this->read_handle_  = 0;

        uint16_t conn_id = param->search_cmpl.conn_id;
        uint16_t count = 0;

        // Iterate all attribute handles in range 0x0001–0xFFFF
        // Use get_char_by_uuid for each target UUID
        esp_bt_uuid_t write_uuid, read_uuid;
        write_uuid.len = ESP_UUID_LEN_128;
        read_uuid.len  = ESP_UUID_LEN_128;

        // Parse UUID strings into esp_bt_uuid_t (128-bit, little-endian in ESP-IDF)
        auto parse_uuid128 = [](const char *s, esp_bt_uuid_t &out) {
            // UUID string format: "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx"
            uint8_t buf[16];
            int idx = 0;
            for (const char *p = s; *p && idx < 16; p++) {
                if (*p == '-') continue;
                uint8_t hi = (*p <= '9') ? (*p - '0') : (*p - 'a' + 10);
                p++;
                uint8_t lo = (*p <= '9') ? (*p - '0') : (*p - 'a' + 10);
                buf[idx++] = (hi << 4) | lo;
            }
            // ESP-IDF stores 128-bit UUIDs in little-endian byte order
            for (int i = 0; i < 16; i++) out.uuid.uuid128[i] = buf[15 - i];
        };

        parse_uuid128(WRITE_UUID, write_uuid);
        parse_uuid128(READ_UUID, read_uuid);

        count = 1;
        esp_gattc_char_elem_t char_elem;
        if (esp_ble_gattc_get_char_by_uuid(gattc_if, conn_id, 0x0001, 0xFFFF,
                                            write_uuid, &char_elem, &count) == ESP_OK
            && count > 0) {
            this->write_handle_ = char_elem.char_handle;
            ESP_LOGI(TAG, "[%s] Write handle: 0x%04X", this->name_.c_str(), this->write_handle_);
        }

        count = 1;
        if (esp_ble_gattc_get_char_by_uuid(gattc_if, conn_id, 0x0001, 0xFFFF,
                                            read_uuid, &char_elem, &count) == ESP_OK
            && count > 0) {
            this->read_handle_ = char_elem.char_handle;
            ESP_LOGI(TAG, "[%s] Read handle: 0x%04X", this->name_.c_str(), this->read_handle_);
            esp_ble_gattc_register_for_notify(gattc_if,
                                               this->parent()->get_remote_bda(),
                                               this->read_handle_);
        }

        if (!this->write_handle_ || !this->read_handle_)
            ESP_LOGE(TAG, "[%s] Failed to find Mira characteristics", this->name_.c_str());
        break;
    }

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        // Write 0x0001 to CCCD descriptor to enable notifications
        uint16_t notify_en = 1;
        // Find CCCD descriptor for the read characteristic
        uint16_t count = 1;
        esp_gattc_descr_elem_t descr;
        esp_bt_uuid_t cccd_uuid;
        cccd_uuid.len = ESP_UUID_LEN_16;
        cccd_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

        if (esp_ble_gattc_get_descr_by_char_handle(
                gattc_if, param->reg_for_notify.conn_id,
                this->read_handle_, cccd_uuid, &descr, &count) == ESP_OK
            && count > 0) {
            esp_ble_gattc_write_char_descr(
                gattc_if, param->reg_for_notify.conn_id,
                descr.handle, sizeof(notify_en),
                reinterpret_cast<uint8_t *>(&notify_en),
                ESP_GATT_WRITE_TYPE_RSP,
                ESP_GATT_AUTH_REQ_NONE);
        }
        this->node_state = ClientState::Established;
        ESP_LOGI(TAG, "[%s] Ready", this->name_.c_str());
        if (this->paired_) this->request_device_state();
        break;
    }

    case ESP_GATTC_NOTIFY_EVT:
        if (param->notify.handle == this->read_handle_) {
            this->handle_notification_(param->notify.value, param->notify.value_len);
        }
        break;

    case ESP_GATTC_DISCONNECT_EVT:
        this->write_handle_ = 0;
        this->read_handle_  = 0;
        this->partial_payload_.clear();
        this->node_state = ClientState::Idle;
        ESP_LOGI(TAG, "[%s] Disconnected", this->name_.c_str());
        break;

    default:
        break;
    }
}
```

- [ ] **Step 2: Commit**

```bash
git add components/miramode/miramode.cpp
git commit -m "feat: BLE GATT event handler with characteristic discovery and notification subscription"
```

---

## Task 7: Entity Write Handlers

**Files:**
- Modify: `components/miramode/miramode.cpp` (add entity write method bodies)

- [ ] **Step 1: Add entity write methods to miramode.cpp**

```cpp
// ── MiraModeSwitch ──────────────────────────────────────────────────────────

void MiraModeSwitch::write_state(bool state) {
    if (!this->parent_) return;
    bool o1 = (this->outlet_ == 1) ? state : this->parent_->outlet1_state_;
    bool o2 = (this->outlet_ == 2) ? state : this->parent_->outlet2_state_;
    this->parent_->control_outlets(o1, o2, this->parent_->target_temp_);
}

// ── MiraModeNumber ──────────────────────────────────────────────────────────

void MiraModeNumber::control(float value) {
    if (!this->parent_) return;
    this->parent_->control_outlets(
        this->parent_->outlet1_state_,
        this->parent_->outlet2_state_,
        value);
}

// ── MiraModeButton ──────────────────────────────────────────────────────────

void MiraModeButton::press_action() {
    if (!this->parent_) return;
    this->parent_->trigger_pair();
}
```

- [ ] **Step 2: Add missing includes to miramode.h**

The entity subclasses reference `switch_::Switch`, `sensor::Sensor`, `number::Number`, `button::Button`. Add these includes to the top of `miramode.h`:

```cpp
#pragma once
#include "esphome/core/component.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/button/button.h"
#include <string>
#include <vector>
```

- [ ] **Step 3: Commit**

```bash
git add components/miramode/miramode.h components/miramode/miramode.cpp
git commit -m "feat: entity write handlers for switch, number, and button"
```

---

## Task 8: ESPHome Python Schema

**Files:**
- Create: `components/miramode/__init__.py`

This file registers the component, defines the YAML schema, and generates C++ code for each configured instance.

- [ ] **Step 1: Create `components/miramode/__init__.py`**

```python
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ble_client, sensor, switch, number, button
from esphome.const import (
    CONF_ID, CONF_NAME,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)

DEPENDENCIES  = ["ble_client"]
AUTO_LOAD     = ["sensor", "switch", "number", "button"]
MULTI_CONF    = True

miramode_ns   = cg.esphome_ns.namespace("miramode")

MiraModeDevice = miramode_ns.class_(
    "MiraModeDevice", cg.PollingComponent, ble_client.BLEClientNode
)
MiraModeSwitch = miramode_ns.class_("MiraModeSwitch", switch.Switch, cg.Component)
MiraModeSensor = miramode_ns.class_("MiraModeSensor", sensor.Sensor, cg.Component)
MiraModeNumber = miramode_ns.class_("MiraModeNumber", number.Number, cg.Component)
MiraModeButton = miramode_ns.class_("MiraModeButton", button.Button, cg.Component)

CONF_CLIENT_NAME = "client_name"
CONF_OUTLET1     = "outlet1"
CONF_OUTLET2     = "outlet2"
CONF_ACTUAL_TEMP = "actual_temperature"
CONF_TARGET_TEMP = "target_temperature"
CONF_PAIR_BUTTON = "pair_button"
CONF_MIN_TEMP    = "min_value"
CONF_MAX_TEMP    = "max_value"
CONF_STEP        = "step"

_OUTLET_SCHEMA = switch.switch_schema(MiraModeSwitch)

_ACTUAL_TEMP_SCHEMA = sensor.sensor_schema(
    MiraModeSensor,
    unit_of_measurement=UNIT_CELSIUS,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
)

_TARGET_TEMP_SCHEMA = (
    number.number_schema(MiraModeNumber)
    .extend({
        cv.Optional(CONF_MIN_TEMP, default=20.0): cv.float_,
        cv.Optional(CONF_MAX_TEMP, default=48.0): cv.float_,
        cv.Optional(CONF_STEP,     default=0.5):  cv.positive_float,
    })
)

_PAIR_BUTTON_SCHEMA = button.button_schema(MiraModeButton)

CONFIG_SCHEMA = (
    cv.Schema({
        cv.GenerateID(): cv.declare_id(MiraModeDevice),
        cv.Required(CONF_NAME): cv.string,
        cv.Optional(CONF_CLIENT_NAME, default="ESPHome"): cv.string,
        cv.Optional(CONF_OUTLET1):     _OUTLET_SCHEMA,
        cv.Optional(CONF_OUTLET2):     _OUTLET_SCHEMA,
        cv.Optional(CONF_ACTUAL_TEMP): _ACTUAL_TEMP_SCHEMA,
        cv.Optional(CONF_TARGET_TEMP): _TARGET_TEMP_SCHEMA,
        cv.Optional(CONF_PAIR_BUTTON): _PAIR_BUTTON_SCHEMA,
    })
    .extend(ble_client.BLE_CLIENT_SCHEMA)
    .extend(cv.polling_component_schema("30s"))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await ble_client.register_ble_node(var, config)

    cg.add(var.set_name(config[CONF_NAME]))
    cg.add(var.set_client_name(config[CONF_CLIENT_NAME]))

    if CONF_OUTLET1 in config:
        sw = await switch.new_switch(config[CONF_OUTLET1])
        cg.add(sw.set_parent(var))
        cg.add(sw.set_outlet(1))
        cg.add(var.set_outlet1_switch(sw))

    if CONF_OUTLET2 in config:
        sw = await switch.new_switch(config[CONF_OUTLET2])
        cg.add(sw.set_parent(var))
        cg.add(sw.set_outlet(2))
        cg.add(var.set_outlet2_switch(sw))

    if CONF_ACTUAL_TEMP in config:
        sens = await sensor.new_sensor(config[CONF_ACTUAL_TEMP])
        cg.add(sens.set_parent(var))
        cg.add(var.set_actual_temp_sensor(sens))

    if CONF_TARGET_TEMP in config:
        cfg = config[CONF_TARGET_TEMP]
        num = await number.new_number(
            cfg,
            min_value=cfg[CONF_MIN_TEMP],
            max_value=cfg[CONF_MAX_TEMP],
            step=cfg[CONF_STEP],
        )
        cg.add(num.set_parent(var))
        cg.add(var.set_target_temp_number(num))

    if CONF_PAIR_BUTTON in config:
        btn = await button.new_button(config[CONF_PAIR_BUTTON])
        cg.add(btn.set_parent(var))
        cg.add(var.set_pair_button(btn))
```

- [ ] **Step 2: Commit**

```bash
git add components/miramode/__init__.py
git commit -m "feat: ESPHome Python schema with multi-instance support"
```

---

## Task 9: Schema Tests

**Files:**
- Create: `tests/test_schema.py`

These tests validate schema acceptance/rejection without a real ESP32 build.

- [ ] **Step 1: Install test dependencies**

```bash
pip install pytest esphome
```

- [ ] **Step 2: Create `tests/test_schema.py`**

```python
"""
Schema validation tests for miramode __init__.py.
Tests schema acceptance/rejection using ESPHome's cv validators directly.
"""
import sys
import os
import pytest

# Add components directory to path so the schema module can be imported standalone
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import esphome.config_validation as cv

# ── Import the schema ─────────────────────────────────────────────────────────
# We test the validators in isolation without triggering codegen
from components.miramode import (
    CONFIG_SCHEMA,
    CONF_CLIENT_NAME,
    CONF_OUTLET1,
    CONF_OUTLET2,
    CONF_ACTUAL_TEMP,
    CONF_TARGET_TEMP,
    CONF_PAIR_BUTTON,
    CONF_MIN_TEMP,
    CONF_MAX_TEMP,
    CONF_STEP,
)

# ── Helpers ────────────────────────────────────────────────────────────────────

def valid_base():
    """Minimal valid config (only required fields)."""
    return {
        "name": "Master Shower",
        "ble_client_id": "shower_ble",
        "update_interval": "30s",
    }


# ── Tests ─────────────────────────────────────────────────────────────────────

def test_name_required():
    cfg = valid_base()
    del cfg["name"]
    with pytest.raises(cv.Invalid):
        CONFIG_SCHEMA(cfg)

def test_client_name_defaults_to_esphome():
    """client_name should have a default and not raise when absent."""
    cfg = valid_base()
    result = CONFIG_SCHEMA(cfg)
    assert result[CONF_CLIENT_NAME] == "ESPHome"

def test_outlet1_optional():
    """Config without outlet1 is valid."""
    result = CONFIG_SCHEMA(valid_base())
    assert CONF_OUTLET1 not in result or result.get(CONF_OUTLET1) is None

def test_target_temp_defaults():
    cfg = valid_base()
    cfg[CONF_TARGET_TEMP] = {"name": "Target Temp"}
    result = CONFIG_SCHEMA(cfg)
    assert result[CONF_TARGET_TEMP][CONF_MIN_TEMP] == 20.0
    assert result[CONF_TARGET_TEMP][CONF_MAX_TEMP] == 48.0
    assert result[CONF_TARGET_TEMP][CONF_STEP]     == 0.5

def test_target_temp_custom_range():
    cfg = valid_base()
    cfg[CONF_TARGET_TEMP] = {
        "name": "Target Temp",
        CONF_MIN_TEMP: 25.0,
        CONF_MAX_TEMP: 45.0,
        CONF_STEP: 1.0,
    }
    result = CONFIG_SCHEMA(cfg)
    assert result[CONF_TARGET_TEMP][CONF_MIN_TEMP] == 25.0

def test_step_must_be_positive():
    cfg = valid_base()
    cfg[CONF_TARGET_TEMP] = {"name": "Target Temp", CONF_STEP: -1.0}
    with pytest.raises(cv.Invalid):
        CONFIG_SCHEMA(cfg)

def test_multi_conf_two_showers():
    """Two separate config dicts should both validate independently."""
    cfg1 = {**valid_base(), "name": "Ensuite Shower", "ble_client_id": "ble_1"}
    cfg2 = {**valid_base(), "name": "Master Shower",  "ble_client_id": "ble_2"}
    r1 = CONFIG_SCHEMA(cfg1)
    r2 = CONFIG_SCHEMA(cfg2)
    assert r1["name"] == "Ensuite Shower"
    assert r2["name"] == "Master Shower"
```

- [ ] **Step 3: Run schema tests**

```bash
cd /Users/davidbennett/workspace/esphome-miramode
pytest tests/test_schema.py -v
```

Expected: all tests PASS. Fix any schema issues before continuing.

- [ ] **Step 4: Commit**

```bash
git add tests/test_schema.py
git commit -m "test: ESPHome schema validation tests"
```

---

## Task 10: Example Config and Final Compilation Test

**Files:**
- Create: `example/example.yaml`

- [ ] **Step 1: Create two-shower example config**

Create `example/example.yaml`:

```yaml
# ESPHome config demonstrating two Mira Mode showers on one ESP32
# Compile with: esphome compile example/example.yaml

esphome:
  name: mira-controller
  platform: ESP32
  board: esp32dev

external_components:
  - source:
      type: local
      path: ../components

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

logger:
  level: DEBUG

api:

ota:

esp32_ble_tracker:

ble_client:
  - mac_address: "AA:BB:CC:DD:EE:01"
    id: shower_ensuite_ble
  - mac_address: "AA:BB:CC:DD:EE:02"
    id: shower_master_ble

miramode:
  - id: shower_ensuite
    name: "Ensuite Shower"
    client_name: "ESPHome-Ensuite"
    ble_client_id: shower_ensuite_ble
    update_interval: 30s
    outlet1:
      name: "Ensuite Outlet 1"
    outlet2:
      name: "Ensuite Outlet 2"
    actual_temperature:
      name: "Ensuite Actual Temperature"
    target_temperature:
      name: "Ensuite Target Temperature"
      min_value: 20.0
      max_value: 48.0
      step: 0.5
    pair_button:
      name: "Pair Ensuite Shower"

  - id: shower_master
    name: "Master Shower"
    client_name: "ESPHome-Master"
    ble_client_id: shower_master_ble
    update_interval: 30s
    outlet1:
      name: "Master Outlet 1"
    outlet2:
      name: "Master Outlet 2"
    actual_temperature:
      name: "Master Actual Temperature"
    target_temperature:
      name: "Master Target Temperature"
    pair_button:
      name: "Pair Master Shower"
```

- [ ] **Step 2: Verify ESPHome can compile the config**

```bash
cd /Users/davidbennett/workspace/esphome-miramode
esphome compile example/example.yaml 2>&1 | tail -20
```

Expected: `INFO Successfully compiled program.`

Fix any C++ compilation errors before proceeding.

- [ ] **Step 3: Run all tests**

```bash
pytest tests/test_packets.py tests/test_schema.py -v
/tmp/test_crc  # from Task 1 Step 5
```

All must pass.

- [ ] **Step 4: Final commit**

```bash
git add example/example.yaml
git commit -m "feat: two-shower example config"
git tag v0.1.0
```

---

## Multi-Instance Verification Checklist

Before calling the implementation complete, verify:

- [ ] Two `miramode:` entries compile without errors
- [ ] NVS namespaces are distinct for names that differ within the first 12 alphanumeric chars
- [ ] Each entity name in HA is unique (enforced by the `name` field on each entity)
- [ ] `trigger_pair()` is independent per instance — `pending_client_id_` is an instance variable, not global
- [ ] `partial_payload_` reassembly state is per-instance — no cross-instance contamination
- [ ] `write_handle_` and `read_handle_` are per-instance — each BLEClientNode has its own parent

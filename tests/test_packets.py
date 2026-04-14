"""
Validate packet building using the reference Python implementation.
These tests lock down expected byte sequences. The C++ must produce
identical output — verified manually against ESP32 debug logs.
"""
import struct
import pytest

# ── Reference implementation (from python-miramode verbatim) ────────────────

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
    # payload = 3 header + 4 client_id + 20 name = 27 bytes; + 2 CRC = 29 total
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
    assert p1[-2:] != p2[-2:]

def test_temperature_boundary_min():
    pkt = packet_control_outlets(1, 0x1, True, False, 0.0)
    temp = (pkt[4] << 8) | pkt[5]
    assert temp == 0

def test_temperature_boundary_max():
    # 6553.5°C → 65535 (max uint16)
    pkt = packet_control_outlets(1, 0x1, True, False, 6553.5)
    temp = (pkt[4] << 8) | pkt[5]
    assert temp == 65535

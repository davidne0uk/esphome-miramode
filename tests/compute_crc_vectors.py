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

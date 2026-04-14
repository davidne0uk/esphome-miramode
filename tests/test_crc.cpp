#include <cstdint>
#include <cstdio>
#include <cstring>

// Port of Python _crc() from python-miramode/miramode/__init__.py
// The Python algorithm uses an unbounded integer for i3 during bit processing,
// but masking i3 to 16 bits after each byte produces identical results, which
// allows a standard uint16_t implementation in C++.
static uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        uint16_t r = crc;
        for (int j = 0; j < 8; j++) {
            int msb = (r >> 15) & 1;
            int bit = (b >> (7 - j)) & 1;
            r = (uint16_t)(r << 1);
            if ((bit ^ msb) != 0)
                r ^= 0x1021;
        }
        crc = r;
    }
    return crc;
}

int main() {
    struct TestCase { const char *name; uint8_t data[16]; size_t len; uint16_t expected; };
    TestCase cases[] = {
        {"client_id_only", {0x12, 0x34, 0x56, 0x78}, 4, 0x30EC},
        {"request_state", {0x01, 0x07, 0x00, 0x12, 0x34, 0x56, 0x78}, 7, 0x35C2},
        {"zero_byte", {0x00}, 1, 0xE1F0},
        {"ff_byte", {0xFF}, 1, 0xFF00},
        {"magic_id_only", {0x54, 0xD2, 0xEE, 0x63}, 4, 0x5F86},
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
}

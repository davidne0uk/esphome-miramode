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
static const uint8_t  TIMER_STOPPED   = 0x00;
static const uint8_t  TIMER_RUNNING   = 0x01;
static const uint8_t  TIMER_PAUSED    = 0x03;

// Port of Python _crc() from python-miramode/miramode/__init__.py.
// The Python algorithm uses unbounded integers for intermediate values, but
// masking to 16 bits after each byte produces identical output, enabling a
// standard uint16_t implementation.
uint16_t MiraModeDevice::crc16_(const uint8_t *data, size_t len) {
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

}  // namespace miramode
}  // namespace esphome

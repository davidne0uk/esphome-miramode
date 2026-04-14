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

std::vector<uint8_t> MiraModeDevice::build_packet_(const std::vector<uint8_t> &payload,
                                                     uint32_t client_id) {
    // CRC is computed over payload || client_id (big-endian 4 bytes)
    std::vector<uint8_t> crc_input = payload;
    crc_input.push_back((client_id >> 24) & 0xFF);
    crc_input.push_back((client_id >> 16) & 0xFF);
    crc_input.push_back((client_id >>  8) & 0xFF);
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

}  // namespace miramode
}  // namespace esphome

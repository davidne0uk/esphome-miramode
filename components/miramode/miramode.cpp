#include "miramode.h"
#include "esphome/core/log.h"
#include <cctype>
#include <cmath>
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_random.h>

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
                                                     uint32_t client_id) const {
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
        static_cast<uint16_t>(len),
        const_cast<uint8_t *>(data),  // ESP-IDF API is non-const; does not mutate buffer
        ESP_GATT_WRITE_TYPE_NO_RSP,
        ESP_GATT_AUTH_REQ_NONE);
    if (status != ESP_OK)
        ESP_LOGW(TAG, "[%s] write_char failed: %d", this->name_.c_str(), status);
}

void MiraModeDevice::request_device_state() {
    std::vector<uint8_t> payload = {this->client_slot_, 0x07, 0x00};
    auto pkt = this->build_packet_(payload, this->client_id_);
    this->write_raw_(pkt.data(), pkt.size());
}

void MiraModeDevice::control_outlets(bool outlet1, bool outlet2, float temperature) {
    float clamped_temp = std::max(0.0f, std::min(6553.5f, temperature));
    this->outlet1_state_ = outlet1;
    this->outlet2_state_ = outlet2;
    this->target_temp_   = clamped_temp;

    uint16_t temp_val = static_cast<uint16_t>(std::round(clamped_temp * 10.0f));

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
    if (this->pairing_pending_) {
        ESP_LOGW(TAG, "[%s] Pairing already in progress, ignoring duplicate request",
                 this->name_.c_str());
        return;
    }
    this->pending_client_id_ = esp_random();
    this->pairing_pending_   = true;

    std::vector<uint8_t> name_bytes(this->client_name_.begin(),
                                     this->client_name_.end());
    if (name_bytes.size() > 20)
        name_bytes.resize(20);   // truncate long names
    name_bytes.resize(20, 0);    // zero-pad short names to exactly 20 bytes

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

std::string MiraModeDevice::nvs_namespace_() {
    std::string ns = "mm_";
    for (char c : this->name_) {
        if (std::isalnum(static_cast<unsigned char>(c)))
            ns += c;
        if (ns.size() == 15)
            break;
    }
    return ns;
}

void MiraModeDevice::load_credentials_() {
    nvs_handle_t handle;
    std::string ns = this->nvs_namespace_();
    esp_err_t err = nvs_open(ns.c_str(), NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "[%s] No NVS credentials (not yet paired)", this->name_.c_str());
        return;
    }
    uint32_t id   = 0;
    uint8_t  slot = 0;
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
        ESP_LOGE(TAG, "[%s] Failed to open NVS for writing: %d",
                 this->name_.c_str(), err);
        return;
    }
    nvs_set_u32(handle, "client_id",   this->client_id_);
    nvs_set_u8 (handle, "client_slot", this->client_slot_);
    nvs_commit(handle);
    nvs_close(handle);
    ESP_LOGI(TAG, "[%s] Saved credentials: id=0x%08X slot=%d",
             this->name_.c_str(), this->client_id_, this->client_slot_);
}

}  // namespace miramode
}  // namespace esphome

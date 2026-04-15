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
    // nvs_flash_init is idempotent for multiple instances; ESP-IDF handles it
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "[%s] nvs_flash_init failed: %d", this->name_.c_str(), ret);
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

std::string MiraModeDevice::nvs_namespace_() const {
    // nvs_key_ is set by Python codegen from the unique YAML component id,
    // so it is always distinct between instances. Fallback to name-derived
    // namespace only if not set (e.g. in tests).
    if (!this->nvs_key_.empty())
        return this->nvs_key_;
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
    err = nvs_set_u32(handle, "client_id", this->client_id_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[%s] nvs_set_u32 client_id failed: %d", this->name_.c_str(), err);
        nvs_close(handle);
        return;
    }
    err = nvs_set_u8(handle, "client_slot", this->client_slot_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[%s] nvs_set_u8 client_slot failed: %d", this->name_.c_str(), err);
        nvs_close(handle);
        return;
    }
    err = nvs_commit(handle);
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[%s] nvs_commit failed: %d", this->name_.c_str(), err);
        return;
    }
    ESP_LOGI(TAG, "[%s] Saved credentials: id=0x%08X slot=%d",
             this->name_.c_str(), this->client_id_, this->client_slot_);
}

void MiraModeDevice::handle_notification_(const uint8_t *data, size_t len) {
    // Handle fragmented packets (two-part notifications).
    // Use expected_length_ > 0 as sentinel — more robust than partial_payload_.empty()
    // which would fail for a zero-payload first fragment.
    if (this->expected_length_ > 0) {
        this->partial_payload_.insert(this->partial_payload_.end(), data, data + len);
        if (this->partial_payload_.size() >= this->expected_length_) {
            this->dispatch_payload_(this->partial_client_slot_,
                                    this->partial_payload_.data(),
                                    this->expected_length_);
            this->partial_payload_.clear();
            this->expected_length_ = 0;
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

        this->outlet1_state_ = outlet1;
        this->outlet2_state_ = outlet2;
        this->target_temp_   = target_temp;
        if (this->outlet1_switch_)     this->outlet1_switch_->set_state_from_device(outlet1);
        if (this->outlet2_switch_)     this->outlet2_switch_->set_state_from_device(outlet2);
        if (this->actual_temp_sensor_) this->actual_temp_sensor_->publish_state(actual_temp);
        if (this->target_temp_number_) this->target_temp_number_->set_state_from_device(target_temp);
        return;
    }

    if (length == 11 && (payload[0] == 1 || payload[0] == 0x80)) {
        // controls_operated: change_made(1) timer(1) target_temp(2) actual_temp(2) o1(1) o2(1) remaining(2) counter(1)
        float target_temp  = read_temp(payload + 2);
        float actual_temp  = read_temp(payload + 4);
        bool  outlet1      = (payload[6] == OUTLET_RUNNING);
        bool  outlet2      = (payload[7] == OUTLET_RUNNING);

        ESP_LOGD(TAG, "[%s] Controls operated: target=%.1f actual=%.1f o1=%d o2=%d",
                 this->name_.c_str(), target_temp, actual_temp, outlet1, outlet2);

        this->outlet1_state_ = outlet1;
        this->outlet2_state_ = outlet2;
        this->target_temp_   = target_temp;
        if (this->outlet1_switch_)     this->outlet1_switch_->set_state_from_device(outlet1);
        if (this->outlet2_switch_)     this->outlet2_switch_->set_state_from_device(outlet2);
        if (this->actual_temp_sensor_) this->actual_temp_sensor_->publish_state(actual_temp);
        if (this->target_temp_number_) this->target_temp_number_->set_state_from_device(target_temp);
        return;
    }

    ESP_LOGD(TAG, "[%s] Unhandled payload length %d", this->name_.c_str(), length);
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

        // Parse UUID strings into esp_bt_uuid_t (128-bit, little-endian in ESP-IDF)
        auto parse_uuid128 = [](const char *s, esp_bt_uuid_t &out) {
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

        esp_bt_uuid_t write_uuid, read_uuid;
        write_uuid.len = ESP_UUID_LEN_128;
        read_uuid.len  = ESP_UUID_LEN_128;
        parse_uuid128(WRITE_UUID, write_uuid);
        parse_uuid128(READ_UUID, read_uuid);

        uint16_t count = 1;
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
            esp_err_t notify_err = esp_ble_gattc_register_for_notify(
                gattc_if, this->parent()->get_remote_bda(), this->read_handle_);
            if (notify_err != ESP_OK)
                ESP_LOGW(TAG, "[%s] register_for_notify failed: %d",
                         this->name_.c_str(), notify_err);
        }

        if (!this->write_handle_ || !this->read_handle_)
            ESP_LOGE(TAG, "[%s] Failed to find Mira characteristics", this->name_.c_str());
        break;
    }

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        // Write 0x0001 to CCCD descriptor to enable notifications
        uint16_t notify_en = 1;
        uint16_t count = 1;
        esp_gattc_descr_elem_t descr;
        esp_bt_uuid_t cccd_uuid;
        cccd_uuid.len = ESP_UUID_LEN_16;
        cccd_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

        if (esp_ble_gattc_get_descr_by_char_handle(
                gattc_if, param->reg_for_notify.conn_id,
                this->read_handle_, cccd_uuid, &descr, &count) == ESP_OK
            && count > 0) {
            esp_err_t descr_err = esp_ble_gattc_write_char_descr(
                gattc_if, param->reg_for_notify.conn_id,
                descr.handle, sizeof(notify_en),
                reinterpret_cast<uint8_t *>(&notify_en),
                ESP_GATT_WRITE_TYPE_RSP,
                ESP_GATT_AUTH_REQ_NONE);
            if (descr_err != ESP_OK)
                ESP_LOGW(TAG, "[%s] write_char_descr (CCCD) failed: %d",
                         this->name_.c_str(), descr_err);
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
        this->write_handle_    = 0;
        this->read_handle_     = 0;
        this->partial_payload_.clear();
        this->expected_length_ = 0;
        this->pairing_pending_ = false;  // prevent trigger_pair() lockout on reconnect
        this->node_state = ClientState::Idle;
        ESP_LOGI(TAG, "[%s] Disconnected", this->name_.c_str());
        break;

    default:
        break;
    }
}

}  // namespace miramode
}  // namespace esphome

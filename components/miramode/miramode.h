#pragma once
#include "esphome/core/component.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/button/button.h"
#include <string>
#include <vector>

namespace esphome {
namespace miramode {

class MiraModeSwitch;
class MiraModeSensor;
class MiraModeNumber;
class MiraModeButton;
class MiraModeUnpairButton;

class MiraModeDevice : public PollingComponent,
                       public ble_client::BLEClientNode {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  // BLE components set up before Wi-Fi — NVS and BLE stack are independent of networking
  float get_setup_priority() const override { return setup_priority::BLUETOOTH; }

  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                            esp_ble_gattc_cb_param_t *param) override;

  void set_name(const std::string &name) { name_ = name; }
  void set_client_name(const std::string &n) { client_name_ = n; }
  // nvs_key is computed by Python codegen from the component's unique YAML id,
  // guaranteeing no namespace collisions between instances in the same config.
  void set_nvs_key(const std::string &k) { nvs_key_ = k; }

  void set_outlet1_switch(MiraModeSwitch *sw) { outlet1_switch_ = sw; }
  void set_outlet2_switch(MiraModeSwitch *sw) { outlet2_switch_ = sw; }
  void set_actual_temp_sensor(MiraModeSensor *s) { actual_temp_sensor_ = s; }
  void set_target_temp_number(MiraModeNumber *n) { target_temp_number_ = n; }
  void set_pair_button(MiraModeButton *b) { pair_button_ = b; }
  void set_unpair_button(MiraModeUnpairButton *b) { unpair_button_ = b; }

  void control_outlets(bool outlet1, bool outlet2, float temperature);
  void trigger_pair();
  void trigger_unpair();
  void request_device_state();

  // Accessible by entity write handlers
  bool  outlet1_state_{false};
  bool  outlet2_state_{false};
  float target_temp_{38.0f};

 protected:
  std::string name_;
  std::string client_name_{"ESPHome"};
  std::string nvs_key_;
  uint32_t client_id_{0};
  uint8_t  client_slot_{0};
  bool paired_{false};
  bool pairing_pending_{false};
  uint32_t pending_client_id_{0};

  uint16_t write_handle_{0};
  uint16_t read_handle_{0};

  MiraModeSwitch  *outlet1_switch_{nullptr};
  MiraModeSwitch  *outlet2_switch_{nullptr};
  MiraModeSensor  *actual_temp_sensor_{nullptr};
  MiraModeNumber  *target_temp_number_{nullptr};
  MiraModeButton       *pair_button_{nullptr};
  MiraModeUnpairButton *unpair_button_{nullptr};

  std::vector<uint8_t> partial_payload_;
  uint8_t partial_client_slot_{0};
  uint8_t expected_length_{0};

  static uint16_t crc16_(const uint8_t *data, size_t len);
  std::vector<uint8_t> build_packet_(const std::vector<uint8_t> &payload, uint32_t id) const;
  void write_chunks_(const std::vector<uint8_t> &data);
  void write_raw_(const uint8_t *data, size_t len);
  void handle_notification_(const uint8_t *data, size_t len);
  void dispatch_payload_(uint8_t client_slot, const uint8_t *payload, uint8_t length);
  void load_credentials_();
  void save_credentials_();  // called by gattc_event_handler after successful pair confirmation
  std::string nvs_namespace_() const;
};

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

class MiraModeUnpairButton : public button::Button, public Component {
 public:
  void set_parent(MiraModeDevice *p) { parent_ = p; }
 protected:
  void press_action() override;
  MiraModeDevice *parent_{nullptr};
};

}  // namespace miramode
}  // namespace esphome

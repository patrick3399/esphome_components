#pragma once

#include <array>
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/number/number.h"

namespace esphome {
namespace jiecang_desk_controller {

class JiecangDeskController : public Component, public uart::UARTDevice {
 public:
  float get_setup_priority() const override {
    return setup_priority::DATA;
  }

  void setup() override;
  void loop() override;
  void dump_config() override;

  // Sensor registration
  void set_height_sensor(sensor::Sensor *s) {
    this->height_sensor_ = s;
  }
  void set_height_min_sensor(sensor::Sensor *s) {
    this->height_min_sensor_ = s;
  }
  void set_height_max_sensor(sensor::Sensor *s) {
    this->height_max_sensor_ = s;
  }
  void set_height_pct_sensor(sensor::Sensor *s) {
    this->height_pct_sensor_ = s;
  }
  void set_m1_sensor(sensor::Sensor *s) {
    this->m1_sensor_ = s;
  }
  void set_m2_sensor(sensor::Sensor *s) {
    this->m2_sensor_ = s;
  }
  void set_m3_sensor(sensor::Sensor *s) {
    this->m3_sensor_ = s;
  }
  void set_m4_sensor(sensor::Sensor *s) {
    this->m4_sensor_ = s;
  }

  // Number registration (base-class pointers — only used to update traits and state)
  void set_height_number(number::Number *n) {
    this->height_number_ = n;
  }
  void set_height_pct_number(number::Number *n) {
    this->height_pct_number_ = n;
  }

  // Commands — called by button and number sub-entities via Parented<>
  void step_up();
  void step_down();
  void stop();
  void move_up();
  void move_down();
  void goto_preset(uint8_t preset);
  void save_preset(uint8_t preset);
  void goto_height(float height_cm);
  void goto_height_pct(float pct);

 protected:
  enum class ParseState : uint8_t { WAIT_FIRST_F2, WAIT_SECOND_F2, BUFFERING };

  ParseState parse_state_{ParseState::WAIT_FIRST_F2};
  std::array<uint8_t, 16> rx_buf_{};
  uint8_t rx_len_{0};

  // TX helpers
  void write_raw_(uint8_t cmd);
  void send_cmd_(uint8_t cmd);
  void send_goto_height_(float height_cm);

  // RX helpers
  void handle_byte_(uint8_t b);
  void handle_message_();
  static float decode_height_(uint8_t hi, uint8_t lo);

  // Desk state
  float current_height_{0.0f};
  float physical_min_{0.0f};
  float physical_max_{0.0f};
  float limit_min_{0.0f};
  float limit_max_{0.0f};
  bool limits_known_{false};
  uint32_t last_rx_ms_{0};
  uint32_t boot_ms_{0};
  bool initial_contact_{false};
  bool no_data_warned_{false};

  // Sensor sub-entities
  sensor::Sensor *height_sensor_{nullptr};
  sensor::Sensor *height_min_sensor_{nullptr};
  sensor::Sensor *height_max_sensor_{nullptr};
  sensor::Sensor *height_pct_sensor_{nullptr};
  sensor::Sensor *m1_sensor_{nullptr};
  sensor::Sensor *m2_sensor_{nullptr};
  sensor::Sensor *m3_sensor_{nullptr};
  sensor::Sensor *m4_sensor_{nullptr};

  // Number sub-entities
  number::Number *height_number_{nullptr};
  number::Number *height_pct_number_{nullptr};
};

}  // namespace jiecang_desk_controller
}  // namespace esphome

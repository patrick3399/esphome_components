#pragma once

#include "esphome/components/key_provider/key_provider.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include <array>
#include <cstdint>
#include <vector>

namespace esphome {
namespace hc138_keypad {

class HC138KeyTrigger : public Trigger<uint8_t> {};

class HC138Keypad : public key_provider::KeyProvider, public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_address_pins(std::vector<GPIOPin *> pins) { this->address_pins_ = std::move(pins); }
  void set_input_pins(std::vector<GPIOPin *> pins) { this->input_pins_ = std::move(pins); }
  void set_debounce_time(uint32_t debounce_time) { this->debounce_time_ = debounce_time; }
  void register_key_trigger(HC138KeyTrigger *trig) { this->key_triggers_.push_back(trig); }

 protected:
  uint64_t scan_keys_();
  void select_address_(uint8_t address);
  static uint8_t key_at_(uint8_t row, uint8_t col);
  void publish_changes_(uint64_t previous, uint64_t current);

  std::vector<GPIOPin *> address_pins_;
  std::vector<GPIOPin *> input_pins_;
  uint32_t debounce_time_{5};
  uint64_t stable_keys_{0};
  uint64_t candidate_keys_{0};
  uint32_t candidate_start_{0};
  std::vector<HC138KeyTrigger *> key_triggers_;
};

}  // namespace hc138_keypad
}  // namespace esphome

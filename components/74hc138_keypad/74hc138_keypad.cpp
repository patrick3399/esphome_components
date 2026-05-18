#include "74hc138_keypad.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <inttypes.h>

namespace esphome {
namespace hc138_keypad {

static const char *const TAG = "74hc138_keypad";

struct XMap {
  uint8_t x_1;
  uint8_t x_2;
};

static constexpr std::array<XMap, 7> X_MAP{{{0, 1}, {2, 3}, {4, 5}, {6, 7}, {8, 9}, {10, 11}, {12, 13}}};

static constexpr uint8_t KEY_MAP[4][14] = {
    {'`', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '-', '=', 0x08},
    {'\t', 'q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p', '[', ']', '\\'},
    {0, 0, 'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', ';', '\'', '\n'},
    {0, 0, 0, 'z', 'x', 'c', 'v', 'b', 'n', 'm', ',', '.', '/', ' '},
};

void HC138Keypad::setup() {
  for (auto *pin : this->address_pins_) {
    pin->setup();
    pin->pin_mode(gpio::FLAG_OUTPUT);
    pin->digital_write(false);
  }
  for (auto *pin : this->input_pins_) {
    pin->setup();
    pin->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);
  }
  this->select_address_(0);
}

void HC138Keypad::loop() {
  const uint32_t now = App.get_loop_component_start_time();
  const uint64_t scanned = this->scan_keys_();

  if (scanned != this->candidate_keys_) {
    this->candidate_keys_ = scanned;
    this->candidate_start_ = now;
    return;
  }

  if (scanned == this->stable_keys_ || now - this->candidate_start_ < this->debounce_time_)
    return;

  const uint64_t previous = this->stable_keys_;
  this->stable_keys_ = scanned;
  this->publish_changes_(previous, scanned);
}

uint64_t HC138Keypad::scan_keys_() {
  uint64_t keys = 0;

  for (uint8_t address = 0; address < 8; address++) {
    this->select_address_(address);
    delayMicroseconds(5);

    for (uint8_t input = 0; input < this->input_pins_.size(); input++) {
      if (!this->input_pins_[input]->digital_read())
        continue;

      const uint8_t col = address > 3 ? X_MAP[input].x_1 : X_MAP[input].x_2;
      const uint8_t raw_row = address > 3 ? address - 4 : address;
      const uint8_t row = 3 - raw_row;
      const uint8_t index = row * 14 + col;
      keys |= 1ULL << index;
    }
  }

  return keys;
}

void HC138Keypad::select_address_(uint8_t address) {
  address &= 0x07;
  for (uint8_t bit = 0; bit < this->address_pins_.size(); bit++)
    this->address_pins_[bit]->digital_write((address & (1 << bit)) != 0);
}

uint8_t HC138Keypad::key_at_(uint8_t row, uint8_t col) { return KEY_MAP[row][col]; }

void HC138Keypad::publish_changes_(uint64_t previous, uint64_t current) {
  const uint64_t pressed = current & ~previous;
  const uint64_t released = previous & ~current;

  for (uint8_t index = 0; index < 56; index++) {
    if ((pressed & (1ULL << index)) == 0)
      continue;

    const uint8_t row = index / 14;
    const uint8_t col = index % 14;
    const uint8_t key = key_at_(row, col);
    ESP_LOGD(TAG, "key @ row %u, col %u pressed%s%c", row, col, key == 0 ? "" : ": ", key == 0 ? ' ' : key);
    if (key == 0)
      continue;
    for (auto *trigger : this->key_triggers_)
      trigger->trigger(key);
    this->send_key_(key);
  }

  for (uint8_t index = 0; index < 56; index++) {
    if ((released & (1ULL << index)) == 0)
      continue;

    const uint8_t row = index / 14;
    const uint8_t col = index % 14;
    ESP_LOGD(TAG, "key @ row %u, col %u released", row, col);
  }
}

void HC138Keypad::dump_config() {
  ESP_LOGCONFIG(TAG, "74HC138 Keypad:");
  ESP_LOGCONFIG(TAG, "  Address pins:");
  for (auto *pin : this->address_pins_)
    LOG_PIN("    Pin: ", pin);
  ESP_LOGCONFIG(TAG, "  Input pins:");
  for (auto *pin : this->input_pins_)
    LOG_PIN("    Pin: ", pin);
  ESP_LOGCONFIG(TAG, "  Debounce: %" PRIu32 " ms", this->debounce_time_);
}

}  // namespace hc138_keypad
}  // namespace esphome

#include "aw9523.h"
#include "esphome/core/log.h"

namespace esphome {
namespace aw9523 {

static const char *const TAG = "aw9523";

// AW9523 register map
enum AW9523Register {
  REG_INPUT_P0 = 0x00,
  REG_INPUT_P1 = 0x01,
  REG_OUTPUT_P0 = 0x02,
  REG_OUTPUT_P1 = 0x03,
  REG_CONFIG_P0 = 0x04,
  REG_CONFIG_P1 = 0x05,
  REG_CONTROL = 0x11,
};

void AW9523::setup() {
  ESP_LOGCONFIG(TAG, "Setting up AW9523...");
  // Disable LED/PWM mode (set all 16 pins to GPIO mode)
  this->write_register(REG_CONTROL, 0x00);
  // Set all pins to input by default
  this->write_register(REG_CONFIG_P0, 0xFF);
  this->write_register(REG_CONFIG_P1, 0xFF);
}

void AW9523::pin_mode(uint8_t pin, gpio::Flags flags) {
  uint8_t reg = pin < 8 ? REG_CONFIG_P0 : REG_CONFIG_P1;
  uint8_t bit = pin % 8;
  bool is_input = flags == gpio::FLAG_INPUT;
  update_bit(reg, bit, is_input);
}

bool AW9523::digital_read(uint8_t pin) {
  uint8_t reg = pin < 8 ? REG_INPUT_P0 : REG_INPUT_P1;
  uint8_t value = 0;
  read_register(reg, &value);
  return (value >> (pin % 8)) & 0x01;
}

void AW9523::digital_write(uint8_t pin, bool value) {
  uint8_t reg = pin < 8 ? REG_OUTPUT_P0 : REG_OUTPUT_P1;
  update_bit(reg, pin % 8, value);
}

float AW9523::get_setup_priority() const {
  return setup_priority::HARDWARE;
}

bool AW9523::write_register(uint8_t reg, uint8_t value) {
  return this->write_byte(reg, value);
}

bool AW9523::read_register(uint8_t reg, uint8_t *value) {
  return this->read_byte(reg, value);
}

void AW9523::update_bit(uint8_t reg, uint8_t bit, bool value) {
  uint8_t current = 0;
  this->read_register(reg, &current);
  if (value)
    current |= (1 << bit);
  else
    current &= ~(1 << bit);
  this->write_register(reg, current);
}

// GPIOPin implementation
void AW9523GPIOPin::setup() {
  pin_mode(flags_);
}
void AW9523GPIOPin::pin_mode(gpio::Flags flags) {
  this->parent_->pin_mode(pin_, flags);
}
bool AW9523GPIOPin::digital_read() {
  return this->parent_->digital_read(pin_) != inverted_;
}
void AW9523GPIOPin::digital_write(bool value) {
  this->parent_->digital_write(pin_, value != inverted_);
}
std::string AW9523GPIOPin::dump_summary() const {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%u via AW9523", pin_);
  return buffer;
}

}  // namespace aw9523
}  // namespace esphome

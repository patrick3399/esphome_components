#include "pi4ioe5v6408.h"
#include "esphome/core/log.h"

namespace esphome {
namespace pi4ioe5v6408 {

static const char *const TAG = "pi4ioe5v6408";

static const uint8_t REG_INPUT = 0x00;
static const uint8_t REG_OUTPUT = 0x01;
static const uint8_t REG_CONFIG = 0x03;

void PI4IOE5V6408::setup() {
  ESP_LOGCONFIG(TAG, "Setting up PI4IOE5V6408...");
  uint8_t config;
  if (!read_config_(&config)) {
    this->mark_failed();
    return;
  }
  this->read_gpio_(&this->output_latch_);
}

float PI4IOE5V6408::get_setup_priority() const {
  return setup_priority::HARDWARE;
}

bool PI4IOE5V6408::read_gpio_(uint8_t *value) {
  return this->read_byte(REG_INPUT, value);
}

bool PI4IOE5V6408::write_output_(uint8_t value) {
  return this->write_byte(REG_OUTPUT, value);
}

bool PI4IOE5V6408::read_config_(uint8_t *value) {
  return this->read_byte(REG_CONFIG, value);
}

bool PI4IOE5V6408::write_config_(uint8_t value) {
  return this->write_byte(REG_CONFIG, value);
}

void PI4IOE5V6408::pin_mode(uint8_t pin, gpio::Flags flags) {
  uint8_t config;
  this->read_config_(&config);
  if (flags == gpio::FLAG_OUTPUT)
    config &= ~(1 << pin);  // output
  else
    config |= (1 << pin);   // input
  this->write_config_(config);
}

bool PI4IOE5V6408::digital_read(uint8_t pin) {
  uint8_t value = 0;
  this->read_gpio_(&value);
  return (value >> pin) & 0x01;
}

void PI4IOE5V6408::digital_write(uint8_t pin, bool value) {
  if (value)
    output_latch_ |= (1 << pin);
  else
    output_latch_ &= ~(1 << pin);
  this->write_output_(output_latch_);
}

void PI4IOE5V6408GPIOPin::setup() {
  pin_mode(flags_);
}

void PI4IOE5V6408GPIOPin::pin_mode(gpio::Flags flags) {
  this->parent_->pin_mode(pin_, flags);
}

bool PI4IOE5V6408GPIOPin::digital_read() {
  return this->parent_->digital_read(pin_) != this->inverted_;
}

void PI4IOE5V6408GPIOPin::digital_write(bool value) {
  this->parent_->digital_write(pin_, value != this->inverted_);
}

std::string PI4IOE5V6408GPIOPin::dump_summary() const {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%u via PI4IOE5V6408", pin_);
  return buffer;
}

gpio::Flags PI4IOE5V6408GPIOPin::get_flags() const {
  return this->flags_;
}

}  // namespace pi4ioe5v6408
}  // namespace esphome

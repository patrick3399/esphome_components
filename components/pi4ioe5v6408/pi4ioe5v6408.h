#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace pi4ioe5v6408 {

// Registers for PI4IOE5V6408 (confirm with datasheet if possible, these are common)
const uint8_t PI4IOE5V6408_INPUT_PORT_REG = 0x00;
const uint8_t PI4IOE5V6408_OUTPUT_PORT_REG = 0x01;
const uint8_t PI4IOE5V6408_POLARITY_INV_REG = 0x02; // We'll let ESPHome handle inversion
const uint8_t PI4IOE5V6408_CONFIG_REG = 0x03;       // 0 = Output, 1 = Input

class PI4IOE5V6408 : public PollingComponent, public i2c::I2CDevice {
 public:
  PI4IOE5V6408() = default;

  void setup() override;
  void dump_config() override;
  void update() override; // For PollingComponent to periodically read inputs

  // For GPIOPin to read cached input state
  bool digital_read_cached(uint8_t pin);
  // For GPIOPin to write output state
  void digital_write(uint8_t pin, bool value);
  // For GPIOPin to set pin mode
  void pin_mode(uint8_t pin, gpio::Flags flags);

  float get_setup_priority() const override { return setup_priority::IO; }

 protected:
  bool read_reg(uint8_t reg, uint8_t *value);
  bool write_reg(uint8_t reg, uint8_t value);

  uint8_t input_reg_cache_{0xFF};   // Cache for input port register
  uint8_t output_reg_cache_{0x00};  // Cache for output port register
  uint8_t config_reg_cache_{0xFF};  // Cache for configuration register (all pins input by default)
};

class PI4IOE5V6408GPIOPin : public GPIOPin {
 public:
  void setup() override;
  void pin_mode(gpio::Flags flags) override;
  bool digital_read() override;
  void digital_write(bool value) override;
  std::string dump_summary() const override;

  void set_parent(PI4IOE5V6408 *parent) { parent_ = parent; }
  void set_pin(uint8_t pin) { pin_ = pin; }
  void set_inverted(bool inverted) { inverted_ = inverted; }
  void set_flags(gpio::Flags flags) { flags_ = flags; }
  gpio::Flags get_flags() const override { return this->flags_; }


 protected:
  PI4IOE5V6408 *parent_;
  uint8_t pin_;
  bool inverted_;
  gpio::Flags flags_;
};

}  // namespace pi4ioe5v6408
}  // namespace esphome

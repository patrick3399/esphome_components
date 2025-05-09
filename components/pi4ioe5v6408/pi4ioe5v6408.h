#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace pi4ioe5v6408 {

class PI4IOE5V6408 : public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::IO; }

  void pin_mode(uint8_t pin, gpio::Flags flags);
  bool digital_read(uint8_t pin);
  void digital_write(uint8_t pin, bool value);

 protected:
  // MODIFIED: Added const char *cmd_name parameter to declarations
  bool read_byte_wrapper(uint8_t command_byte, uint8_t *value, const char *cmd_name);
  bool write_byte_wrapper(uint8_t command_byte, uint8_t value, const char *cmd_name);
  
  bool read_config_register(uint8_t *value);
  bool write_config_register(uint8_t value);
  bool read_input_register(uint8_t *value);
  bool write_output_register(uint8_t value);

  uint8_t output_latch_{0x00}; 
};

class PI4IOE5V6408GPIOPin : public GPIOPin {
 public:
  void setup() override;
  void pin_mode(gpio::Flags flags) override;
  bool digital_read() override;
  void digital_write(bool value) override;
  std::string dump_summary() const override;

  gpio::Flags get_flags() const override { return flags_; }

  void set_parent(PI4IOE5V6408 *parent) { parent_ = parent; }
  void set_pin(uint8_t pin) { pin_ = pin; }
  void set_inverted(bool inverted) { inverted_ = inverted; }
  void set_flags(gpio::Flags flags) { flags_ = flags; }

 protected:
  PI4IOE5V6408 *parent_;
  uint8_t pin_;
  bool inverted_;
  gpio::Flags flags_;
};

}  // namespace pi4ioe5v6408
}  // namespace esphome

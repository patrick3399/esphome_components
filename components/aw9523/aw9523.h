#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace aw9523 {

class AW9523 : public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::IO; }

  void pin_mode(uint8_t pin, gpio::Flags flags);
  bool digital_read(uint8_t pin);
  void digital_write(uint8_t pin, bool value);

 protected:
  bool read_byte_wrapper(uint8_t reg, uint8_t *value);
  bool write_byte_wrapper(uint8_t reg, uint8_t value);
  bool update_register_bit(uint8_t reg, uint8_t bit, bool bit_value);
};

class AW9523GPIOPin : public GPIOPin {
 public:
  void setup() override;
  void pin_mode(gpio::Flags flags) override;
  bool digital_read() override;
  void digital_write(bool value) override;
  std::string dump_summary() const override;

  // ESPHome >= 2023.2.0 requires get_flags
  gpio::Flags get_flags() const override { return flags_; }


  void set_parent(AW9523 *parent) { parent_ = parent; }
  void set_pin(uint8_t pin) { pin_ = pin; }
  void set_inverted(bool inverted) { inverted_ = inverted; }
  void set_flags(gpio::Flags flags) { flags_ = flags; }

 protected:
  AW9523 *parent_;
  uint8_t pin_;
  bool inverted_;
  gpio::Flags flags_;
};

}  // namespace aw9523
}  // namespace esphome

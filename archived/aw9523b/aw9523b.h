#pragma once

#include "esphome/components/gpio_expander/cached_gpio.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace aw9523b {

enum AW9523BP0DriveMode : uint8_t {
  OPEN_DRAIN = 0x00,
  PUSH_PULL = 0x01,
};

enum AW9523BLEDMaxCurrent : uint8_t {
  CURRENT_MAX = 0x00,
  CURRENT_3QUARTERS = 0x01,
  CURRENT_HALF = 0x02,
  CURRENT_1QUARTER = 0x03,
};

class AW9523BComponent : public Component,
                         public i2c::I2CDevice,
                         public gpio_expander::CachedGpioExpander<uint8_t, 16> {
 public:
  AW9523BComponent() = default;

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::IO; }

  void pin_mode(uint8_t pin, gpio::Flags flags);

  void set_reset(bool reset) { this->reset_ = reset; }
  void set_interrupt_pin(InternalGPIOPin *interrupt_pin) { this->interrupt_pin_ = interrupt_pin; }
  void set_p0_drive_mode(AW9523BP0DriveMode mode) { this->p0_drive_mode_ = mode; }
  void set_led_max_current(AW9523BLEDMaxCurrent current) { this->led_max_current_ = current; }

  void setup_gpio_interrupt(uint8_t pin, bool enable);
  bool setup_led_mode(uint8_t pin);
  void write_led_current(uint8_t pin, uint8_t current);

 protected:
  static void IRAM_ATTR gpio_intr(AW9523BComponent *arg);

  bool digital_read_hw(uint8_t pin) override;
  bool digital_read_cache(uint8_t pin) override;
  void digital_write_hw(uint8_t pin, bool value) override;

  bool read_gpio_modes_();
  bool write_gpio_modes_();
  bool read_gpio_outputs_();
  bool read_gpio_interrupts_();
  bool read_led_modes_();
  bool enable_gpio_interrupt_(uint8_t pin);
  bool disable_gpio_interrupt_(uint8_t pin);

  uint16_t mode_mask_{0xFFFF};
  uint16_t output_mask_{0xFFFF};  // matches hardware default after soft reset (OUTPUT_P0/P1 = 0xFF)
  uint16_t input_mask_{0};
  uint16_t interrupt_mask_{0xFFFF};
  uint16_t led_mode_mask_{0xFFFF};

  AW9523BP0DriveMode p0_drive_mode_{OPEN_DRAIN};
  AW9523BLEDMaxCurrent led_max_current_{CURRENT_MAX};
  bool reset_{true};

  InternalGPIOPin *interrupt_pin_{nullptr};
};

class AW9523BGPIOPin : public GPIOPin {
 public:
  void setup() override;
  void pin_mode(gpio::Flags flags) override;
  bool digital_read() override;
  void digital_write(bool value) override;
  size_t dump_summary(char *buffer, size_t len) const override;

  void set_parent(AW9523BComponent *parent) { this->parent_ = parent; }
  void set_pin(uint8_t pin) { this->pin_ = pin; }
  void set_inverted(bool inverted) { this->inverted_ = inverted; }
  void set_flags(gpio::Flags flags) { this->flags_ = flags; }
  void set_use_interrupt(bool use_interrupt) { this->use_interrupt_ = use_interrupt; }
  gpio::Flags get_flags() const override { return this->flags_; }

 protected:
  AW9523BComponent *parent_;
  uint8_t pin_;
  bool inverted_;
  gpio::Flags flags_;
  bool use_interrupt_{false};
};

}  // namespace aw9523b
}  // namespace esphome

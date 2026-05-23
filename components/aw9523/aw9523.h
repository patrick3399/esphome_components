#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace aw9523 {

// Register map (AW9523B datasheet V2.4)
static const uint8_t AW9523_REG_INPUT0 = 0x00;
static const uint8_t AW9523_REG_INPUT1 = 0x01;
static const uint8_t AW9523_REG_OUTPUT0 = 0x02;  // reset: 0xFF
static const uint8_t AW9523_REG_OUTPUT1 = 0x03;  // reset: 0xFF
static const uint8_t AW9523_REG_CONFIG0 = 0x04;  // reset: 0xFF (all input)
static const uint8_t AW9523_REG_CONFIG1 = 0x05;  // reset: 0xFF
static const uint8_t AW9523_REG_INTENABLE0 = 0x06;  // 0=enabled, 1=disabled; reset: 0x00 (all enabled)
static const uint8_t AW9523_REG_INTENABLE1 = 0x07;
static const uint8_t AW9523_REG_CHIPID = 0x10;  // fixed 0x23
static const uint8_t AW9523_REG_GCR = 0x11;  // bit4=GPOMD(P0 push-pull), bits[1:0]=ISEL
static const uint8_t AW9523_REG_LEDMODE0 = 0x12;  // 1=GPIO, 0=LED; reset: 0xFF
static const uint8_t AW9523_REG_LEDMODE1 = 0x13;
static const uint8_t AW9523_REG_SOFTRESET = 0x7F;

// DIM_REGS[pin] → I2C address of that pin's 256-step LED dimming register.
// Layout is non-sequential: P1_0..P1_3 at 0x20-0x23, then P0_0..P0_7 at 0x24-0x2B,
// then P1_4..P1_7 at 0x2C-0x2F.
static const uint8_t AW9523_DIM_REGS[16] = {
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B,  // P0_0..P0_7  (pins 0-7)
    0x20, 0x21, 0x22, 0x23, 0x2C, 0x2D, 0x2E, 0x2F,  // P1_0..P1_7  (pins 8-15)
};

class AW9523Component : public PollingComponent, public i2c::I2CDevice {
 public:
  AW9523Component() = default;

  void setup() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override {
    return setup_priority::IO;
  }

  // GPIO
  bool digital_read(uint8_t pin);
  void digital_write(uint8_t pin, bool value);
  void pin_mode(uint8_t pin, gpio::Flags flags);

  // LED constant-current driver
  void led_mode(uint8_t pin);
  void set_led_value(uint8_t pin, uint8_t value);

  // Config setters (called from generated code)
  void set_imax_divider(uint8_t divider) {
    this->imax_divider_ = divider & 0x03;
  }
  void set_latch_inputs(bool latch) {
    this->latch_inputs_ = latch;
  }

 protected:
  bool read_reg(uint8_t reg, uint8_t *value);
  bool write_reg(uint8_t reg, uint8_t value);

  // Input cache — refreshed in update(); only used when latch_inputs_=true
  uint8_t input_mask_0_{0x00};
  uint8_t input_mask_1_{0x00};

  // Output shadow — must match hardware OUTPUT registers (reset value 0xFF)
  uint8_t output_mask_0_{0xFF};
  uint8_t output_mask_1_{0xFF};

  // Direction shadow — 1=input, 0=output (reset value 0xFF = all input)
  uint8_t dir_mask_0_{0xFF};
  uint8_t dir_mask_1_{0xFF};

  // LEDMODE shadow — 1=GPIO, 0=LED (reset value 0xFF = all GPIO)
  uint8_t ledmode_mask_0_{0xFF};
  uint8_t ledmode_mask_1_{0xFF};

  uint8_t imax_divider_{0};  // GCR bits[1:0]: 0=37mA, 1=28mA, 2=19mA, 3=9mA
  bool latch_inputs_{true};  // true: digital_read() returns cached value; false: reads hardware
};

class AW9523GPIOPin : public GPIOPin {
 public:
  void setup() override;
  void pin_mode(gpio::Flags flags) override;
  bool digital_read() override;
  void digital_write(bool value) override;
  std::string dump_summary() const override;

  void set_parent(AW9523Component *parent) {
    this->parent_ = parent;
  }
  void set_pin(uint8_t pin) {
    this->pin_ = pin;
  }
  void set_inverted(bool inverted) {
    this->inverted_ = inverted;
  }
  void set_flags(gpio::Flags flags) {
    this->flags_ = flags;
  }
  gpio::Flags get_flags() const override {
    return this->flags_;
  }

 protected:
  AW9523Component *parent_;
  uint8_t pin_;
  bool inverted_;
  gpio::Flags flags_;
};

}  // namespace aw9523
}  // namespace esphome

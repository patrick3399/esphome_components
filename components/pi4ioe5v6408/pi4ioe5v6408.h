#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace pi4ioe5v6408 {

// 寄存器地址根據 PI4IOE5V6408 數據手冊修正
const uint8_t PI4IOE5V6408_DEVICE_ID_CONTROL_REG = 0x01;
const uint8_t PI4IOE5V6408_IO_DIRECTION_REG = 0x03;      // 原 CONFIG_REG，數據手冊: 0=input, 1=output
const uint8_t PI4IOE5V6408_OUTPUT_STATE_REG = 0x05;       // 原 OUTPUT_PORT_REG
const uint8_t PI4IOE5V6408_OUTPUT_HIGH_Z_REG = 0x07;    // 控制輸出是否為高阻態
const uint8_t PI4IOE5V6408_INPUT_DEFAULT_STATE_REG = 0x09;
const uint8_t PI4IOE5V6408_PULL_UP_DOWN_ENABLE_REG = 0x0B;
const uint8_t PI4IOE5V6408_PULL_UP_DOWN_SELECT_REG = 0x0D;
const uint8_t PI4IOE5V6408_INPUT_STATUS_REG = 0x0F;      // 原 INPUT_PORT_REG

class PI4IOE5V6408 : public PollingComponent, public i2c::I2CDevice {
 public:
  PI4IOE5V6408() = default;

  void setup() override;
  void dump_config() override;
  void update() override;

  bool digital_read_cached(uint8_t pin);
  void digital_write(uint8_t pin, bool value);
  void pin_mode(uint8_t pin, gpio::Flags flags);

  float get_setup_priority() const override { return setup_priority::IO; }
  uint8_t get_i2c_address() const { return this->address_; }

 protected:
  bool read_reg(uint8_t reg, uint8_t *value);
  bool write_reg(uint8_t reg, uint8_t value);

  uint8_t io_direction_cache_{0x00};
  uint8_t output_state_cache_{0x00};
  uint8_t output_high_z_cache_{0xFF};
  uint8_t input_status_cache_{0x00};
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
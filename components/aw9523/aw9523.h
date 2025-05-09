#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace aw9523 {

// AW9523 暫存器 (根據 datasheet)
const uint8_t AW9523_REG_INPUT0 = 0x00;      // 輸入 Port 0 狀態
const uint8_t AW9523_REG_INPUT1 = 0x01;      // 輸入 Port 1 狀態
const uint8_t AW9523_REG_OUTPUT0 = 0x02;     // 輸出 Port 0
const uint8_t AW9523_REG_OUTPUT1 = 0x03;     // 輸出 Port 1
const uint8_t AW9523_REG_CONFIG0 = 0x04;     // 方向 Port 0 (0=輸出, 1=輸入)
const uint8_t AW9523_REG_CONFIG1 = 0x05;     // 方向 Port 1 (0=輸出, 1=輸入)
// const uint8_t AW9523_REG_INTENABLE0 = 0x06; // 中斷致能 Port 0 (0=致能, 1=禁能) - 此處未完全實作
// const uint8_t AW9523_REG_INTENABLE1 = 0x07; // 中斷致能 Port 1 - 此處未完全實作
const uint8_t AW9523_REG_CHIPID = 0x10;      // 晶片 ID
const uint8_t AW9523_REG_CTL = 0x11;         // 全域控制暫存器 (P0 模式, LED Imax)
const uint8_t AW9523_REG_LEDMODE0 = 0x12;    // P0_7-P0_0 工作模式 (1=GPIO, 0=LED)
const uint8_t AW9523_REG_LEDMODE1 = 0x13;    // P1_7-P1_0 工作模式 (1=GPIO, 0=LED)
const uint8_t AW9523_REG_SOFTRESET = 0x7F;   // 軟體重置

class AW9523Component : public PollingComponent, public i2c::I2CDevice {
 public:
  AW9523Component() = default;

  void setup() override;
  void dump_config() override;
  void update() override; // PollingComponent 的方法
  float get_setup_priority() const override { return setup_priority::IO; }

  bool digital_read(uint8_t pin);
  void digital_write(uint8_t pin, bool value);
  void pin_mode(uint8_t pin, gpio::Flags flags);

 protected:
  bool read_reg(uint8_t reg, uint8_t *value);
  bool write_reg(uint8_t reg, uint8_t value);
  
  // 用於儲存輸入值的內部快取，在 update() 中更新
  uint8_t input_mask_0_{0x00};
  uint8_t input_mask_1_{0x00};
  // 用於儲存輸出值的內部快取
  uint8_t output_mask_0_{0x00};
  uint8_t output_mask_1_{0x00};
  // 用於儲存方向設定的內部快取
  uint8_t dir_mask_0_{0xFF}; // 初始設定後預設全為輸入
  uint8_t dir_mask_1_{0xFF}; // 初始設定後預設全為輸入
};

class AW9523GPIOPin : public GPIOPin {
 public:
  void setup() override;
  void pin_mode(gpio::Flags flags) override;
  bool digital_read() override;
  void digital_write(bool value) override;
  std::string dump_summary() const override;

  // Getters 和 Setters
  void set_parent(AW9523Component *parent) { this->parent_ = parent; }
  void set_pin(uint8_t pin) { this->pin_ = pin; }
  void set_inverted(bool inverted) { this->inverted_ = inverted; }
  void set_flags(gpio::Flags flags) { this->flags_ = flags; }
  gpio::Flags get_flags() const override { return this->flags_; }


 protected:
  AW9523Component *parent_;
  uint8_t pin_;
  bool inverted_;
  gpio::Flags flags_;
};

}  // namespace aw9523
}  // namespace esphome
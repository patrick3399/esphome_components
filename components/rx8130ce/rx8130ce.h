#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/time/real_time_clock.h"
#include "esphome/core/time.h"
#include "esphome/core/automation.h"

namespace esphome {
namespace rx8130ce {

// Register addresses (保持不變)
const uint8_t RX8130CE_REGISTER_SEC    = 0x10;
const uint8_t RX8130CE_REGISTER_MIN    = 0x11;
const uint8_t RX8130CE_REGISTER_HOUR   = 0x12;
const uint8_t RX8130CE_REGISTER_WEEK   = 0x13;
const uint8_t RX8130CE_REGISTER_DAY    = 0x14;
const uint8_t RX8130CE_REGISTER_MONTH  = 0x15;
const uint8_t RX8130CE_REGISTER_YEAR   = 0x16;
const uint8_t RX8130CE_REGISTER_EXT    = 0x1C;
const uint8_t RX8130CE_REGISTER_FLAG   = 0x1D;
const uint8_t RX8130CE_REGISTER_CTRL0  = 0x1E;
const uint8_t RX8130CE_REGISTER_CTRL1  = 0x1F;

const uint8_t RX8130CE_FLAG_VLF_BIT    = 1;
const uint8_t RX8130CE_CTRL0_TEST_BIT  = 7;
const uint8_t RX8130CE_CTRL0_STOP_BIT  = 6;
const uint8_t RX8130CE_CTRL1_INIEN_BIT = 4;


class RX8130CEComponent : public time::RealTimeClock, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return esphome::setup_priority::HARDWARE; }
  void update() override; // 這個函式將被 ReadTimeAction 呼叫

  void write_time_to_rtc(const ESPTime &new_time);
  void write_esphome_time_to_rtc();

 protected:
  bool read_time_from_rtc_();
  bool write_time_to_rtc_internal_(const ESPTime &new_time);

  uint8_t bcd_to_dec(uint8_t bcd);
  uint8_t dec_to_bcd(uint8_t dec);
  
  bool check_vlf_and_initialize_();
};

// WriteTimeAction 的 C++ 類別定義
template<typename... Ts>
class RX8130CEWriteTimeAction : public Action<Ts...>, public Parented<RX8130CEComponent> {
 public:
  void play(Ts... x) override {
    this->parent_->write_esphome_time_to_rtc();
  }
};

// ReadTimeAction 的 C++ 類別定義
template<typename... Ts>
class RX8130CEReadTimeAction : public Action<Ts...>, public Parented<RX8130CEComponent> {
 public:
  void play(Ts... x) override {
    // 呼叫 RX8130CEComponent 的 update() 方法來觸發從 RTC 讀取和同步
    this->parent_->update();
  }
};

}  // namespace rx8130ce
}  // namespace esphome
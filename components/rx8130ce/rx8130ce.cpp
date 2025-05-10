#include "rx8130ce.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace rx8130ce {

static const char *const TAG = "rx8130ce.time";

void RX8130CEComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up RX8130CE RTC...");
  if (this->is_failed()) { 
    ESP_LOGE(TAG, "Communication with RX8130CE failed early. Check I2C address and wiring.");
    return;
  }
  
  delay(40); 

  if (!check_vlf_and_initialize_()) {
      this->mark_failed();
      return;
  }

  ESP_LOGCONFIG(TAG, "RX8130CE setup complete.");
}

bool RX8130CEComponent::check_vlf_and_initialize_() {
  uint8_t flag_reg;
  if (!this->read_byte(RX8130CE_REGISTER_FLAG, &flag_reg)) {
    ESP_LOGE(TAG, "Failed to read Flag Register (0x%02X)", RX8130CE_REGISTER_FLAG);
    return false;
  }
  ESP_LOGD(TAG, "Flag Register (0x%02X) value: 0x%02X", RX8130CE_REGISTER_FLAG, flag_reg);

  bool vlf_set = (flag_reg & (1 << RX8130CE_FLAG_VLF_BIT));

  if (vlf_set) {
    ESP_LOGW(TAG, "VLF (Voltage Low Flag) is set. RTC data may be invalid. Initializing registers.");
    uint8_t new_flag_reg = flag_reg & ~(1 << RX8130CE_FLAG_VLF_BIT);
    if (!this->write_byte(RX8130CE_REGISTER_FLAG, new_flag_reg)) {
        ESP_LOGE(TAG, "Failed to clear VLF in Flag Register (0x%02X)", RX8130CE_REGISTER_FLAG);
    } else {
        ESP_LOGI(TAG, "VLF bit cleared in Flag Register (0x%02X). New value: 0x%02X", RX8130CE_REGISTER_FLAG, new_flag_reg);
    }
  }

  uint8_t ctrl0_val = 0x00; 
  if (!this->write_byte(RX8130CE_REGISTER_CTRL0, ctrl0_val)) {
    ESP_LOGE(TAG, "Failed to write Control Register 0 (0x%02X)", RX8130CE_REGISTER_CTRL0);
    return false;
  }
  ESP_LOGD(TAG, "Control Register 0 (0x%02X) set to 0x%02X", RX8130CE_REGISTER_CTRL0, ctrl0_val);

  uint8_t ctrl1_val;
  if(!this->read_byte(RX8130CE_REGISTER_CTRL1, &ctrl1_val)){
    ESP_LOGW(TAG, "Failed to read Control Register 1 (0x%02X). Using default for INIEN.", RX8130CE_REGISTER_CTRL1);
    ctrl1_val = 0x00; 
  }
  ctrl1_val |= (1 << RX8130CE_CTRL1_INIEN_BIT); 
  if (!this->write_byte(RX8130CE_REGISTER_CTRL1, ctrl1_val)) {
    ESP_LOGW(TAG, "Failed to write Control Register 1 (0x%02X) to set INIEN.", RX8130CE_REGISTER_CTRL1);
  } else {
    ESP_LOGD(TAG, "Control Register 1 (0x%02X) INIEN bit set. Value: 0x%02X", RX8130CE_REGISTER_CTRL1, ctrl1_val);
  }

  if (vlf_set) { 
    ESP_LOGI(TAG, "VLF was set, attempting to write a default time (2000-01-01 00:00:00) if current RTC time is invalid after initialization.");
    bool read_ok = this->read_time_from_rtc_(); // This attempts to read from RTC and call synchronize_epoch_
    
    // After attempting a read, check if the time exposed by RealTimeClock's public API is valid.
    if (!read_ok || !this->now().is_valid()) { 
        ESP_LOGW(TAG, "RTC time is invalid after VLF condition or initial read. Writing default time.");
        ESPTime default_time{};
        default_time.year = 2000;
        default_time.month = 1;
        default_time.day_of_month = 1;
        default_time.hour = 0;
        default_time.minute = 0;
        default_time.second = 0;
        default_time.day_of_week = 7; // Saturday for 2000-01-01
        default_time.recalc_timestamp_utc(false);
        write_time_to_rtc_internal_(default_time);
    }
  }
  return true;
}

void RX8130CEComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "RX8130CE RTC:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with RX8130CE has failed!");
  }
  ESP_LOGCONFIG(TAG, "  Timezone: '%s'", this->timezone_.c_str());
  uint8_t flag_reg;
  if (this->read_byte(RX8130CE_REGISTER_FLAG, &flag_reg)) {
    ESP_LOGD(TAG, "  Flag Register (0x1D): 0x%02X (VLF is %s)", flag_reg, (flag_reg & (1 << RX8130CE_FLAG_VLF_BIT)) ? "SET" : "CLEAR");
  }
  uint8_t ctrl0_reg;
  if (this->read_byte(RX8130CE_REGISTER_CTRL0, &ctrl0_reg)) {
    ESP_LOGD(TAG, "  Control0 Register (0x1E): 0x%02X (STOP is %s, TEST is %s)", ctrl0_reg, (ctrl0_reg & (1 << RX8130CE_CTRL0_STOP_BIT)) ? "SET" : "CLEAR", (ctrl0_reg & (1 << RX8130CE_CTRL0_TEST_BIT)) ? "SET" : "CLEAR" );
  }
}

void RX8130CEComponent::update() {
  if (this->is_failed()) return;
  this->read_time_from_rtc_();
}

uint8_t RX8130CEComponent::bcd_to_dec(uint8_t bcd) { return (bcd / 16 * 10) + (bcd % 16); }
uint8_t RX8130CEComponent::dec_to_bcd(uint8_t dec) { return (dec / 10 * 16) + (dec % 10); }

bool RX8130CEComponent::read_time_from_rtc_() {
  uint8_t buffer[7]; 

  uint8_t ctrl0;
  if(!this->read_byte(RX8130CE_REGISTER_CTRL0, &ctrl0)){
      ESP_LOGW(TAG, "Failed to read control register 0 before reading time.");
      return false;
  }
  if(ctrl0 & (1 << RX8130CE_CTRL0_STOP_BIT)){
      ESP_LOGW(TAG, "RTC is stopped (STOP bit is set). Time will not be updated from RTC.");
      return true; 
  }

  if (!this->read_bytes(RX8130CE_REGISTER_SEC, buffer, 7)) {
    ESP_LOGW(TAG, "Reading time registers from RX8130CE failed!");
    return false;
  }

  ESPTime rtc_time;
  rtc_time.second = bcd_to_dec(buffer[0] & 0x7F); 
  rtc_time.minute = bcd_to_dec(buffer[1] & 0x7F); 
  rtc_time.hour = bcd_to_dec(buffer[2] & 0x3F); 
  
  uint8_t week_val = buffer[3];
  if (week_val & 0x01) rtc_time.day_of_week = 1; 
  else if (week_val & 0x02) rtc_time.day_of_week = 2; 
  else if (week_val & 0x04) rtc_time.day_of_week = 3; 
  else if (week_val & 0x08) rtc_time.day_of_week = 4; 
  else if (week_val & 0x10) rtc_time.day_of_week = 5; 
  else if (week_val & 0x20) rtc_time.day_of_week = 6; 
  else if (week_val & 0x40) rtc_time.day_of_week = 7; 
  else rtc_time.day_of_week = 1; 

  rtc_time.day_of_month = bcd_to_dec(buffer[4] & 0x3F);
  rtc_time.month = bcd_to_dec(buffer[5] & 0x1F);
  rtc_time.year = bcd_to_dec(buffer[6]) + 2000;

  rtc_time.is_dst = false;
  rtc_time.recalc_timestamp_utc(this->timezone_.empty());

  if (!rtc_time.is_valid()) {
    ESP_LOGW(TAG, "Invalid time read from RX8130CE: %04d-%02d-%02d %02d:%02d:%02d",
             rtc_time.year, rtc_time.month, rtc_time.day_of_month,
             rtc_time.hour, rtc_time.minute, rtc_time.second);
    return false;
  }

  this->synchronize_epoch_(rtc_time.timestamp);
  ESP_LOGD(TAG, "Time successfully read from RX8130CE: %04d-%02d-%02d %02d:%02d:%02d",
           rtc_time.year, rtc_time.month, rtc_time.day_of_month,
           rtc_time.hour, rtc_time.minute, rtc_time.second);
  return true;
}

void RX8130CEComponent::write_esphome_time_to_rtc() {
    if (this->is_failed()) return;
    ESPTime current_esphome_time = this->now(); 
    if (!current_esphome_time.is_valid()) {
        ESP_LOGW(TAG, "ESPhome system time is not valid. Cannot write to RTC.");
        return;
    }
    this->write_time_to_rtc_internal_(current_esphome_time);
}

void RX8130CEComponent::write_time_to_rtc(const ESPTime &new_time) {
    if (this->is_failed()) return;
     if (!new_time.is_valid()) {
        ESP_LOGW(TAG, "Provided time for writing to RTC is not valid.");
        return;
    }
    this->write_time_to_rtc_internal_(new_time);
}

bool RX8130CEComponent::write_time_to_rtc_internal_(const ESPTime &new_time) {
  uint8_t buffer[7];

  uint8_t ctrl0_orig;
  bool read_ctrl0_ok = this->read_byte(RX8130CE_REGISTER_CTRL0, &ctrl0_orig);
  if (!read_ctrl0_ok) {
      ESP_LOGE(TAG, "Failed to read Control Register 0 before writing time. Aborting write.");
      return false;
  }

  uint8_t ctrl0_stop = (ctrl0_orig & ~(1 << RX8130CE_CTRL0_TEST_BIT)) | (1 << RX8130CE_CTRL0_STOP_BIT);
  if (!this->write_byte(RX8130CE_REGISTER_CTRL0, ctrl0_stop)) {
      ESP_LOGE(TAG, "Failed to set STOP bit in Control Register 0. Aborting write.");
      return false;
  }
  ESP_LOGD(TAG, "STOP bit set before writing time.");

  buffer[0] = dec_to_bcd(new_time.second);    
  buffer[1] = dec_to_bcd(new_time.minute);    
  buffer[2] = dec_to_bcd(new_time.hour);      
  
  if (new_time.day_of_week >= 1 && new_time.day_of_week <= 7) {
      buffer[3] = (1 << (new_time.day_of_week - 1));
  } else {
      buffer[3] = 0x01; 
  }

  buffer[4] = dec_to_bcd(new_time.day_of_month); 
  buffer[5] = dec_to_bcd(new_time.month);        
  buffer[6] = dec_to_bcd(new_time.year % 100);

  bool success = this->write_bytes(RX8130CE_REGISTER_SEC, buffer, 7);

  uint8_t ctrl0_run = (ctrl0_orig & ~((1 << RX8130CE_CTRL0_TEST_BIT) | (1 << RX8130CE_CTRL0_STOP_BIT))); 
  if (!this->write_byte(RX8130CE_REGISTER_CTRL0, ctrl0_run)) {
      ESP_LOGE(TAG, "Failed to clear STOP bit in Control Register 0 after writing time.");
  } else {
      ESP_LOGD(TAG, "STOP bit cleared after writing time. Ctrl0 set to: 0x%02X", ctrl0_run);
  }

  if (!success) {
    ESP_LOGE(TAG, "Writing time registers to RX8130CE failed!");
    return false;
  }

  ESP_LOGD(TAG, "Time written to RX8130CE: %04d-%02d-%02d %02d:%02d:%02d",
           new_time.year, new_time.month, new_time.day_of_month,
           new_time.hour, new_time.minute, new_time.second);
  this->synchronize_epoch_(new_time.timestamp); 
  return true;
}

}  // namespace rx8130ce
}  // namespace esphome
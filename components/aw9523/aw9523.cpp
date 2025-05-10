#include "aw9523.h"
#include "esphome/core/log.h"
#include <cstdio> // 用於 snprintf

namespace esphome {
namespace aw9523 {

static const char *const TAG = "aw9523";

void AW9523Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up AW9523...");
  uint8_t chip_id = 0;
  if (!this->read_reg(AW9523_REG_CHIPID, &chip_id)) {
    ESP_LOGE(TAG, "Chip ID read failed!");
    this->mark_failed();
    return;
  }
  if (chip_id != 0x23) { // AW9523B 預期的晶片 ID
    ESP_LOGE(TAG, "Invalid Chip ID: 0x%02X (expected 0x23)", chip_id);
    this->mark_failed();
    return;
  }
  ESP_LOGCONFIG(TAG, "Chip ID: 0x%02X", chip_id);

  // 軟體重置晶片
  if (!this->write_reg(AW9523_REG_SOFTRESET, 0x00)) {
    ESP_LOGW(TAG, "Soft reset failed");
    // 不要標記為失敗，嘗試繼續
  }
  // 重置後，暫存器恢復預設值。
  // Datasheet 指出 LEDMODEx 預設為 0xFF (GPIO 模式)
  // CONFIGx 預設為 0x00 (輸出模式)

  // 明確設定所有 pin 為 GPIO 模式 (雖然重置後預設應是如此)
  this->write_reg(AW9523_REG_LEDMODE0, 0xFF); // 所有 P0 pin 設為 GPIO 模式
  this->write_reg(AW9523_REG_LEDMODE1, 0xFF); // 所有 P1 pin 設為 GPIO 模式

  // 設定 P0 為 Push-Pull 模式 (P1 預設為 Push-Pull 且不可設定)
  // 讀取目前的 CTL 暫存器
  uint8_t ctl_reg_val = 0;
  this->read_reg(AW9523_REG_CTL, &ctl_reg_val);
  ctl_reg_val |= (1 << 4); // 設定 bit 4 使 P0 為 Push-Pull 模式
  this->write_reg(AW9523_REG_CTL, ctl_reg_val);

  // 透過讀取重置後的目前輸出狀態來初始化輸出遮罩
  // 預設輸出狀態取決於 AD0/AD1 pin
  // 為安全起見，我們將其初始化為 0，實際狀態將由 switch 實體設定。
  this->read_reg(AW9523_REG_OUTPUT0, &this->output_mask_0_);
  this->read_reg(AW9523_REG_OUTPUT1, &this->output_mask_1_);

  // 根據 ESPHome 中常見的擴展器設定，將方向遮罩初始化為全輸入
  // 雖然 datasheet 預設 CONFIG 為輸出，但用作感測器的 pin 需要是輸入。
  // 個別的 pin_mode 呼叫會根據需要進行設定。
  this->dir_mask_0_ = 0xFF; // 所有 P0 設為輸入
  this->dir_mask_1_ = 0xFF; // 所有 P1 設為輸入
  this->write_reg(AW9523_REG_CONFIG0, this->dir_mask_0_);
  this->write_reg(AW9523_REG_CONFIG1, this->dir_mask_1_);
  
  ESP_LOGCONFIG(TAG, "AW9523 setup complete.");
}

void AW9523Component::dump_config() {
  ESP_LOGCONFIG(TAG, "AW9523:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with AW9523 failed!");
  }
  ESP_LOGCONFIG(TAG, "  Update Interval: %.0f ms", this->get_update_interval());
  // 如果需要，可透過迭代已註冊的 pin 來記錄 pin 的設定
}

void AW9523Component::update() {
  // 此方法由 PollingComponent 框架呼叫
  // 讀取輸入 port 0
  if (!this->read_reg(AW9523_REG_INPUT0, &this->input_mask_0_)) {
    ESP_LOGW(TAG, "Failed to read input port 0");
    this->status_set_warning();
    return;
  }
  // 讀取輸入 port 1
  if (!this->read_reg(AW9523_REG_INPUT1, &this->input_mask_1_)) {
    ESP_LOGW(TAG, "Failed to read input port 1");
    this->status_set_warning();
    return;
  }
  this->status_clear_warning();
  // ESP_LOGV(TAG, "Inputs: P0=0x%02X, P1=0x%02X", this->input_mask_0_, this->input_mask_1_);
}

bool AW9523Component::digital_read(uint8_t pin) {
  uint8_t port = pin / 8; // 0 代表 P0 (pins 0-7), 1 代表 P1 (pins 8-15)
  uint8_t pin_of_port = pin % 8;

  if (port == 0) {
    return (this->input_mask_0_ >> pin_of_port) & 0x01;
  } else {
    return (this->input_mask_1_ >> pin_of_port) & 0x01;
  }
}

void AW9523Component::digital_write(uint8_t pin, bool value) {
  uint8_t port = pin / 8;
  uint8_t pin_of_port = pin % 8;
  uint8_t reg_addr;
  uint8_t *current_output_mask;

  if (port == 0) {
    reg_addr = AW9523_REG_OUTPUT0;
    current_output_mask = &this->output_mask_0_;
  } else {
    reg_addr = AW9523_REG_OUTPUT1;
    current_output_mask = &this->output_mask_1_;
  }

  if (value) {
    *current_output_mask |= (1 << pin_of_port);
  } else {
    *current_output_mask &= ~(1 << pin_of_port);
  }

  if (!this->write_reg(reg_addr, *current_output_mask)) {
      ESP_LOGW(TAG, "Failed to write to output register for pin %d", pin);
  }
}

void AW9523Component::pin_mode(uint8_t pin, gpio::Flags flags) {
  uint8_t port = pin / 8;
  uint8_t pin_of_port = pin % 8;
  uint8_t config_reg_addr;
  uint8_t ledmode_reg_addr;
  uint8_t *current_dir_mask;

  if (port == 0) {
    config_reg_addr = AW9523_REG_CONFIG0;
    ledmode_reg_addr = AW9523_REG_LEDMODE0;
    current_dir_mask = &this->dir_mask_0_;
  } else {
    config_reg_addr = AW9523_REG_CONFIG1;
    ledmode_reg_addr = AW9523_REG_LEDMODE1;
    current_dir_mask = &this->dir_mask_1_;
  }

  // 確保 pin 處於 GPIO 模式 (而非 LED 模式)
  uint8_t ledmode_val = 0;
  this->read_reg(ledmode_reg_addr, &ledmode_val);
  ledmode_val |= (1 << pin_of_port); // 設定 bit 為 1 代表 GPIO 模式
  this->write_reg(ledmode_reg_addr, ledmode_val);
  
  // 設定方向
  if (flags == gpio::FLAG_INPUT) {
    *current_dir_mask |= (1 << pin_of_port); // 設定 bit 為 1 代表輸入
  } else if (flags == gpio::FLAG_OUTPUT) {
    *current_dir_mask &= ~(1 << pin_of_port); // 設定 bit 為 0 代表輸出
  }
  this->write_reg(config_reg_addr, *current_dir_mask);
}

bool AW9523Component::read_reg(uint8_t reg, uint8_t *value) {
  if (this->is_failed())
    return false;
  return this->read_byte(reg, value);
}

bool AW9523Component::write_reg(uint8_t reg, uint8_t value) {
  if (this->is_failed())
    return false;
  return this->write_byte(reg, value);
}

// AW9523GPIOPin 實作
void AW9523GPIOPin::setup() {
   ESP_LOGV(TAG, "Setting up AW9523Pin %u", this->pin_);
   this->pin_mode(this->flags_); // 套用設定的模式
}

void AW9523GPIOPin::pin_mode(gpio::Flags flags) {
  this->parent_->pin_mode(this->pin_, flags);
}

bool AW9523GPIOPin::digital_read() {
  return this->parent_->digital_read(this->pin_) != this->inverted_;
}

void AW9523GPIOPin::digital_write(bool value) {
  this->parent_->digital_write(this->pin_, value != this->inverted_);
}

std::string AW9523GPIOPin::dump_summary() const {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "GPIO%u via AW9523", this->pin_);
  return buffer;
}

}  // namespace aw9523
}  // namespace esphome
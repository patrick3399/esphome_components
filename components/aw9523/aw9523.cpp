#include "aw9523.h"
#include "esphome/core/log.h"

namespace esphome {
namespace aw9523 {

static const char *const TAG = "aw9523";

// AW9523B Register Map (based on AW9523B Datasheet V1.1.1)
enum AW9523Register : uint8_t {
  REG_INPUT_PORT0 = 0x00,   // Input status of P0 (Read Only)
  REG_INPUT_PORT1 = 0x01,   // Input status of P1 (Read Only)
  REG_OUTPUT_PORT0 = 0x02,  // Output status of P0
  REG_OUTPUT_PORT1 = 0x03,  // Output status of P1
  REG_CONFIG_PORT0 = 0x04,  // Direction config for P0 (0=Output, 1=Input, Default: 00H)
  REG_CONFIG_PORT1 = 0x05,  // Direction config for P1 (0=Output, 1=Input, Default: 00H)
  REG_CHIP_ID = 0x10,       // Chip ID (Default: 23H, Read Only)
  REG_CTL = 0x11,           // Global Control Register (Default: 00H)
  REG_P0_MODE_SWITCH = 0x12, // P0_7~P0_0 Mode: 0=LED, 1=GPIO (Default: FFH)
  REG_P1_MODE_SWITCH = 0x13, // P1_7~P1_0 Mode: 0=LED, 1=GPIO (Default: FFH)
};

void AW9523::setup() {
  ESP_LOGCONFIG(TAG, "Setting up AW9523B (Address: 0x%02X)...", this->address_);

  uint8_t id = 0;
  if (!this->read_byte_wrapper(REG_CHIP_ID, &id, "REG_CHIP_ID")) { 
    ESP_LOGE(TAG, "Chip ID read failed.");
    this->mark_failed();
    return;
  }

  if (id != 0x23) {
    ESP_LOGE(TAG, "AW9523B not found or wrong ID (expected 0x23, got 0x%02X).", id);
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "AW9523B CHIP_ID: 0x%02X.", id);

  uint8_t ctl_val = 0x10; 
  ESP_LOGD(TAG, "Writing 0x%02X to CTL (0x11) for P0 Push-Pull.", ctl_val);
  if (!this->write_byte_wrapper(REG_CTL, ctl_val, "REG_CTL")) { 
    ESP_LOGE(TAG, "Failed to write CTL register.");
    this->mark_failed();
    return;
  }

  ESP_LOGD(TAG, "Writing 0xFF to P0_MODE_SWITCH (0x12) for GPIO mode.");
  if (!this->write_byte_wrapper(REG_P0_MODE_SWITCH, 0xFF, "REG_P0_MODE_SWITCH")) { 
    ESP_LOGE(TAG, "Failed to set Port 0 to GPIO mode.");
    this->mark_failed();
    return;
  }
  ESP_LOGD(TAG, "Writing 0xFF to P1_MODE_SWITCH (0x13) for GPIO mode.");
  if (!this->write_byte_wrapper(REG_P1_MODE_SWITCH, 0xFF, "REG_P1_MODE_SWITCH")) { 
    ESP_LOGE(TAG, "Failed to set Port 1 to GPIO mode.");
    this->mark_failed();
    return;
  }

  ESP_LOGD(TAG, "Writing 0xFF to CONFIG_PORT0 (0x04) for INPUT direction.");
  if (!this->write_byte_wrapper(REG_CONFIG_PORT0, 0xFF, "REG_CONFIG_PORT0")) { 
    ESP_LOGE(TAG, "Failed to set Port 0 direction to input.");
    this->mark_failed();
    return;
  }
  ESP_LOGD(TAG, "Writing 0xFF to CONFIG_PORT1 (0x05) for INPUT direction.");
  if (!this->write_byte_wrapper(REG_CONFIG_PORT1, 0xFF, "REG_CONFIG_PORT1")) { 
    ESP_LOGE(TAG, "Failed to set Port 1 direction to input.");
    this->mark_failed();
    return;
  }

  ESP_LOGCONFIG(TAG, "AW9523B setup successful.");
}

void AW9523::dump_config() {
  ESP_LOGCONFIG(TAG, "AW9523B:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with AW9523B failed!");
  }
}

void AW9523::pin_mode(uint8_t pin, gpio::Flags flags) {
  uint8_t reg = pin < 8 ? REG_CONFIG_PORT0 : REG_CONFIG_PORT1; 
  uint8_t bit_in_reg = pin % 8;
  bool is_input_mode = false;

  if ((flags & gpio::FLAG_INPUT) && !(flags & gpio::FLAG_OUTPUT)) {
    is_input_mode = true; 
  } else if ((flags & gpio::FLAG_OUTPUT) && !(flags & gpio::FLAG_INPUT)) {
    is_input_mode = false; 
  } else {
    ESP_LOGW(TAG, "Pin %u: Invalid mode flags 0x%X. Defaulting to input.", pin, static_cast<uint8_t>(flags));
    is_input_mode = true;
  }

  const char* reg_name = (reg == REG_CONFIG_PORT0) ? "REG_CONFIG_PORT0" : "REG_CONFIG_PORT1";
  ESP_LOGD(TAG, "Pin mode for P%d_%d (abs %u): %s. Target reg: %s (0x%02X), bit %d", 
           (pin < 8 ? 0 : 1), bit_in_reg, pin, is_input_mode ? "INPUT (1)" : "OUTPUT (0)", reg_name, reg, bit_in_reg);
  if (!this->update_register_bit(reg, bit_in_reg, is_input_mode, reg_name)) { 
      ESP_LOGE(TAG, "Failed to set pin mode for pin %u", pin);
  }
}

bool AW9523::digital_read(uint8_t pin) {
  uint8_t reg = pin < 8 ? REG_INPUT_PORT0 : REG_INPUT_PORT1;
  uint8_t bit_in_reg = pin % 8;
  uint8_t port_value = 0;
  const char* reg_name = (reg == REG_INPUT_PORT0) ? "REG_INPUT_PORT0" : "REG_INPUT_PORT1";

  ESP_LOGV(TAG, "Digital read for P%d_%d (abs %u). Target reg: %s (0x%02X)", 
           (pin < 8 ? 0 : 1), bit_in_reg, pin, reg_name, reg);

  if (!this->read_byte_wrapper(reg, &port_value, reg_name)) { 
    ESP_LOGW(TAG, "Failed to read %s for pin %u", reg_name, pin);
    return false; 
  }
  bool result = (port_value >> bit_in_reg) & 0x01;
  ESP_LOGV(TAG, "Read %s value: 0x%02X. Pin P%d_%d (abs %u) state: %s", 
           reg_name, port_value, (pin < 8 ? 0 : 1), bit_in_reg, pin, ONOFF(result));
  return result;
}

void AW9523::digital_write(uint8_t pin, bool value) {
  uint8_t reg = pin < 8 ? REG_OUTPUT_PORT0 : REG_OUTPUT_PORT1;
  uint8_t bit_in_reg = pin % 8;
  const char* reg_name = (reg == REG_OUTPUT_PORT0) ? "REG_OUTPUT_PORT0" : "REG_OUTPUT_PORT1";

  ESP_LOGD(TAG, "Digital write for P%d_%d (abs %u): %s. Target reg: %s (0x%02X), bit %d", 
           (pin < 8 ? 0 : 1), bit_in_reg, pin, ONOFF(value), reg_name, reg, bit_in_reg);
  if (!this->update_register_bit(reg, bit_in_reg, value, reg_name)) { 
      ESP_LOGE(TAG, "Failed to write %s for pin %u", reg_name, pin);
  }
}

bool AW9523::read_byte_wrapper(uint8_t reg, uint8_t *value, const char *reg_name) {
  ESP_LOGV(TAG, "I2C Read from %s (0x%02X)...", reg_name, reg);
  if (this->read_byte(reg, value) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "I2C Read from %s (0x%02X) FAILED!", reg_name, reg);
    return false;
  }
  ESP_LOGV(TAG, "I2C Read from %s (0x%02X) successful, value: 0x%02X", reg_name, reg, *value);
  return true;
}

bool AW9523::write_byte_wrapper(uint8_t reg, uint8_t value, const char *reg_name) {
  ESP_LOGV(TAG, "I2C Write to %s (0x%02X) with value 0x%02X...", reg_name, reg, value);
  if (this->write_byte(reg, value) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "I2C Write to %s (0x%02X) with value 0x%02X FAILED!", reg_name, reg, value);
    return false;
  }
  ESP_LOGV(TAG, "I2C Write to %s (0x%02X) with value 0x%02X successful.", reg_name, reg, value);
  return true;
}

bool AW9523::update_register_bit(uint8_t reg, uint8_t bit, bool bit_value, const char *reg_name) {
  uint8_t current_val;
  ESP_LOGV(TAG, "Updating bit %d in %s (0x%02X) to %s.", bit, reg_name, reg, ONOFF(bit_value));
  if (!this->read_byte_wrapper(reg, &current_val, reg_name)) { 
    ESP_LOGW(TAG, "Failed to read %s (0x%02X) for bit update.", reg_name, reg);
    return false;
  }
  ESP_LOGV(TAG, "Current value of %s (0x%02X): 0x%02X.", reg_name, reg, current_val);
  if (bit_value) {
    current_val |= (1 << bit);
  } else {
    current_val &= ~(1 << bit);
  }
  ESP_LOGV(TAG, "New value to write to %s (0x%02X): 0x%02X.", reg_name, reg, current_val);
  return this->write_byte_wrapper(reg, current_val, reg_name); 
}

// AW9523GPIOPin implementation
void AW9523GPIOPin::setup() {
  ESP_LOGV(TAG, "Setting up AW9523GPIOPin %u with flags 0x%X", pin_, static_cast<uint8_t>(flags_));
  this->pin_mode(flags_); 
}

void AW9523GPIOPin::pin_mode(gpio::Flags flags) {
  this->parent_->pin_mode(this->pin_, flags);
}

bool AW9523GPIOPin::digital_read() {
  bool parent_read = this->parent_->digital_read(this->pin_);
  bool final_read = parent_read != this->inverted_;
  ESP_LOGV(TAG, "AW9523GPIOPin %u digital_read: parent_val=%s, inverted=%s, final_val=%s", 
           pin_, ONOFF(parent_read), ONOFF(this->inverted_), ONOFF(final_read));
  return final_read;
}

void AW9523GPIOPin::digital_write(bool value) {
  bool value_to_write = value != this->inverted_;
  ESP_LOGV(TAG, "AW9523GPIOPin %u digital_write: requested_val=%s, inverted=%s, final_val_to_chip=%s",
           pin_, ONOFF(value), ONOFF(this->inverted_), ONOFF(value_to_write));
  this->parent_->digital_write(this->pin_, value_to_write);
}

std::string AW9523GPIOPin::dump_summary() const {
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "Pin %u (AW9523) Mode: %s Inverted: %s", 
           pin_, (flags_ & gpio::FLAG_INPUT) ? "Input" : "Output", ONOFF(inverted_));
  return buffer;
}

}  // namespace aw9523
}  // namespace esphome

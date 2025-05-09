#include "pi4ioe5v6408.h"
#include "esphome/core/log.h"

namespace esphome {
namespace pi4ioe5v6408 {

static const char *const TAG = "pi4ioe5v6408";

void PI4IOE5V6408::setup() {
  ESP_LOGCONFIG(TAG, "Setting up PI4IOE5V6408...");
  // Try to read a register to check if device is present
  uint8_t temp_config;
  if (!this->read_reg(PI4IOE5V6408_CONFIG_REG, &temp_config)) {
    ESP_LOGE(TAG, "PI4IOE5V6408 not available under I2C address 0x%02X", this->address_);
    this->mark_failed();
    return;
  }
  this->config_reg_cache_ = temp_config; // Store the currently read config

  // Initialize config register: set all to input by default (0xFF)
  // This is a safe default. Pin setup will configure them as needed.
  // It's also good practice to read current output state if chip retains it
  if (!this->read_reg(PI4IOE5V6408_OUTPUT_PORT_REG, &this->output_reg_cache_)) {
     ESP_LOGW(TAG, "Failed to read initial output register state.");
     // Keep output_reg_cache_ as 0x00 or some default if read fails
  }
  
  // Initial read of inputs
  if (!this->read_reg(PI4IOE5V6408_INPUT_PORT_REG, &this->input_reg_cache_)) {
    ESP_LOGW(TAG, "Failed to read initial input register state.");
    // Keep input_reg_cache_ as 0xFF or some default
  }

  ESP_LOGCONFIG(TAG, "PI4IOE5V6408 setup complete.");
}

void PI4IOE5V6408::dump_config() {
  ESP_LOGCONFIG(TAG, "PI4IOE5V6408:");
  ESP_LOGCONFIG(TAG, "  I2C Address: 0x%02X", this->address_);
  ESP_LOGCONFIG(TAG, "  Update Interval: %.2f ms", this->get_update_interval() / 1e3);
}

void PI4IOE5V6408::update() {
  // This is called by PollingComponent based on update_interval
  // Read the input port register
  if (this->is_failed())
    return;

  uint8_t new_input_val;
  if (!this->read_reg(PI4IOE5V6408_INPUT_PORT_REG, &new_input_val)) {
    ESP_LOGW(TAG, "Failed to read input port register during update");
    // Consider how to handle read failure, maybe keep old cache or mark component as failed temporarily
    return;
  }
  if (new_input_val != this->input_reg_cache_) {
      // ESP_LOGD(TAG, "Input port changed from 0x%02X to 0x%02X", this->input_reg_cache_, new_input_val);
      this->input_reg_cache_ = new_input_val;
      // BinarySensor/Switch components will read this cache when they poll
  }
}

bool PI4IOE5V6408::digital_read_cached(uint8_t pin) {
  if (this->is_failed() || pin >= 8)
    return false; // Or handle error appropriately
  return (this->input_reg_cache_ >> pin) & 0x01;
}

void PI4IOE5V6408::digital_write(uint8_t pin, bool value) {
  if (this->is_failed() || pin >= 8)
    return;

  if (value) {
    this->output_reg_cache_ |= (1 << pin);
  } else {
    this->output_reg_cache_ &= ~(1 << pin);
  }

  if (!this->write_reg(PI4IOE5V6408_OUTPUT_PORT_REG, this->output_reg_cache_)) {
    ESP_LOGW(TAG, "Failed to write to output port register");
    // Optionally mark_failed or revert cache
  }
}

void PI4IOE5V6408::pin_mode(uint8_t pin, gpio::Flags flags) {
  if (this->is_failed() || pin >= 8)
    return;

  uint8_t old_config_reg_cache = this->config_reg_cache_;

  if (flags == gpio::FLAG_INPUT) {
    this->config_reg_cache_ |= (1 << pin); // Set bit to 1 for input
  } else if (flags == gpio::FLAG_OUTPUT) {
    this->config_reg_cache_ &= ~(1 << pin); // Clear bit to 0 for output
  } else {
      // Other flags like PULLUP/PULLDOWN are not directly supported by basic PI4IOE5V6408 config
      // but this expander might have separate registers for pullups/pulldowns.
      // For now, we only handle INPUT/OUTPUT direction.
      // If it was an input, it remains an input. If output, remains output.
      // This part depends on whether PI4IOE5V6408 supports internal pullups/pulldowns via registers
      // and if you want to implement that. The PI4IOE5V6408 datasheet is key here.
      // Assuming it doesn't have easily configurable pullups/downs via the main config reg:
      if ((flags & gpio::FLAG_INPUT) && (flags & gpio::FLAG_PULLUP)) {
          this->config_reg_cache_ |= (1 << pin);
          ESP_LOGW(TAG, "PI4IOE5V6408 Pin %u: Pullup request ignored, chip might not support or not implemented.", pin);
      } else if ((flags & gpio::FLAG_INPUT) && (flags & gpio::FLAG_PULLDOWN)) {
          this->config_reg_cache_ |= (1 << pin);
          ESP_LOGW(TAG, "PI4IOE5V6408 Pin %u: Pulldown request ignored, chip might not support or not implemented.", pin);
      } else if (flags & gpio::FLAG_INPUT) {
          this->config_reg_cache_ |= (1 << pin);
      } else if (flags & gpio::FLAG_OUTPUT) {
          this->config_reg_cache_ &= ~(1 << pin);
      }
  }


  if (this->config_reg_cache_ != old_config_reg_cache) {
    if (!this->write_reg(PI4IOE5V6408_CONFIG_REG, this->config_reg_cache_)) {
      ESP_LOGW(TAG, "Failed to write to config register");
      this->config_reg_cache_ = old_config_reg_cache; // Revert cache on failure
      // Optionally mark_failed
    }
  }
}

bool PI4IOE5V6408::read_reg(uint8_t reg, uint8_t *value) {
  if (this->is_failed()) {
    return false;
  }
  // PI4IOE5V6408 typically doesn't require writing register address first for read if it's a plain expander
  // but some I2C devices do. The I2CDevice::read_byte_16bit etc. handle this.
  // For simple 8-bit register access:
  return this->read_byte(reg, value);
}

bool PI4IOE5V6408::write_reg(uint8_t reg, uint8_t value) {
  if (this->is_failed()) {
    return false;
  }
  return this->write_byte(reg, value);
}


// --- PI4IOE5V6408GPIOPin Methods ---
void PI4IOE5V6408GPIOPin::setup() {
  ESP_LOGV(TAG, "Setting up GPIO pin %u", pin_);
  this->pin_mode(this->flags_); // Apply initial mode and flags
}

void PI4IOE5V6408GPIOPin::pin_mode(gpio::Flags flags) {
  this->flags_ = flags; // Store the flags
  this->parent_->pin_mode(this->pin_, flags);
}

bool PI4IOE5V6408GPIOPin::digital_read() {
  // Read from parent's cache, XOR with inverted status
  return this->parent_->digital_read_cached(this->pin_) != this->inverted_;
}

void PI4IOE5V6408GPIOPin::digital_write(bool value) {
  // XOR with inverted status before writing to parent
  this->parent_->digital_write(this->pin_, value != this->inverted_);
}

std::string PI4IOE5V6408GPIOPin::dump_summary() const {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%u via PI4IOE5V6408", pin_);
  return buffer;
}

}  // namespace pi4ioe5v6408
}  // namespace esphome

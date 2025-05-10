#include "pi4ioe5v6408.h"
#include "esphome/core/log.h"
#include <cstdio>   // For snprintf
#include <cstdint>  // For UINT32_MAX

namespace esphome {
namespace pi4ioe5v6408 {

static const char *const TAG = "pi4ioe5v6408";

void PI4IOE5V6408::setup() {
  ESP_LOGCONFIG(TAG, "Setting up PI4IOE5V6408 at I2C address 0x%02X...", this->address_);
  uint8_t temp_val;
  bool success = true;

  if (!this->read_reg(PI4IOE5V6408_IO_DIRECTION_REG, &temp_val)) {
    ESP_LOGE(TAG, "Failed to read IO_DIRECTION_REG (0x%02X). Marking as failed.", PI4IOE5V6408_IO_DIRECTION_REG);
    success = false;
  } else {
    this->io_direction_cache_ = temp_val;
    ESP_LOGD(TAG, "Initial IO_DIRECTION_REG (0x%02X) is 0x%02X.", PI4IOE5V6408_IO_DIRECTION_REG, this->io_direction_cache_);
  }

  if (success && !this->read_reg(PI4IOE5V6408_OUTPUT_STATE_REG, &temp_val)) {
     ESP_LOGW(TAG, "Failed to read OUTPUT_STATE_REG (0x%02X). Assuming 0x00.", PI4IOE5V6408_OUTPUT_STATE_REG);
     this->output_state_cache_ = 0x00;
  } else if (success) {
     this->output_state_cache_ = temp_val;
     ESP_LOGD(TAG, "Initial OUTPUT_STATE_REG (0x%02X) is 0x%02X.", PI4IOE5V6408_OUTPUT_STATE_REG, this->output_state_cache_);
  }

  if (success && !this->read_reg(PI4IOE5V6408_OUTPUT_HIGH_Z_REG, &temp_val)) {
     ESP_LOGW(TAG, "Failed to read OUTPUT_HIGH_Z_REG (0x07). Assuming 0xFF.", PI4IOE5V6408_OUTPUT_HIGH_Z_REG);
     this->output_high_z_cache_ = 0xFF;
  } else if (success) {
     this->output_high_z_cache_ = temp_val;
     ESP_LOGD(TAG, "Initial OUTPUT_HIGH_Z_REG (0x%02X) is 0x%02X.", PI4IOE5V6408_OUTPUT_HIGH_Z_REG, this->output_high_z_cache_);
  }
  
  if (success && !this->read_reg(PI4IOE5V6408_INPUT_STATUS_REG, &this->input_status_cache_)) {
    ESP_LOGW(TAG, "Failed to read INPUT_STATUS_REG (0x%02X). Assuming 0x00.", PI4IOE5V6408_INPUT_STATUS_REG);
    this->input_status_cache_ = 0x00; 
  } else if (success) {
    ESP_LOGD(TAG, "Initial INPUT_STATUS_REG (0x%02X) is 0x%02X.", PI4IOE5V6408_INPUT_STATUS_REG, this->input_status_cache_);
  }

  uint8_t initial_pull_enable, initial_pull_select;
  if (success && this->read_reg(PI4IOE5V6408_PULL_UP_DOWN_ENABLE_REG, &initial_pull_enable)) {
      ESP_LOGD(TAG, "Initial PULL_UP_DOWN_ENABLE_REG (0x0B) is 0x%02X.", initial_pull_enable);
  } else if (success) {
      ESP_LOGW(TAG, "Failed to read PULL_UP_DOWN_ENABLE_REG (0x0B). It defaults to 0xFF (all enabled).");
  }
  if (success && this->read_reg(PI4IOE5V6408_PULL_UP_DOWN_SELECT_REG, &initial_pull_select)) {
      ESP_LOGD(TAG, "Initial PULL_UP_DOWN_SELECT_REG (0x0D) is 0x%02X.", initial_pull_select);
  } else if (success) {
      ESP_LOGW(TAG, "Failed to read PULL_UP_DOWN_SELECT_REG (0x0D). It defaults to 0x00 (all pull-down).");
  }

  if (!success) {
      this->mark_failed();
      return;
  }
  
  ESP_LOGCONFIG(TAG, "PI4IOE5V6408 setup complete.");
}

void PI4IOE5V6408::dump_config() {
  ESP_LOGCONFIG(TAG, "PI4IOE5V6408:");
  ESP_LOGCONFIG(TAG, "  I2C Address: 0x%02X", this->address_);
  if (this->is_failed()) {
    ESP_LOGCONFIG(TAG, "  Communication failed!");
  }
  ESP_LOGCONFIG(TAG, "  IO Direction Cache (Reg 0x03 final): 0x%02X", this->io_direction_cache_);
  ESP_LOGCONFIG(TAG, "  Output State Cache (Reg 0x05 final): 0x%02X", this->output_state_cache_);
  ESP_LOGCONFIG(TAG, "  Output High-Z Cache (Reg 0x07 final): 0x%02X", this->output_high_z_cache_);
  
  uint8_t pull_enable, pull_select;
  if (!this->is_failed()) {
    if (this->read_reg(PI4IOE5V6408_PULL_UP_DOWN_ENABLE_REG, &pull_enable)) {
        ESP_LOGCONFIG(TAG, "  PUD Enable Reg (0x0B final): 0x%02X", pull_enable);
    }
    if (this->read_reg(PI4IOE5V6408_PULL_UP_DOWN_SELECT_REG, &pull_select)) {
        ESP_LOGCONFIG(TAG, "  PUD Select Reg (0x0D final): 0x%02X", pull_select);
    }
  }

  uint32_t interval = this->get_update_interval();
  if (interval == 0) {
    ESP_LOGCONFIG(TAG, "  Update Interval: ALWAYS (every loop)");
  } else if (interval == UINT32_MAX) { // UINT32_MAX is from <cstdint>
    ESP_LOGCONFIG(TAG, "  Update Interval: NEVER");
  } else {
    ESP_LOGCONFIG(TAG, "  Configured Update Interval: %" PRIu32 " ms", interval);
  }
}

void PI4IOE5V6408::update() {
  if (this->is_failed())
    return;

  uint8_t new_input_val;
  if (!this->read_reg(PI4IOE5V6408_INPUT_STATUS_REG, &new_input_val)) {
    ESP_LOGW(TAG, "Failed to read INPUT_STATUS_REG (0x%02X) during update", PI4IOE5V6408_INPUT_STATUS_REG);
    return;
  }
  if (new_input_val != this->input_status_cache_) {
      ESP_LOGV(TAG, "Input status (Reg 0x%02X) changed from 0x%02X to 0x%02X", PI4IOE5V6408_INPUT_STATUS_REG, this->input_status_cache_, new_input_val);
      this->input_status_cache_ = new_input_val;
  }
}

bool PI4IOE5V6408::digital_read_cached(uint8_t pin) {
  if (this->is_failed() || pin >= 8) {
    return false; 
  }
  return (this->input_status_cache_ >> pin) & 0x01;
}

void PI4IOE5V6408::digital_write(uint8_t pin, bool value) {
  if (this->is_failed() || pin >= 8) {
    ESP_LOGW(TAG, "digital_write: Component failed or pin P%u out of range.", pin);
    return;
  }

  uint8_t old_output_state_cache = this->output_state_cache_;
  if (value) {
    this->output_state_cache_ |= (1 << pin);
  } else {
    this->output_state_cache_ &= ~(1 << pin);
  }

  ESP_LOGD(TAG, "Pin P%u write %s. Output state cache changing from 0x%02X to 0x%02X. Writing to OUTPUT_STATE_REG (0x%02X)",
           pin, value ? "ON" : "OFF", old_output_state_cache, this->output_state_cache_, PI4IOE5V6408_OUTPUT_STATE_REG);
  if (!this->write_reg(PI4IOE5V6408_OUTPUT_STATE_REG, this->output_state_cache_)) {
    ESP_LOGW(TAG, "Failed to write 0x%02X to OUTPUT_STATE_REG (0x%02X). Reverting cache.", this->output_state_cache_, PI4IOE5V6408_OUTPUT_STATE_REG);
    this->output_state_cache_ = old_output_state_cache;
  } else {
    ESP_LOGV(TAG, "Successfully wrote 0x%02X to OUTPUT_STATE_REG (0x%02X)", this->output_state_cache_, PI4IOE5V6408_OUTPUT_STATE_REG);
  }
}

void PI4IOE5V6408::pin_mode(uint8_t pin, gpio::Flags flags) {
  if (this->is_failed() || pin >= 8) {
    ESP_LOGW(TAG, "pin_mode: Component failed or pin P%u out of range.", pin);
    return;
  }

  uint8_t old_io_direction_cache = this->io_direction_cache_;
  uint8_t old_output_high_z_cache = this->output_high_z_cache_;

  ESP_LOGD(TAG, "Pin P%u mode change. Current IO Direction: 0x%02X, High-Z: 0x%02X. Requested flags: 0x%02X", 
           pin, this->io_direction_cache_, this->output_high_z_cache_, static_cast<uint8_t>(flags));

  if ((flags & gpio::FLAG_INPUT) == gpio::FLAG_INPUT) {
    this->io_direction_cache_ &= ~(1 << pin); 
    ESP_LOGD(TAG, "Pin P%u set to INPUT. Target IO Direction: 0x%02X", pin, this->io_direction_cache_);

    if (pin == 0 || pin == 1 || pin == 2) {
        ESP_LOGD(TAG, "Configuring internal pull-up for button pin P%u.", pin);
        uint8_t pull_enable_val, pull_select_val;
        bool pud_setup_success = true;

        if (this->read_reg(PI4IOE5V6408_PULL_UP_DOWN_ENABLE_REG, &pull_enable_val)) {
            uint8_t original_pull_enable_val = pull_enable_val;
            pull_enable_val |= (1 << pin); 
            if (pull_enable_val != original_pull_enable_val) {
                 ESP_LOGD(TAG, "PUD_ENABLE_REG (0x0B) for P%u: changing from 0x%02X to 0x%02X", pin, original_pull_enable_val, pull_enable_val);
                if(!this->write_reg(PI4IOE5V6408_PULL_UP_DOWN_ENABLE_REG, pull_enable_val)) {
                    ESP_LOGW(TAG, "Failed to write 0x%02X to PUD_ENABLE_REG (0x0B) for P%u", pull_enable_val, pin);
                    pud_setup_success = false;
                } else {
                    ESP_LOGV(TAG, "Wrote 0x%02X to PUD_ENABLE_REG (0x0B) for P%u pull-up", pull_enable_val, pin);
                }
            } else {
                 ESP_LOGV(TAG, "PUD_ENABLE_REG (0x0B) for P%u already 0x%02X (or bit already set), no write needed.", pin, pull_enable_val);
            }
        } else {
            ESP_LOGW(TAG, "Failed to read PUD_ENABLE_REG (0x0B) for P%u pull-up setup.", pin);
            pud_setup_success = false;
        }

        if (pud_setup_success && this->read_reg(PI4IOE5V6408_PULL_UP_DOWN_SELECT_REG, &pull_select_val)) {
            uint8_t original_pull_select_val = pull_select_val;
            pull_select_val |= (1 << pin); 
             if (pull_select_val != original_pull_select_val) {
                ESP_LOGD(TAG, "PUD_SELECT_REG (0x0D) for P%u: changing from 0x%02X to 0x%02X", pin, original_pull_select_val, pull_select_val);
                if(!this->write_reg(PI4IOE5V6408_PULL_UP_DOWN_SELECT_REG, pull_select_val)) {
                    ESP_LOGW(TAG, "Failed to write 0x%02X to PUD_SELECT_REG (0x0D) for P%u", pull_select_val, pin);
                } else {
                    ESP_LOGV(TAG, "Wrote 0x%02X to PUD_SELECT_REG (0x0D) for P%u pull-up", pull_select_val, pin);
                }
            } else {
                ESP_LOGV(TAG, "PUD_SELECT_REG (0x0D) for P%u already 0x%02X (or bit already set), no write needed.", pin, pull_select_val);
            }
        } else if (pud_setup_success) {
            ESP_LOGW(TAG, "Failed to read PUD_SELECT_REG (0x0D) for P%u pull-up setup.", pin);
        }
    }
  } else if ((flags & gpio::FLAG_OUTPUT) == gpio::FLAG_OUTPUT) {
    this->io_direction_cache_ |= (1 << pin);  
    this->output_high_z_cache_ &= ~(1 << pin); 
    ESP_LOGD(TAG, "Pin P%u set to OUTPUT. Target IO Direction: 0x%02X, Target High-Z: 0x%02X", pin, this->io_direction_cache_, this->output_high_z_cache_);
  } else {
    ESP_LOGW(TAG, "Pin P%u: Unknown mode flags 0x%02X. Defaulting to INPUT.", pin, static_cast<uint8_t>(flags));
    this->io_direction_cache_ &= ~(1 << pin); 
  }

  if (this->io_direction_cache_ != old_io_direction_cache) {
    ESP_LOGD(TAG, "Writing new IO Direction: 0x%02X to Reg 0x%02X", this->io_direction_cache_, PI4IOE5V6408_IO_DIRECTION_REG);
    if (!this->write_reg(PI4IOE5V6408_IO_DIRECTION_REG, this->io_direction_cache_)) {
      ESP_LOGW(TAG, "Failed to write 0x%02X to IO_DIRECTION_REG (0x%02X). Reverting cache.", this->io_direction_cache_, PI4IOE5V6408_IO_DIRECTION_REG);
      this->io_direction_cache_ = old_io_direction_cache;
    } else {
      ESP_LOGV(TAG, "Successfully wrote 0x%02X to IO_DIRECTION_REG (0x%02X)", this->io_direction_cache_, PI4IOE5V6408_IO_DIRECTION_REG);
    }
  } else {
    ESP_LOGV(TAG, "IO Direction cache 0x%02X did not change. No write to Reg 0x%02X needed.", this->io_direction_cache_, PI4IOE5V6408_IO_DIRECTION_REG);
  }

  if (this->output_high_z_cache_ != old_output_high_z_cache) {
    ESP_LOGD(TAG, "Writing new Output High-Z: 0x%02X to Reg 0x%02X", this->output_high_z_cache_, PI4IOE5V6408_OUTPUT_HIGH_Z_REG);
    if (!this->write_reg(PI4IOE5V6408_OUTPUT_HIGH_Z_REG, this->output_high_z_cache_)) {
      ESP_LOGW(TAG, "Failed to write 0x%02X to OUTPUT_HIGH_Z_REG (0x%02X). Reverting cache.", this->output_high_z_cache_, PI4IOE5V6408_OUTPUT_HIGH_Z_REG);
      this->output_high_z_cache_ = old_output_high_z_cache;
    } else {
      ESP_LOGV(TAG, "Successfully wrote 0x%02X to OUTPUT_HIGH_Z_REG (0x%02X)", this->output_high_z_cache_, PI4IOE5V6408_OUTPUT_HIGH_Z_REG);
    }
  } else {
     ESP_LOGV(TAG, "Output High-Z cache 0x%02X did not change. No write to Reg 0x%02X needed.", this->output_high_z_cache_, PI4IOE5V6408_OUTPUT_HIGH_Z_REG);
  }
}

bool PI4IOE5V6408::read_reg(uint8_t reg, uint8_t *value) {
  if (this->is_failed()) {
    return false;
  }
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
  ESP_LOGD(TAG, "PI4IOE5V6408GPIOPin P%u: setup() called. Flags: 0x%02X, Inverted: %s",
           this->pin_, static_cast<uint8_t>(this->flags_), this->inverted_ ? "true" : "false");
  if (this->parent_ == nullptr) {
      ESP_LOGE(TAG, "PI4IOE5V6408GPIOPin P%u: Parent is NULL in setup!", this->pin_);
      return;
  }
  this->pin_mode(this->flags_);
}

void PI4IOE5V6408GPIOPin::pin_mode(gpio::Flags flags) {
  this->flags_ = flags;
  if (this->parent_ == nullptr) { 
    ESP_LOGE(TAG, "PI4IOE5V6408GPIOPin P%u: Parent is NULL in pin_mode!", this->pin_); 
    return; 
  }
  this->parent_->pin_mode(this->pin_, flags);
}

bool PI4IOE5V6408GPIOPin::digital_read() {
  if (this->parent_ == nullptr) { 
    ESP_LOGE(TAG, "PI4IOE5V6408GPIOPin P%u: Parent is NULL in digital_read!", this->pin_); 
    return this->inverted_ ? true : false;
  }
  return this->parent_->digital_read_cached(this->pin_) != this->inverted_;
}

void PI4IOE5V6408GPIOPin::digital_write(bool value) {
  if (this->parent_ == nullptr) { 
    ESP_LOGE(TAG, "PI4IOE5V6408GPIOPin P%u: Parent is NULL in digital_write!", this->pin_); 
    return; 
  }
  this->parent_->digital_write(this->pin_, value != this->inverted_);
}

std::string PI4IOE5V6408GPIOPin::dump_summary() const {
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "Pin P%u via PI4IOE5V6408 (I2C @ 0x%02X)",
           this->pin_,
           (this->parent_ ? this->parent_->get_i2c_address() : 0xFF));
  return buffer;
}

}  // namespace pi4ioe5v6408
}  // namespace esphome
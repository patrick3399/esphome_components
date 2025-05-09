#include "pi4ioe5v6408.h"
#include "esphome/core/log.h"

namespace esphome {
namespace pi4ioe5v6408 {

static const char *const TAG = "pi4ioe5v6408";

// PI4IOE5V6408 Register Command Bytes (from datasheet)
static const uint8_t CMD_DEVICE_ID_CONTROL = 0x01; 
static const uint8_t CMD_IO_DIRECTION = 0x03;      
static const uint8_t CMD_OUTPUT_STATE = 0x05;      
static const uint8_t CMD_INPUT_STATUS = 0x0F;      

void PI4IOE5V6408::setup() {
  ESP_LOGCONFIG(TAG, "Setting up PI4IOE5V6408 (Address: 0x%02X)...", this->address_);

  uint8_t device_id;
  if (!this->read_byte_wrapper(CMD_DEVICE_ID_CONTROL, &device_id, "CMD_DEVICE_ID_CONTROL")) { // Call with 3 args
    ESP_LOGE(TAG, "Failed to read Device ID.");
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "PI4IOE5V6408 Device ID/Control reg: 0x%02X.", device_id);
  
  this->output_latch_ = 0x00;
  ESP_LOGD(TAG, "Initializing output latch to 0x00.");
  
  if (!this->write_output_register(this->output_latch_)) {
     ESP_LOGW(TAG, "Failed to write initial output latch (0x00) during setup.");
  }

  ESP_LOGCONFIG(TAG, "PI4IOE5V6408 setup complete. Pin modes will be set by GPIOPin instances.");
}

void PI4IOE5V6408::dump_config() {
  ESP_LOGCONFIG(TAG, "PI4IOE5V6408:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with PI4IOE5V6408 failed!");
  }
  uint8_t iodir, out_st, in_st;
  if (this->read_config_register(&iodir)) {
    ESP_LOGCONFIG(TAG, "  IO Direction (0x03): 0x%02X", iodir);
  } else {
    ESP_LOGCONFIG(TAG, "  IO Direction (0x03): Read Failed");
  }
  if (this->read_byte_wrapper(CMD_OUTPUT_STATE, &out_st, "CMD_OUTPUT_STATE (for dump_config)")) { // Call with 3 args
     ESP_LOGCONFIG(TAG, "  Output State (0x05): 0x%02X (Hardware value, internal latch is 0x%02X)", out_st, this->output_latch_);
  } else {
     ESP_LOGCONFIG(TAG, "  Output State (0x05): Read Failed (internal latch is 0x%02X)", this->output_latch_);
  }
  if (this->read_input_register(&in_st)) {
    ESP_LOGCONFIG(TAG, "  Input Status (0x0F): 0x%02X", in_st);
  } else {
    ESP_LOGCONFIG(TAG, "  Input Status (0x0F): Read Failed");
  }
}

// Definition with 3 arguments
bool PI4IOE5V6408::read_byte_wrapper(uint8_t command_byte, uint8_t *value, const char *cmd_name) {
  ESP_LOGV(TAG, "I2C Read from %s (CMD 0x%02X): Sending command byte...", cmd_name, command_byte);
  if (this->write(&command_byte, 1) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "I2C Write of command byte 0x%02X for %s FAILED!", command_byte, cmd_name);
    return false;
  }
  ESP_LOGV(TAG, "I2C Read from %s (CMD 0x%02X): Reading data byte...", cmd_name, command_byte);
  if (this->read(value, 1) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "I2C Read after command 0x%02X for %s FAILED!", command_byte, cmd_name);
    return false;
  }
  ESP_LOGV(TAG, "I2C Read from %s (CMD 0x%02X) successful, value: 0x%02X", cmd_name, command_byte, *value);
  return true;
}

// Definition with 3 arguments
bool PI4IOE5V6408::write_byte_wrapper(uint8_t command_byte, uint8_t value, const char *cmd_name) {
  uint8_t buffer[2] = {command_byte, value};
  ESP_LOGV(TAG, "I2C Write to %s (CMD 0x%02X) with value 0x%02X...", cmd_name, command_byte, value);
  if (this->write(buffer, 2) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "I2C Write to %s (CMD 0x%02X) with value 0x%02X FAILED!", cmd_name, command_byte, value);
    return false;
  }
  ESP_LOGV(TAG, "I2C Write to %s (CMD 0x%02X) with value 0x%02X successful.", cmd_name, command_byte, value);
  return true;
}

bool PI4IOE5V6408::read_config_register(uint8_t *value) {
    return this->read_byte_wrapper(CMD_IO_DIRECTION, value, "CMD_IO_DIRECTION");
}

bool PI4IOE5V6408::write_config_register(uint8_t value) {
    return this->write_byte_wrapper(CMD_IO_DIRECTION, value, "CMD_IO_DIRECTION");
}

bool PI4IOE5V6408::read_input_register(uint8_t *value) {
    ESP_LOGV(TAG, "Reading PI4IOE5V6408 Input Register (CMD_INPUT_STATUS 0x%02X)", CMD_INPUT_STATUS);
    return this->read_byte_wrapper(CMD_INPUT_STATUS, value, "CMD_INPUT_STATUS");
}

bool PI4IOE5V6408::write_output_register(uint8_t value) {
    return this->write_byte_wrapper(CMD_OUTPUT_STATE, value, "CMD_OUTPUT_STATE");
}


void PI4IOE5V6408::pin_mode(uint8_t pin, gpio::Flags flags) {
  uint8_t current_config;
  ESP_LOGD(TAG, "Pin mode for P%u: Reading current IO_DIRECTION (0x%02X)...", pin, CMD_IO_DIRECTION);
  if (!this->read_config_register(&current_config)) {
    ESP_LOGE(TAG, "Pin P%u: Failed to read IO_DIRECTION for pin_mode.", pin);
    this->mark_failed(); 
    return;
  }
  ESP_LOGV(TAG, "Pin P%u: Current IO_DIRECTION value: 0x%02X", pin, current_config);

  bool set_to_input = false;
  uint8_t new_config = current_config;

  if ((flags & gpio::FLAG_INPUT) && !(flags & gpio::FLAG_OUTPUT)) {
    new_config |= (1 << pin); 
    set_to_input = true;
  } else if ((flags & gpio::FLAG_OUTPUT) && !(flags & gpio::FLAG_INPUT)) {
    new_config &= ~(1 << pin); 
    set_to_input = false;
  } else {
    ESP_LOGW(TAG, "Pin P%u: Invalid mode flags 0x%X. Defaulting to input.", pin, static_cast<uint8_t>(flags));
    new_config |= (1 << pin);
    set_to_input = true;
  }

  if (new_config != current_config) {
    ESP_LOGD(TAG, "Pin P%u: Setting as %s. New IO_DIRECTION to write: 0x%02X (old: 0x%02X)", 
             pin, set_to_input ? "INPUT (1)" : "OUTPUT (0)", new_config, current_config);
    if (!this->write_config_register(new_config)) {
      ESP_LOGE(TAG, "Pin P%u: Failed to write IO_DIRECTION for pin_mode.", pin);
      this->mark_failed(); 
    }
  } else {
    ESP_LOGV(TAG, "Pin P%u: Mode already set as %s. IO_DIRECTION (0x%02X) is 0x%02X.", 
             pin, set_to_input ? "INPUT" : "OUTPUT", CMD_IO_DIRECTION, current_config);
  }
}

bool PI4IOE5V6408::digital_read(uint8_t pin) {
  uint8_t port_value = 0;
  ESP_LOGV(TAG, "Digital read for P%u. Target CMD_INPUT_STATUS (0x%02X)", pin, CMD_INPUT_STATUS);
  if (!this->read_input_register(&port_value)) {
    ESP_LOGW(TAG, "Failed to read CMD_INPUT_STATUS for pin P%u", pin);
    return false; 
  }
  bool result = (port_value >> pin) & 0x01;
  ESP_LOGV(TAG, "Read CMD_INPUT_STATUS value: 0x%02X. Pin P%u state: %s", 
           port_value, pin, ONOFF(result));
  return result;
}

void PI4IOE5V6408::digital_write(uint8_t pin, bool value) {
  ESP_LOGV(TAG, "Digital write for P%u: %s. Current output_latch: 0x%02X", pin, ONOFF(value), this->output_latch_);
  if (value) {
    this->output_latch_ |= (1 << pin);
  } else {
    this->output_latch_ &= ~(1 << pin);
  }
  ESP_LOGD(TAG, "Writing to P%u: %s. New output_latch: 0x%02X. Target CMD_OUTPUT_STATE (0x%02X)", 
           pin, ONOFF(value), this->output_latch_, CMD_OUTPUT_STATE);
  if (!this->write_output_register(this->output_latch_)) {
    ESP_LOGE(TAG, "Failed to write CMD_OUTPUT_STATE for pin P%u", pin);
  }
}

// PI4IOE5V6408GPIOPin implementation
void PI4IOE5V6408GPIOPin::setup() {
  ESP_LOGV(TAG, "Setting up PI4IOE5V6408GPIOPin %u with flags 0x%X", pin_, static_cast<uint8_t>(flags_));
  this->pin_mode(flags_); 
}

void PI4IOE5V6408GPIOPin::pin_mode(gpio::Flags flags) {
  this->parent_->pin_mode(this->pin_, flags);
}

bool PI4IOE5V6408GPIOPin::digital_read() {
  bool parent_read = this->parent_->digital_read(this->pin_);
  bool final_read = parent_read != this->inverted_;
  ESP_LOGV(TAG, "PI4IOE5V6408GPIOPin %u digital_read: parent_val=%s, inverted=%s, final_val=%s", 
           pin_, ONOFF(parent_read), ONOFF(this->inverted_), ONOFF(final_read));
  return final_read;
}

void PI4IOE5V6408GPIOPin::digital_write(bool value) {
  bool value_to_write = value != this->inverted_;
  ESP_LOGV(TAG, "PI4IOE5V6408GPIOPin %u digital_write: requested_val=%s, inverted=%s, final_val_to_chip=%s",
           pin_, ONOFF(value), ONOFF(this->inverted_), ONOFF(value_to_write));
  this->parent_->digital_write(this->pin_, value_to_write);
}

std::string PI4IOE5V6408GPIOPin::dump_summary() const {
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "Pin %u (PI4IOE5V6408) Mode: %s Inverted: %s", 
           pin_, (flags_ & gpio::FLAG_INPUT) ? "Input" : "Output", ONOFF(inverted_));
  return buffer;
}

}  // namespace pi4ioe5v6408
}  // namespace esphome

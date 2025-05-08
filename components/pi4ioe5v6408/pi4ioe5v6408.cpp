#include "pi4ioe5v6408.h"
#include "esphome/core/log.h"

namespace esphome {
namespace pi4ioe5v6408 {

static const char *const TAG = "pi4ioe5v6408";

// PI4IOE5V6408 Register Command Bytes (from datasheet)
static const uint8_t CMD_DEVICE_ID_CONTROL = 0x01; // R/W, Default A2h
static const uint8_t CMD_IO_DIRECTION = 0x03;      // R/W, Default 00h (all output). 0=Output, 1=Input.
static const uint8_t CMD_OUTPUT_STATE = 0x05;      // R/W, Default 00h
static const uint8_t CMD_INPUT_STATUS = 0x0F;      // R,   Default xxh

void PI4IOE5V6408::setup() {
  ESP_LOGCONFIG(TAG, "Setting up PI4IOE5V6408...");

  // Verify device presence by reading Device ID (optional but good practice)
  uint8_t device_id;
  if (!this->read_byte_wrapper(CMD_DEVICE_ID_CONTROL, &device_id)) {
    ESP_LOGE(TAG, "Failed to read Device ID. Check I2C address (0x%02X) and wiring.", this->address_);
    this->mark_failed();
    return;
  }
  // The actual Device ID might vary or not be strictly A2h if control bits are changed.
  // For now, just log it. A more robust check might be needed if this causes issues.
  ESP_LOGI(TAG, "PI4IOE5V6408 Device ID/Control reg: 0x%02X. Found at I2C address 0x%02X", device_id, this->address_);
  
  // Initialize output_latch_ to 0x00 (all outputs low by default if configured as output)
  this->output_latch_ = 0x00;
  ESP_LOGD(TAG, "Initializing output latch to 0x00.");
  
  // Write the initial output_latch_ to the output state register.
  // Pins not configured as output won't be affected if their direction is input.
  if (!this->write_output_register(this->output_latch_)) {
     ESP_LOGW(TAG, "Failed to write initial output latch (0x00) during setup. This might be okay if pins are inputs.");
     // Don't mark as failed for this, pin_mode will set directions.
  }

  // Pin directions will be set by individual GPIOPin setup calls via pin_mode.
  // The chip defaults to all outputs (CMD_IO_DIRECTION = 0x00).
  // The pin_mode calls will override this based on YAML.
  ESP_LOGCONFIG(TAG, "PI4IOE5V6408 setup complete. Pin modes will be set individually.");
}

void PI4IOE5V6408::dump_config() {
  ESP_LOGCONFIG(TAG, "PI4IOE5V6408:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with PI4IOE5V6408 failed!");
  }
}

bool PI4IOE5V6408::read_byte_wrapper(uint8_t command_byte, uint8_t *value) {
  // For PI4IOE5V6408, reading requires sending the command byte first, then reading.
  // The i2c::I2CDevice::read_byte function assumes the register address is the first byte written.
  if (this->write(&command_byte, 1) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "I2C Write of command byte 0x%02X failed for read operation!", command_byte);
    return false;
  }
  if (this->read(value, 1) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "I2C Read after command 0x%02X failed!", command_byte);
    return false;
  }
  return true;
}

bool PI4IOE5V6408::write_byte_wrapper(uint8_t command_byte, uint8_t value) {
  // For PI4IOE5V6408, writing involves sending the command byte, then the data byte.
  uint8_t buffer[2] = {command_byte, value};
  if (this->write(buffer, 2) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "I2C Write to command 0x%02X with value 0x%02X failed!", command_byte, value);
    return false;
  }
  return true;
}

bool PI4IOE5V6408::read_config_register(uint8_t *value) {
    return this->read_byte_wrapper(CMD_IO_DIRECTION, value);
}

bool PI4IOE5V6408::write_config_register(uint8_t value) {
    return this->write_byte_wrapper(CMD_IO_DIRECTION, value);
}

bool PI4IOE5V6408::read_input_register(uint8_t *value) {
    return this->read_byte_wrapper(CMD_INPUT_STATUS, value);
}

bool PI4IOE5V6408::write_output_register(uint8_t value) {
    return this->write_byte_wrapper(CMD_OUTPUT_STATE, value);
}


void PI4IOE5V6408::pin_mode(uint8_t pin, gpio::Flags flags) {
  uint8_t current_config;
  if (!this->read_config_register(&current_config)) {
    ESP_LOGE(TAG, "Pin %u: Failed to read I/O Direction register (0x%02X) for pin_mode.", pin, CMD_IO_DIRECTION);
    this->mark_failed(); // Or handle more gracefully
    return;
  }
  ESP_LOGD(TAG, "Pin %u: Current I/O Direction Register (0x%02X) value: 0x%02X", pin, CMD_IO_DIRECTION, current_config);

  bool set_to_input = false;
  if ((flags & gpio::FLAG_INPUT) && !(flags & gpio::FLAG_OUTPUT)) {
    current_config |= (1 << pin); // Set bit to 1 for Input
    set_to_input = true;
  } else if ((flags & gpio::FLAG_OUTPUT) && !(flags & gpio::FLAG_INPUT)) {
    current_config &= ~(1 << pin); // Clear bit to 0 for Output
    set_to_input = false;
  } else {
    ESP_LOGW(TAG, "Pin %u: Invalid mode flags 0x%X. Defaulting to input.", pin, static_cast<uint8_t>(flags));
    current_config |= (1 << pin);
    set_to_input = true;
  }

  ESP_LOGD(TAG, "Pin %u: Setting as %s. New I/O Direction Register value to write: 0x%02X", pin, set_to_input ? "INPUT" : "OUTPUT", current_config);
  if (!this->write_config_register(current_config)) {
    ESP_LOGE(TAG, "Pin %u: Failed to write I/O Direction register (0x%02X) for pin_mode.", pin, CMD_IO_DIRECTION);
    this->mark_failed(); // Or handle more gracefully
  }
}

bool PI4IOE5V6408::digital_read(uint8_t pin) {
  uint8_t port_value = 0;
  if (!this->read_input_register(&port_value)) {
    ESP_LOGW(TAG, "Failed to read Input Status Register (0x%02X) for pin %u", CMD_INPUT_STATUS, pin);
    return false; // Or handle error appropriately
  }
  return (port_value >> pin) & 0x01;
}

void PI4IOE5V6408::digital_write(uint8_t pin, bool value) {
  if (value) {
    this->output_latch_ |= (1 << pin);
  } else {
    this->output_latch_ &= ~(1 << pin);
  }
  ESP_LOGV(TAG, "Writing %s to pin %u. Output latch: 0x%02X", ONOFF(value), pin, this->output_latch_);
  if (!this->write_output_register(this->output_latch_)) {
    ESP_LOGE(TAG, "Failed to write Output State Register (0x%02X) for pin %u", CMD_OUTPUT_STATE, pin);
  }
}

// PI4IOE5V6408GPIOPin implementation
void PI4IOE5V6408GPIOPin::setup() {
  ESP_LOGV(TAG, "Setting up PI4IOE5V6408GPIOPin %u", pin_);
  this->pin_mode(flags_); // Initial mode setup
}

void PI4IOE5V6408GPIOPin::pin_mode(gpio::Flags flags) {
  this->parent_->pin_mode(this->pin_, flags);
}

bool PI4IOE5V6408GPIOPin::digital_read() {
  return this->parent_->digital_read(this->pin_) != this->inverted_;
}

void PI4IOE5V6408GPIOPin::digital_write(bool value) {
  this->parent_->digital_write(this->pin_, value != this->inverted_);
}

std::string PI4IOE5V6408GPIOPin::dump_summary() const {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "Pin %u (PI4IOE5V6408)", pin_);
  return buffer;
}

}  // namespace pi4ioe5v6408
}  // namespace esphome

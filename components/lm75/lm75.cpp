#include "lm75.h"
#include "esphome/core/log.h"

namespace esphome {
namespace lm75 {

static const uint8_t LM75_REGISTER_TEMP = 0x00;   //  Temperature register
static const uint8_t LM75_REGISTER_CONFIG = 0x01; //  Configuration register
static const uint8_t LM75_REGISTER_THYST = 0x02;  //  Hysterisis register
static const uint8_t LM75_REGISTER_TOS = 0x03;    //  OS register

static const char *const TAG = "lm75";

static const float LM75_CONVERSION_FACTOR = 0.125;
static const uint8_t LM75_REGISTER_DATA_SHIFT = 5;

void LM75Component::setup() { ESP_LOGCONFIG(TAG, "Setting up LM75..."); }

void LM75Component::dump_config() {
  ESP_LOGCONFIG(TAG, "LM75:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with LM75 failed!");
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Temperature", this);
}

void LM75Component::update() {
  uint16_t raw_temperature;
  if (this->write(&LM75_REGISTER_TEMP, 1) != i2c::ERROR_OK) {
    this->status_set_warning();
    return;
  }
  if (this->read(reinterpret_cast<uint8_t *>(&raw_temperature), 2) != i2c::ERROR_OK) {
    this->status_set_warning();
    return;
  }
  raw_temperature = i2c::i2ctohs(raw_temperature);
  raw_temperature = raw_temperature >> LM75_REGISTER_DATA_SHIFT;
  float temperature = raw_temperature * LM75_CONVERSION_FACTOR;
  ESP_LOGD(TAG, "Got Temperature=%.1f°C", temperature);

  this->publish_state(temperature);
  this->status_clear_warning();
}

float LM75Component::get_setup_priority() const { return setup_priority::DATA; }

} // namespace lm75
} // namespace esphome

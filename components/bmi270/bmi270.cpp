#include "bmi270.h"
#include "bmi270_config.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <algorithm>
#include <cstring>

namespace esphome {
namespace bmi270 {

static const char *const TAG = "bmi270";

static const uint8_t REG_CHIP_ID        = 0x00;
static const uint8_t REG_INTERNAL_STATUS = 0x21;
static const uint8_t REG_ACC_DATA       = 0x0C;
static const uint8_t REG_TEMP_DATA      = 0x22;
static const uint8_t REG_ACC_CONF       = 0x40;
static const uint8_t REG_ACC_RANGE      = 0x41;
static const uint8_t REG_GYR_CONF       = 0x42;
static const uint8_t REG_GYR_RANGE      = 0x43;
static const uint8_t REG_INIT_CTRL      = 0x59;
static const uint8_t REG_INIT_ADDR_0    = 0x5B;
static const uint8_t REG_INIT_ADDR_1    = 0x5C;
static const uint8_t REG_INIT_DATA      = 0x5E;
static const uint8_t REG_PWR_CONF       = 0x7C;
static const uint8_t REG_PWR_CTRL       = 0x7D;
static const uint8_t REG_CMD            = 0x7E;

static const uint8_t CHIP_ID            = 0x24;
static const uint8_t CMD_SOFT_RESET     = 0xB6;

void BMI270Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up BMI270...");

  uint8_t val = CMD_SOFT_RESET;
  this->write_register(REG_CMD, &val, 1);
  delay(100);

  uint8_t chip_id = 0;
  if (this->read_register(REG_CHIP_ID, &chip_id, 1) != i2c::ERROR_OK || chip_id != CHIP_ID) {
    ESP_LOGE(TAG, "Unexpected chip ID: 0x%02X (expected 0x%02X)", chip_id, CHIP_ID);
    this->mark_failed();
    return;
  }

  if (!this->bmi270_init_config_file()) {
    ESP_LOGE(TAG, "Failed to load BMI270 config file");
    this->mark_failed();
    return;
  }

  // Enable accel, gyro, temperature
  val = 0x0E;
  this->write_register(REG_PWR_CTRL, &val, 1);
  delay(50);

  // Accel: ODR=100Hz, BWP=normal, perf mode, range=±2g → 16384 LSB/g
  uint8_t acc_conf = 0xA8, acc_range = 0x00;
  this->write_register(REG_ACC_CONF, &acc_conf, 1);
  this->write_register(REG_ACC_RANGE, &acc_range, 1);
  this->accel_sensitivity_ = 16384.0f;

  // Gyro: ODR=100Hz, BWP=normal, perf mode, range=±2000 dps → 16.4 LSB/dps
  uint8_t gyr_conf = 0xA8, gyr_range = 0x00;
  this->write_register(REG_GYR_CONF, &gyr_conf, 1);
  this->write_register(REG_GYR_RANGE, &gyr_range, 1);
  this->gyro_sensitivity_ = 16.4f;

  this->apply_power_save_mode();
  this->sensors_active_ = true;
  ESP_LOGI(TAG, "BMI270 setup complete");
}

bool BMI270Component::bmi270_init_config_file() {
  // Disable advanced power save before writing config
  uint8_t val = 0x00;
  this->write_register(REG_PWR_CONF, &val, 1);
  delay(1);

  val = 0x00;
  this->write_register(REG_INIT_CTRL, &val, 1);

  const size_t config_len = sizeof(bmi270_config_file);
  uint8_t chunk[32];
  for (size_t i = 0; i < config_len; i += 32) {
    uint16_t addr = (uint16_t)(i / 2);
    uint8_t addr_lo = (uint8_t)(addr & 0xFF);
    uint8_t addr_hi = (uint8_t)((addr >> 8) & 0xFF);
    this->write_register(REG_INIT_ADDR_0, &addr_lo, 1);
    this->write_register(REG_INIT_ADDR_1, &addr_hi, 1);

    size_t chunk_size = std::min((size_t)32, config_len - i);
    memcpy(chunk, bmi270_config_file + i, chunk_size);
    this->write_register(REG_INIT_DATA, chunk, chunk_size);
  }

  val = 0x01;
  this->write_register(REG_INIT_CTRL, &val, 1);
  delay(20);

  uint8_t status = 0;
  if (this->read_register(REG_INTERNAL_STATUS, &status, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to read INTERNAL_STATUS");
    return false;
  }
  if ((status & 0x0F) != 0x01) {
    ESP_LOGE(TAG, "BMI270 init failed, INTERNAL_STATUS=0x%02X", status);
    return false;
  }

  ESP_LOGI(TAG, "BMI270 config file loaded successfully");
  return true;
}

void BMI270Component::apply_power_save_mode() {
  uint8_t val = (this->power_save_mode_ == POWER_SAVE_MODE_LOW_POWER) ? 0x03 : 0x00;
  this->write_register(REG_PWR_CONF, &val, 1);
}

void BMI270Component::update() {
  if (!this->sensors_active_)
    return;

  // Read accel (0x0C–0x11) + gyro (0x12–0x17) in one burst: 12 bytes
  uint8_t data[12];
  if (this->read_register(REG_ACC_DATA, data, 12) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Failed to read sensor data");
    return;
  }

  auto raw = [&](int i) -> int16_t { return (int16_t)((data[i + 1] << 8) | data[i]); };

  if (this->accel_x_sensor_) this->accel_x_sensor_->publish_state(raw(0) / this->accel_sensitivity_);
  if (this->accel_y_sensor_) this->accel_y_sensor_->publish_state(raw(2) / this->accel_sensitivity_);
  if (this->accel_z_sensor_) this->accel_z_sensor_->publish_state(raw(4) / this->accel_sensitivity_);
  if (this->gyro_x_sensor_)  this->gyro_x_sensor_->publish_state(raw(6) / this->gyro_sensitivity_);
  if (this->gyro_y_sensor_)  this->gyro_y_sensor_->publish_state(raw(8) / this->gyro_sensitivity_);
  if (this->gyro_z_sensor_)  this->gyro_z_sensor_->publish_state(raw(10) / this->gyro_sensitivity_);

  if (this->temperature_sensor_) {
    uint8_t temp_data[2];
    if (this->read_register(REG_TEMP_DATA, temp_data, 2) == i2c::ERROR_OK) {
      int16_t raw_temp = (int16_t)((temp_data[1] << 8) | temp_data[0]);
      this->temperature_sensor_->publish_state((raw_temp / 512.0f) + 23.0f);
    }
  }
}

void BMI270Component::dump_config() {
  ESP_LOGCONFIG(TAG, "BMI270:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "BMI270 communication failed");
  }
}

float BMI270Component::get_setup_priority() const { return setup_priority::DATA; }

}  // namespace bmi270
}  // namespace esphome

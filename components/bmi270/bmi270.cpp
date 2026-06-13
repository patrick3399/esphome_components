#include "bmi270.h"
#include "bmi270_config.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace esphome {
namespace bmi270 {

static const char *const TAG = "bmi270";

// Register map
static const uint8_t BMI270_REG_CHIP_ID = 0x00;
static const uint8_t BMI270_REG_INTERNAL_STATUS = 0x21;
static const uint8_t BMI270_REG_ACC_DATA = 0x0C;
static const uint8_t BMI270_REG_TEMP_DATA = 0x22;
static const uint8_t BMI270_REG_ACC_CONF = 0x40;
static const uint8_t BMI270_REG_ACC_RANGE = 0x41;
static const uint8_t BMI270_REG_GYR_CONF = 0x42;
static const uint8_t BMI270_REG_GYR_RANGE = 0x43;
static const uint8_t BMI270_REG_INIT_CTRL = 0x59;
static const uint8_t BMI270_REG_INIT_ADDR_0 = 0x5B;
static const uint8_t BMI270_REG_INIT_ADDR_1 = 0x5C;
static const uint8_t BMI270_REG_INIT_DATA = 0x5E;
static const uint8_t BMI270_REG_PWR_CONF = 0x7C;
static const uint8_t BMI270_REG_PWR_CTRL = 0x7D;
static const uint8_t BMI270_REG_CMD = 0x7E;

static const uint8_t BMI270_CHIP_ID = 0x24;
static const uint8_t BMI270_CMD_SOFT_RESET = 0xB6;

// Sensitivity constants — ±2g range, ±2000 dps range
static const float BMI270_ACCEL_2G_SENSITIVITY = 1.0f / 16384.0f;  // g/LSB
static const float BMI270_GYRO_2000DPS_SENSITIVITY = 1.0f / 16.4f;  // dps/LSB
static const float GRAVITY_EARTH = 9.80665f;

void BMI270Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up BMI270...");

  // Soft reset; datasheet specifies 2 ms POR time after reset
  uint8_t val = BMI270_CMD_SOFT_RESET;
  if (!this->write_checked_(BMI270_REG_CMD, &val, 1, "soft reset")) {
    this->mark_failed();
    return;
  }
  delay(2);

  uint8_t chip_id = 0;
  if (this->read_register(BMI270_REG_CHIP_ID, &chip_id, 1) != i2c::ERROR_OK || chip_id != BMI270_CHIP_ID) {
    ESP_LOGE(TAG, "Unexpected chip ID: 0x%02X (expected 0x%02X)", chip_id, BMI270_CHIP_ID);
    this->mark_failed();
    return;
  }

  if (!this->load_config_file_()) {
    ESP_LOGE(TAG, "Failed to load BMI270 config file");
    this->mark_failed();
    return;
  }

  // Enable accelerometer, gyroscope, temperature sensor
  val = 0x0E;
  if (!this->write_checked_(BMI270_REG_PWR_CTRL, &val, 1, "power control")) {
    this->mark_failed();
    return;
  }
  delay(2);

  // Accel: ODR=100 Hz, BWP=normal, performance mode, ±2 g → 16384 LSB/g
  uint8_t acc_conf = 0xA8, acc_range = 0x00;
  if (!this->write_checked_(BMI270_REG_ACC_CONF, &acc_conf, 1, "accelerometer configuration") ||
      !this->write_checked_(BMI270_REG_ACC_RANGE, &acc_range, 1, "accelerometer range")) {
    this->mark_failed();
    return;
  }

  // Gyro: ODR=100 Hz, BWP=normal, performance mode, ±2000 dps → 16.4 LSB/dps
  uint8_t gyr_conf = 0xA8, gyr_range = 0x00;
  if (!this->write_checked_(BMI270_REG_GYR_CONF, &gyr_conf, 1, "gyroscope configuration") ||
      !this->write_checked_(BMI270_REG_GYR_RANGE, &gyr_range, 1, "gyroscope range")) {
    this->mark_failed();
    return;
  }

  if (!this->apply_power_save_mode_()) {
    this->mark_failed();
    return;
  }
  this->setup_complete_ = true;
  ESP_LOGI(TAG, "BMI270 setup complete");
}

bool BMI270Component::load_config_file_() {
  // Disable advanced power save before uploading config blob
  uint8_t val = 0x00;
  if (!this->write_checked_(BMI270_REG_PWR_CONF, &val, 1, "disable advanced power save"))
    return false;
  delay(1);

  // Halt config load
  if (!this->write_checked_(BMI270_REG_INIT_CTRL, &val, 1, "halt config load"))
    return false;
  delay(2);

  const size_t config_len = sizeof(bmi270_config_file);
  static constexpr size_t CHUNK = 64;
  uint8_t chunk[CHUNK];
  for (size_t i = 0; i < config_len; i += CHUNK) {
    uint16_t addr = static_cast<uint16_t>(i / 2);
    uint8_t addr_buf[2] = {static_cast<uint8_t>(addr & 0x0F), static_cast<uint8_t>((addr >> 4) & 0xFF)};
    if (this->write_register(BMI270_REG_INIT_ADDR_0, addr_buf, 2) != i2c::ERROR_OK) {
      ESP_LOGE(TAG, "BMI270 config upload: I2C address write failed at offset %d", static_cast<int>(i));
      return false;
    }
    size_t chunk_size = std::min(CHUNK, config_len - i);
    memcpy(chunk, bmi270_config_file + i, chunk_size);
    if (this->write_register(BMI270_REG_INIT_DATA, chunk, chunk_size) != i2c::ERROR_OK) {
      ESP_LOGE(TAG, "BMI270 config upload: I2C data write failed at offset %d", static_cast<int>(i));
      return false;
    }
  }

  // Signal config load complete
  val = 0x01;
  if (!this->write_checked_(BMI270_REG_INIT_CTRL, &val, 1, "complete config load"))
    return false;

  // Poll INTERNAL_STATUS until bits[3:0] == 0x01 (init_ok); timeout ~150 ms
  uint8_t status = 0;
  bool init_ok = false;
  for (int attempt = 0; attempt < 15; attempt++) {
    delay(10);
    if (this->read_register(BMI270_REG_INTERNAL_STATUS, &status, 1) != i2c::ERROR_OK) {
      ESP_LOGE(TAG, "Failed to read INTERNAL_STATUS");
      return false;
    }
    if ((status & 0x0F) == 0x01) {
      init_ok = true;
      break;
    }
  }
  this->setup_status_ = status;
  if (!init_ok) {
    ESP_LOGE(TAG, "BMI270 init timed out, INTERNAL_STATUS=0x%02X (expected 0x01)", status);
    return false;
  }
  ESP_LOGI(TAG, "BMI270 config loaded, INTERNAL_STATUS=0x%02X", status);
  return true;
}

bool BMI270Component::apply_power_save_mode_() {
  uint8_t val = (this->power_save_mode_ == POWER_SAVE_MODE_LOW_POWER) ? 0x03 : 0x00;
  return this->write_checked_(BMI270_REG_PWR_CONF, &val, 1, "power save mode");
}

bool BMI270Component::write_checked_(uint8_t reg, const uint8_t *data, size_t length, const char *operation) {
  if (this->write_register(reg, data, length) == i2c::ERROR_OK)
    return true;
  ESP_LOGE(TAG, "I2C write failed during %s (register 0x%02X)", operation, reg);
  return false;
}

void BMI270Component::update() {
  if (!this->setup_complete_)
    return;

  // Burst-read accel (0x0C–0x11) + gyro (0x12–0x17): 12 bytes
  uint8_t data[12];
  if (this->read_register(BMI270_REG_ACC_DATA, data, 12) != i2c::ERROR_OK) {
    this->status_set_warning();
    ESP_LOGW(TAG, "Failed to read sensor data");
    return;
  }

  auto raw = [&](int i) -> int16_t { return static_cast<int16_t>((data[i + 1] << 8) | data[i]); };

  float accel_x = raw(0) * BMI270_ACCEL_2G_SENSITIVITY * GRAVITY_EARTH;
  float accel_y = raw(2) * BMI270_ACCEL_2G_SENSITIVITY * GRAVITY_EARTH;
  float accel_z = raw(4) * BMI270_ACCEL_2G_SENSITIVITY * GRAVITY_EARTH;
  float gyro_x = raw(6) * BMI270_GYRO_2000DPS_SENSITIVITY;
  float gyro_y = raw(8) * BMI270_GYRO_2000DPS_SENSITIVITY;
  float gyro_z = raw(10) * BMI270_GYRO_2000DPS_SENSITIVITY;

  float temp_c = NAN;
  if (this->temperature_sensor_ != nullptr) {
    uint8_t temp_data[2];
    if (this->read_register(BMI270_REG_TEMP_DATA, temp_data, 2) == i2c::ERROR_OK) {
      int16_t raw_temp = static_cast<int16_t>((temp_data[1] << 8) | temp_data[0]);
      temp_c = (raw_temp / 512.0f) + 23.0f;
    }
  }

  ESP_LOGD(TAG,
           "Got accel={x=%.3f m/s², y=%.3f m/s², z=%.3f m/s²}, gyro={x=%.3f °/s, y=%.3f °/s, z=%.3f °/s}, "
           "temp=%.1f°C",
           accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temp_c);

  if (this->accel_x_sensor_ != nullptr)
    this->accel_x_sensor_->publish_state(accel_x);
  if (this->accel_y_sensor_ != nullptr)
    this->accel_y_sensor_->publish_state(accel_y);
  if (this->accel_z_sensor_ != nullptr)
    this->accel_z_sensor_->publish_state(accel_z);
  if (this->gyro_x_sensor_ != nullptr)
    this->gyro_x_sensor_->publish_state(gyro_x);
  if (this->gyro_y_sensor_ != nullptr)
    this->gyro_y_sensor_->publish_state(gyro_y);
  if (this->gyro_z_sensor_ != nullptr)
    this->gyro_z_sensor_->publish_state(gyro_z);
  if (this->temperature_sensor_ != nullptr && !std::isnan(temp_c))
    this->temperature_sensor_->publish_state(temp_c);

  this->status_clear_warning();
}

void BMI270Component::dump_config() {
  ESP_LOGCONFIG(TAG, "BMI270:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
    if (this->setup_status_ == 0xFF) {
      ESP_LOGE(TAG, "  Setup failed before config upload (chip ID mismatch or I2C error)");
    } else {
      ESP_LOGE(TAG, "  INTERNAL_STATUS=0x%02X", this->setup_status_);
    }
  }
  LOG_UPDATE_INTERVAL(this);
  ESP_LOGCONFIG(TAG, "  Power save mode: %s",
                this->power_save_mode_ == POWER_SAVE_MODE_LOW_POWER ? "LOW_POWER" : "NORMAL");
  LOG_SENSOR("  ", "Acceleration X", this->accel_x_sensor_);
  LOG_SENSOR("  ", "Acceleration Y", this->accel_y_sensor_);
  LOG_SENSOR("  ", "Acceleration Z", this->accel_z_sensor_);
  LOG_SENSOR("  ", "Gyroscope X", this->gyro_x_sensor_);
  LOG_SENSOR("  ", "Gyroscope Y", this->gyro_y_sensor_);
  LOG_SENSOR("  ", "Gyroscope Z", this->gyro_z_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
}

float BMI270Component::get_setup_priority() const {
  return setup_priority::DATA;
}

}  // namespace bmi270
}  // namespace esphome

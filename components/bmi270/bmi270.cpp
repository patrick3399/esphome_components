#include "bmi270.h"
#include "esphome/core/log.h"

namespace esphome {
namespace bmi270 {

static const char *const TAG = "bmi270";

void BMI270Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up BMI270...");
  this->i2c_dev_ = make_unique<esphome::i2c::I2CDevice>(this->i2c_bus_, this->address_);
  this->i2c_dev_->set_timeout(50);

  this->sensor_.intf_ptr = this->i2c_dev_.get();
  this->sensor_.intf = BMI2_I2C_INTF;
  this->sensor_.read = read_bytes;
  this->sensor_.write = write_bytes;
  this->sensor_.delay_us = delay_usec;

  int8_t rslt;

  // Initialize the BMI270
  rslt = bmi270_init(&this->sensor_);
  if (rslt != BMI2_OK) {
    ESP_LOGE(TAG, "BMI270 initialization failed: %d", rslt);
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG, "BMI270 initialization succeeded");

  // Enable accelerometer and gyroscope
  uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };
  rslt = bmi270_sensor_enable(sens_list, 2, &this->sensor_);
  if (rslt != BMI2_OK) {
    ESP_LOGE(TAG, "Failed to enable accelerometer and gyroscope: %d", rslt);
    this->mark_failed();
    return;
  }

  // Configure accelerometer
  this->accel_cfg_.odr = BMI2_ACC_ODR_100HZ;
  this->accel_cfg_.range = BMI2_ACC_RANGE_2G;
  this->accel_cfg_.bw = BMI2_ACC_NORMAL_AVG4;
  this->accel_cfg_.perf_mode = BMI2_PERF_OPT_MODE;
  rslt = bmi270_set_sensor_config(&this->accel_cfg_, 1, &this->sensor_);
  if (rslt != BMI2_OK) {
    ESP_LOGE(TAG, "Failed to configure accelerometer: %d", rslt);
    this->mark_failed();
    return;
  }

  // Configure gyroscope
  this->gyro_cfg_.odr = BMI2_GYR_ODR_100HZ;
  this->gyro_cfg_.range = BMI2_GYR_RANGE_2000;
  this->gyro_cfg_.bw = BMI2_GYR_NORMAL_MODE;
  this->gyro_cfg_.noise_perf = BMI2_POWER_OPT_MODE;
  this->gyro_cfg_.filter_perf = BMI2_PERF_OPT_MODE;
  rslt = bmi270_set_sensor_config(&this->gyro_cfg_, 1, &this->sensor_);
  if (rslt != BMI2_OK) {
    ESP_LOGE(TAG, "Failed to configure gyroscope: %d", rslt);
    this->mark_failed();
    return;
  }

  this->is_initialized_ = true;
}

void BMI270Component::update() {
  if (!this->is_initialized_)
    return;

  struct bmi2_sensor_data sensor_data[2] = {};
  sensor_data[0].type = BMI2_ACCEL;
  sensor_data[1].type = BMI2_GYRO;

  int8_t rslt = bmi2_get_sensor_data(sensor_data, 2, &this->sensor_);
  if (rslt != BMI2_OK) {
    ESP_LOGW(TAG, "Failed to read sensor data: %d", rslt);
    return;
  }

  if (this->accel_x_sensor_ != nullptr)
    this->accel_x_sensor_->publish_state(sensor_data[0].sens_data.acc.x / 1000.0f);
  if (this->accel_y_sensor_ != nullptr)
    this->accel_y_sensor_->publish_state(sensor_data[0].sens_data.acc.y / 1000.0f);
  if (this->accel_z_sensor_ != nullptr)
    this->accel_z_sensor_->publish_state(sensor_data[0].sens_data.acc.z / 1000.0f);

  if (this->gyro_x_sensor_ != nullptr)
    this->gyro_x_sensor_->publish_state(sensor_data[1].sens_data.gyr.x / 16.4f);
  if (this->gyro_y_sensor_ != nullptr)
    this->gyro_y_sensor_->publish_state(sensor_data[1].sens_data.gyr.y / 16.4f);
  if (this->gyro_z_sensor_ != nullptr)
    this->gyro_z_sensor_->publish_state(sensor_data[1].sens_data.gyr.z / 16.4f);
}

void BMI270Component::dump_config() {
  ESP_LOGCONFIG(TAG, "BMI270:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "BMI270 communication failed");
  }
}

int8_t BMI270Component::read_bytes(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
  auto *dev = reinterpret_cast<esphome::i2c::I2CDevice *>(intf_ptr);
  return dev->read(reg_addr, data, len) ? BMI2_OK : BMI2_E_COM_FAIL;
}

int8_t BMI270Component::write_bytes(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
  auto *dev = reinterpret_cast<esphome::i2c::I2CDevice *>(intf_ptr);
  return dev->write(reg_addr, data, len) ? BMI2_OK : BMI2_E_COM_FAIL;
}

void BMI270Component::delay_usec(uint32_t period, void *) {
  delayMicroseconds(period);
}

}  // namespace bmi270
}  // namespace esphome

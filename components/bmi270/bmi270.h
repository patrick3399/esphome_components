#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace bmi270 {

// 省電模式枚舉
enum PowerSaveMode {
  POWER_SAVE_MODE_NORMAL = 0,
  POWER_SAVE_MODE_LOW_POWER = 1,
  // POWER_SAVE_MODE_PERFORMANCE = 2, // 可選
};

class BMI270Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override;

  void set_accel_x_sensor(sensor::Sensor *accel_x_sensor) { accel_x_sensor_ = accel_x_sensor; }
  void set_accel_y_sensor(sensor::Sensor *accel_y_sensor) { accel_y_sensor_ = accel_y_sensor; }
  void set_accel_z_sensor(sensor::Sensor *accel_z_sensor) { accel_z_sensor_ = accel_z_sensor; }
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
  void set_gyro_x_sensor(sensor::Sensor *gyro_x_sensor) { gyro_x_sensor_ = gyro_x_sensor; }
  void set_gyro_y_sensor(sensor::Sensor *gyro_y_sensor) { gyro_y_sensor_ = gyro_y_sensor; }
  void set_gyro_z_sensor(sensor::Sensor *gyro_z_sensor) { gyro_z_sensor_ = gyro_z_sensor; }
  
  void set_power_save_mode(PowerSaveMode mode) { power_save_mode_ = mode; } // 新增

 protected:
  bool bmi270_init_config_file();
  void apply_power_save_mode(); // 新增輔助函數

  sensor::Sensor *accel_x_sensor_{nullptr};
  sensor::Sensor *accel_y_sensor_{nullptr};
  sensor::Sensor *accel_z_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *gyro_x_sensor_{nullptr};
  sensor::Sensor *gyro_y_sensor_{nullptr};
  sensor::Sensor *gyro_z_sensor_{nullptr};

  float accel_sensitivity_{0.0f};
  float gyro_sensitivity_{0.0f};
  PowerSaveMode power_save_mode_{POWER_SAVE_MODE_NORMAL}; // 新增，默認為 Normal
  bool sensors_active_{false}; // 新增，追蹤傳感器是否已啟用
};

}  // namespace bmi270
}  // namespace esphome
#include "bmi270.h"
#include "bmi270_config.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cmath>
#include <algorithm>

namespace esphome {
namespace bmi270 {

static const char *const TAG = "bmi270";

// ... (寄存器定義等保持不變) ...
const uint8_t BMI270_REG_CHIP_ID          = 0x00;
const uint8_t BMI270_REG_ERR              = 0x02;
const uint8_t BMI270_REG_STATUS           = 0x03;
const uint8_t BMI270_REG_ACC_X_LSB        = 0x0C;
const uint8_t BMI270_REG_ACC_X_MSB        = 0x0D;
const uint8_t BMI270_REG_ACC_Y_LSB        = 0x0E;
const uint8_t BMI270_REG_ACC_Y_MSB        = 0x0F;
const uint8_t BMI270_REG_ACC_Z_LSB        = 0x10;
const uint8_t BMI270_REG_ACC_Z_MSB        = 0x11;
const uint8_t BMI270_REG_GYR_X_LSB        = 0x12;
const uint8_t BMI270_REG_GYR_X_MSB        = 0x13;
const uint8_t BMI270_REG_GYR_Y_LSB        = 0x14;
const uint8_t BMI270_REG_GYR_Y_MSB        = 0x15;
const uint8_t BMI270_REG_GYR_Z_LSB        = 0x16;
const uint8_t BMI270_REG_GYR_Z_MSB        = 0x17;
const uint8_t BMI270_REG_TEMP_LSB         = 0x22;
const uint8_t BMI270_REG_TEMP_MSB         = 0x23;
const uint8_t BMI270_REG_INTERNAL_STATUS  = 0x21;
const uint8_t BMI270_REG_ACC_CONF         = 0x40;
const uint8_t BMI270_REG_ACC_RANGE        = 0x41;
const uint8_t BMI270_REG_GYR_CONF         = 0x42;
const uint8_t BMI270_REG_GYR_RANGE        = 0x43;
const uint8_t BMI270_REG_INIT_CTRL        = 0x59;
const uint8_t BMI270_REG_INIT_ADDR_0      = 0x5B;
const uint8_t BMI270_REG_INIT_ADDR_1      = 0x5C;
const uint8_t BMI270_REG_INIT_DATA        = 0x5E;
const uint8_t BMI270_REG_PWR_CONF         = 0x7C;
const uint8_t BMI270_REG_PWR_CTRL         = 0x7D;
const uint8_t BMI270_REG_CMD              = 0x7E;

const uint8_t BMI270_CHIP_ID_VAL        = 0x24;
const float GRAVITY_EARTH = 9.80665f;


bool BMI270Component::bmi270_init_config_file() {
    // ... (此函數內容保持不變) ...
    ESP_LOGD(TAG, "開始上傳 BMI270 設定檔 (來自 bmi270_config.h)...");

    if (bmi270_config_file_len == 0 || 
        (bmi270_config_file_len == 1 && bmi270_config_file[0] == 0x00) || 
        bmi270_config_file_len > 8192 ) {
        ESP_LOGE(TAG, "BMI270 設定檔數據缺失、為空、僅為佔位符或長度不正確 (預期 8192 字節, 實際 %u 字節)。請務必在 bmi270_config.h 中填充真實數據！", bmi270_config_file_len);
        return false;
    }
    const unsigned int expected_len = 8192; 
    if (bmi270_config_file_len != expected_len) { 
        ESP_LOGW(TAG, "BMI270 設定檔數據長度 (%u 字節) 與預期長度 (%u 字節) 不符。可能導致初始化失敗。", bmi270_config_file_len, expected_len);
    }

    uint16_t current_addr_words = 0;

    if (!this->write_byte(BMI270_REG_INIT_ADDR_0, 0x00) ||
        !this->write_byte(BMI270_REG_INIT_ADDR_1, 0x00)) {
        ESP_LOGE(TAG, "設定檔上傳：設置 INIT_ADDR 失敗。");
        return false;
    }

    const uint16_t MAX_BURST_WRITE_BYTES = 30;
    unsigned int effective_len = std::min(expected_len, bmi270_config_file_len);

    for (uint16_t bytes_written = 0; bytes_written < effective_len; ) {
        uint16_t chunk_size_bytes = std::min((uint16_t)MAX_BURST_WRITE_BYTES, (uint16_t)(effective_len - bytes_written));
        
        current_addr_words = bytes_written / 2;
        uint8_t init_addr_0_val = current_addr_words & 0x0F; 
        uint8_t init_addr_1_val = (current_addr_words >> 4) & 0xFF;

        if (!this->write_byte(BMI270_REG_INIT_ADDR_0, init_addr_0_val) ||
            !this->write_byte(BMI270_REG_INIT_ADDR_1, init_addr_1_val)) {
            ESP_LOGE(TAG, "設定檔上傳：更新 INIT_ADDR 到 0x%04X (字) 失敗。", current_addr_words);
            return false;
        }
        ESP_LOGV(TAG, "設定檔上傳：INIT_ADDR 設定為 0x%02X (ADDR1) 0x%02X (ADDR0) 對應字地址 0x%04X", init_addr_1_val, init_addr_0_val, current_addr_words);

        if (!this->write_bytes(BMI270_REG_INIT_DATA, &bmi270_config_file[bytes_written], chunk_size_bytes)) {
            ESP_LOGE(TAG, "設定檔上傳：寫入數據塊失敗，偏移量 %u", bytes_written);
            return false;
        }
        ESP_LOGV(TAG, "已寫入 %u 字節的設定檔數據，有效數據中剩餘 %u 字節。", chunk_size_bytes, effective_len - bytes_written - chunk_size_bytes);
        
        bytes_written += chunk_size_bytes;
        delayMicroseconds(500); 
    }
    if (effective_len < expected_len) {
        ESP_LOGE(TAG, "設定檔上傳警告：提供的設定檔數據長度 (%u) 小於預期的 %u 字節。初始化可能不完整。", effective_len, expected_len);
    }

    ESP_LOGD(TAG, "設定檔上傳完成。");
    return true;
}

void BMI270Component::apply_power_save_mode() {
    ESP_LOGD(TAG, "應用省電模式: %d", static_cast<int>(this->power_save_mode_));
    uint8_t pwr_ctrl_val = 0x00; // 默認禁用所有
    uint8_t acc_conf_val = 0xA8; // Normal mode default: ODR 100Hz, perf filter
    uint8_t gyr_conf_val = 0xA9; // Normal mode default: ODR 200Hz, perf filter, no noise perf
    uint8_t pwr_conf_val = 0x02; // Normal mode default: adv_power_save=0, fifo_self_wakeup=1

    bool accel_needed = (this->accel_x_sensor_ != nullptr || this->accel_y_sensor_ != nullptr || this->accel_z_sensor_ != nullptr);
    bool gyro_needed = (this->gyro_x_sensor_ != nullptr || this->gyro_y_sensor_ != nullptr || this->gyro_z_sensor_ != nullptr);
    bool temp_needed = (this->temperature_sensor_ != nullptr);

    this->sensors_active_ = false; // 重置狀態

    if (accel_needed) {
        pwr_ctrl_val |= (1 << 2); // acc_en
    }
    if (gyro_needed) {
        pwr_ctrl_val |= (1 << 1); // gyr_en
    }
    if (temp_needed || gyro_needed) { // 溫度通常隨陀螺儀啟用，或者如果顯式請求
        pwr_ctrl_val |= (1 << 3); // temp_en
    }

    if (pwr_ctrl_val == 0x00 && this->power_save_mode_ != POWER_SAVE_MODE_LOW_POWER) { // 如果沒有傳感器啟用，並且不是明確的低功耗模式，則可以認為是 Suspend
        ESP_LOGD(TAG, "沒有傳感器被請求，進入類似 Suspend 的狀態 (PWR_CTRL=0x00)");
        if (!this->write_byte(BMI270_REG_PWR_CTRL, 0x00)) {
            ESP_LOGE(TAG, "設置 PWR_CTRL 為 0x00 失敗");
            this->status_set_error();
            return;
        }
        // For full suspend as per datasheet, adv_power_save should be 1.
        // PWR_CONF for suspend: 0x03 (adv_power_save=1, fifo_self_wakeup=1)
        // Or 0x01 (adv_power_save=1, fifo_self_wakeup=0)
        if (!this->write_byte(BMI270_REG_PWR_CONF, 0x01)) { // example for suspend
             ESP_LOGE(TAG, "設置 PWR_CONF 為 Suspend 失敗");
        }
        this->sensors_active_ = false;
        return;
    }


    switch (this->power_save_mode_) {
        case POWER_SAVE_MODE_LOW_POWER:
            ESP_LOGD(TAG, "配置為 LOW_POWER 模式");
            pwr_conf_val = 0x01; // adv_power_save = 1, fifo_self_wakeup = 0 (可調整)
            
            // ACC_CONF for Low Power: acc_filter_perf=0
            // ODR 50Hz (0x07), bwp for averaging (e.g., 0x02 for avg 4)
            // 0b00100111 = 0x27
            if (accel_needed) acc_conf_val = 0x27; // ODR 50Hz, avg 4, no perf filter
            
            // GYR_CONF for Low Power: gyr_filter_perf=0
            // ODR 50Hz (0x07), bwp for averaging (e.g., 0x02 for normal which becomes avg in LP)
            // 0b00100111 = 0x27 (Noise perf is irrelevant if filter_perf is 0)
            if (gyro_needed) gyr_conf_val = 0x27; // ODR 50Hz, avg 4, no perf filter
            break;
        
        // case POWER_SAVE_MODE_PERFORMANCE:
        //     ESP_LOGD(TAG, "配置為 PERFORMANCE 模式");
        //     pwr_conf_val = 0x02; // adv_power_save=0, fifo_self_wakeup=1
        //     acc_conf_val = 0xA8; // Normal Accel (100Hz, perf filter)
        //     gyr_conf_val = 0xE9; // Performance Gyro (200Hz, perf filter, noise perf)
        //     break;

        case POWER_SAVE_MODE_NORMAL:
        default:
            ESP_LOGD(TAG, "配置為 NORMAL 模式");
            pwr_conf_val = 0x02; // adv_power_save=0, fifo_self_wakeup=1
            acc_conf_val = 0xA8; // Normal Accel (100Hz, perf filter)
            gyr_conf_val = 0xA9; // Normal Gyro (200Hz, perf filter, no noise perf)
            break;
    }

    // 1. 禁用所有傳感器以更改配置 (如果它們當前已啟用)
    if (this->sensors_active_) {
        if (!this->write_byte(BMI270_REG_PWR_CTRL, 0x00)) {
            ESP_LOGE(TAG, "在模式切換前禁用傳感器失敗");
            this->status_set_error();
            return;
        }
        delay(1); // 短延遲確保命令生效
    }

    // 2. 寫入配置寄存器
    if (accel_needed) {
        if (!this->write_byte(BMI270_REG_ACC_CONF, acc_conf_val)) {
            ESP_LOGE(TAG, "配置 ACC_CONF 失敗 (0x%02X)", acc_conf_val);
            this->status_set_error(); return;
        }
    }
    if (gyro_needed) {
         if (!this->write_byte(BMI270_REG_GYR_CONF, gyr_conf_val)) {
            ESP_LOGE(TAG, "配置 GYR_CONF 失敗 (0x%02X)", gyr_conf_val);
            this->status_set_error(); return;
        }
    }
   
    // 3. 寫入 PWR_CONF
    if (!this->write_byte(BMI270_REG_PWR_CONF, pwr_conf_val)) {
        ESP_LOGE(TAG, "配置 PWR_CONF 失敗 (0x%02X)", pwr_conf_val);
        this->status_set_error(); return;
    }
    delayMicroseconds(450); // 如果從 adv_power_save=1 切換到 0，需要延遲

    // 4. 寫入 PWR_CTRL 以啟用所需的傳感器
    if (!this->write_byte(BMI270_REG_PWR_CTRL, pwr_ctrl_val)) {
        ESP_LOGE(TAG, "啟用傳感器失敗 (PWR_CTRL=0x%02X)", pwr_ctrl_val);
        this->status_set_error(); return;
    }
    this->sensors_active_ = (pwr_ctrl_val != 0x00);
    delay(2); // 給傳感器一點時間來穩定

    ESP_LOGD(TAG, "電源模式應用完成。PWR_CTRL=0x%02X, ACC_CONF=0x%02X, GYR_CONF=0x%02X, PWR_CONF=0x%02X",
             pwr_ctrl_val, acc_conf_val, gyr_conf_val, pwr_conf_val);
}


void BMI270Component::setup() {
  ESP_LOGCONFIG(TAG, "設定 BMI270...");

  uint8_t chip_id;
  if (!this->read_byte(BMI270_REG_CHIP_ID, &chip_id)) {
    ESP_LOGE(TAG, "讀取 CHIP_ID 失敗");
    this->mark_failed();
    return;
  }
  ESP_LOGD(TAG, "CHIP_ID: 0x%02X", chip_id);
  if (chip_id != BMI270_CHIP_ID_VAL) {
    ESP_LOGE(TAG, "無效的 CHIP_ID。預期 0x%02X, 實際 0x%02X", BMI270_CHIP_ID_VAL, chip_id);
    this->mark_failed();
    return;
  }

  ESP_LOGD(TAG, "開始 BMI270 初始化序列...");

  ESP_LOGV(TAG, "禁用高級省電模式...");
  if (!this->write_byte(BMI270_REG_PWR_CONF, 0x00)) { 
    ESP_LOGE(TAG, "禁用高級省電模式失敗。");
    this->mark_failed();
    return;
  }
  delayMicroseconds(450); 

  ESP_LOGV(TAG, "準備加載設定檔 (INIT_CTRL = 0x00)...");
  if (!this->write_byte(BMI270_REG_INIT_CTRL, 0x00)) {
    ESP_LOGE(TAG, "設置 INIT_CTRL 為 0x00 失敗。");
    this->mark_failed();
    return;
  }

  if (!bmi270_init_config_file()) { 
      ESP_LOGE(TAG, "上傳 BMI270 設定檔失敗。");
      this->mark_failed();
      return;
  }

  ESP_LOGV(TAG, "完成加載設定檔 (INIT_CTRL = 0x01)...");
  if (!this->write_byte(BMI270_REG_INIT_CTRL, 0x01)) {
    ESP_LOGE(TAG, "設置 INIT_CTRL 為 0x01 失敗。");
    this->mark_failed();
    return;
  }

  ESP_LOGV(TAG, "等待初始化就緒 (INTERNAL_STATUS.message == 0b0001)...");
  uint8_t internal_status_val = 0; 
  unsigned long start_time = millis();
  bool init_ok = false;
  while (millis() - start_time < 50) { 
    if (this->read_byte(BMI270_REG_INTERNAL_STATUS, &internal_status_val)) {
      if ((internal_status_val & 0x0F) == 0x01) { 
        init_ok = true;
        break;
      }
    }
    delay(2); 
  }

  if (!init_ok) {
    ESP_LOGE(TAG, "BMI270 初始化超時或 INTERNAL_STATUS 不正確 (0x%02X).", internal_status_val);
    this->mark_failed();
    return;
  }
  ESP_LOGD(TAG, "BMI270 初始化成功。");
  
  // 應用 YAML 中配置的電源模式（這會設置 ACC_CONF, GYR_CONF, PWR_CTRL, PWR_CONF）
  apply_power_save_mode();
  if(this->is_failed()) return; // 如果 apply_power_save_mode 失敗

  // 量程和靈敏度設置（這些值應基於 apply_power_save_mode 中實際設置的 CONF 寄存器）
  // 如果 apply_power_save_mode 改變了默認量程，這裡需要重新計算
  // 為簡單起見，我們假設量程保持默認或由 YAML 控制（未來功能）
  uint8_t accel_range_val = 0x02; // 假設 +/-8g，與 Normal mode 匹配
  // TODO: 如果 power_save_mode 更改了量程，需要讀回或從配置中獲取
  // this->read_byte(BMI270_REG_ACC_RANGE, &accel_range_val);
  switch (accel_range_val) {
    case 0x00: this->accel_sensitivity_ = 1.0f / 16384.0f; break; 
    case 0x01: this->accel_sensitivity_ = 1.0f / 8192.0f;  break; 
    case 0x02: this->accel_sensitivity_ = 1.0f / 4096.0f;  break; 
    case 0x03: this->accel_sensitivity_ = 1.0f / 2048.0f;  break; 
    default:   this->accel_sensitivity_ = 1.0f / 4096.0f;
  }

  uint8_t gyro_range_val = 0x00; // 假設 +/-2000dps
  // TODO: 如果 power_save_mode 更改了量程，需要讀回或從配置中獲取
  // this->read_byte(BMI270_REG_GYR_RANGE, &gyro_range_val); 
  // gyro_range_val &= 0x07; // 只取低3位用於 gyr_range
  switch (gyro_range_val) {
    case 0x00: this->gyro_sensitivity_ = 1.0f / 16.384f; break; 
    case 0x01: this->gyro_sensitivity_ = 1.0f / 32.768f; break; 
    case 0x02: this->gyro_sensitivity_ = 1.0f / 65.536f; break; 
    case 0x03: this->gyro_sensitivity_ = 1.0f / 131.072f; break; 
    case 0x04: this->gyro_sensitivity_ = 1.0f / 262.144f; break; 
    default:   this->gyro_sensitivity_ = 1.0f / 16.384f;
  }
  // 寫入 ACC_RANGE 和 GYR_RANGE (如果尚未在 apply_power_save_mode 中處理)
  // 通常這些在 apply_power_save_mode 中與 CONF 寄存器一起設置
  if (!this->write_byte(BMI270_REG_ACC_RANGE, accel_range_val)) { /* ... error handling ... */ }
  if (!this->write_byte(BMI270_REG_GYR_RANGE, gyro_range_val)) { /* ... error handling ... */ }


  ESP_LOGD(TAG, "BMI270 設定完成。");
}

void BMI270Component::dump_config() {
  ESP_LOGCONFIG(TAG, "BMI270:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "與 BMI270 通訊失敗!");
  }
  LOG_UPDATE_INTERVAL(this);
  
  switch(this->power_save_mode_){
    case POWER_SAVE_MODE_NORMAL:
      ESP_LOGCONFIG(TAG, "  省電模式: NORMAL");
      break;
    case POWER_SAVE_MODE_LOW_POWER:
      ESP_LOGCONFIG(TAG, "  省電模式: LOW_POWER");
      break;
    // case POWER_SAVE_MODE_PERFORMANCE:
    //   ESP_LOGCONFIG(TAG, "  Power Save Mode: PERFORMANCE");
    //   break;
    default:
      ESP_LOGCONFIG(TAG, "  省電模式: UNKNOWN");
      break;
  }

  LOG_SENSOR("  ", "加速度 X", this->accel_x_sensor_);
  LOG_SENSOR("  ", "加速度 Y", this->accel_y_sensor_);
  LOG_SENSOR("  ", "加速度 Z", this->accel_z_sensor_);
  LOG_SENSOR("  ", "陀螺儀 X", this->gyro_x_sensor_);
  LOG_SENSOR("  ", "陀螺儀 Y", this->gyro_y_sensor_);
  LOG_SENSOR("  ", "陀螺儀 Z", this->gyro_z_sensor_);
  LOG_SENSOR("  ", "溫度", this->temperature_sensor_);
}

void BMI270Component::update() {
    ESP_LOGV(TAG, "更新 BMI270 數據...");

    // 在每次更新前，確保傳感器處於活動狀態 (如果之前被禁用)
    // 如果 polling interval 很大，並且我們希望在讀取之間進入低功耗，這裡需要更複雜的邏輯
    // 目前的 apply_power_save_mode() 會在 setup() 時設置模式。
    // 如果 polling interval 之後需要重新喚醒/配置，則需要在 update() 的開頭調用 apply_power_save_mode()
    // 或者確保傳感器在 setup() 後持續保持配置。
    // 為了簡單起見，我們假設 setup() 中的配置在輪詢間隔內是持續的。
    // 如果 sensors_active_ 為 false，可能需要重新激活。
    if (!this->sensors_active_) {
        ESP_LOGD(TAG, "傳感器未激活，嘗試重新應用電源模式。");
        apply_power_save_mode(); // 這會啟用 PWR_CTRL
        if(this->is_failed() || !this->sensors_active_){
            ESP_LOGE(TAG, "無法在更新前激活傳感器。");
            this->status_set_error();
            return;
        }
    }


  uint8_t raw_sensor_data[12];
  if (!this->read_bytes(BMI270_REG_ACC_X_LSB, raw_sensor_data, 12)) {
    this->status_set_warning();
    ESP_LOGW(TAG, "讀取加速度/陀螺儀數據失敗");
    return;
  }

  int16_t ax_raw = (int16_t)(((uint16_t)raw_sensor_data[1] << 8) | raw_sensor_data[0]);
  int16_t ay_raw = (int16_t)(((uint16_t)raw_sensor_data[3] << 8) | raw_sensor_data[2]);
  int16_t az_raw = (int16_t)(((uint16_t)raw_sensor_data[5] << 8) | raw_sensor_data[4]);

  int16_t gx_raw = (int16_t)(((uint16_t)raw_sensor_data[7] << 8) | raw_sensor_data[6]);
  int16_t gy_raw = (int16_t)(((uint16_t)raw_sensor_data[9] << 8) | raw_sensor_data[8]);
  int16_t gz_raw = (int16_t)(((uint16_t)raw_sensor_data[11] << 8) | raw_sensor_data[10]);

  uint8_t raw_temp_data[2];
  float temperature = NAN; 
  if (!this->read_bytes(BMI270_REG_TEMP_LSB, raw_temp_data, 2)) {
    this->status_set_warning();
    ESP_LOGW(TAG, "讀取溫度數據失敗");
  } else {
      int16_t temp_raw = (int16_t)(((uint16_t)raw_temp_data[1] << 8) | raw_temp_data[0]);
      if (temp_raw == (int16_t)0x8000) { 
          ESP_LOGW(TAG, "溫度數據無效 (0x8000)");
      } else {
          temperature = ((float)temp_raw / 512.0f) + 23.0f;
      }
  }

  float accel_x = ax_raw * this->accel_sensitivity_ * GRAVITY_EARTH;
  float accel_y = ay_raw * this->accel_sensitivity_ * GRAVITY_EARTH;
  float accel_z = az_raw * this->accel_sensitivity_ * GRAVITY_EARTH;

  float gyro_x = gx_raw * this->gyro_sensitivity_;
  float gyro_y = gy_raw * this->gyro_sensitivity_;
  float gyro_z = gz_raw * this->gyro_sensitivity_;

  ESP_LOGD(TAG,
           "原始數據: Acc(X:%d Y:%d Z:%d) Gyr(X:%d Y:%d Z:%d) Temp:%s",
           ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, std::isnan(temperature) ? "N/A" : esphome::to_string(temperature).c_str());
  ESP_LOGD(TAG,
           "計算值: Accel={x=%.3f m/s², y=%.3f m/s², z=%.3f m/s²}, "
           "Gyro={x=%.3f °/s, y=%.3f °/s, z=%.3f °/s}, Temp=%.2f°C",
           accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temperature);

  if (this->accel_x_sensor_ != nullptr)
    this->accel_x_sensor_->publish_state(accel_x);
  if (this->accel_y_sensor_ != nullptr)
    this->accel_y_sensor_->publish_state(accel_y);
  if (this->accel_z_sensor_ != nullptr)
    this->accel_z_sensor_->publish_state(accel_z);

  if (this->temperature_sensor_ != nullptr && !std::isnan(temperature))
    this->temperature_sensor_->publish_state(temperature);

  if (this->gyro_x_sensor_ != nullptr)
    this->gyro_x_sensor_->publish_state(gyro_x);
  if (this->gyro_y_sensor_ != nullptr)
    this->gyro_y_sensor_->publish_state(gyro_y);
  if (this->gyro_z_sensor_ != nullptr)
    this->gyro_z_sensor_->publish_state(gyro_z);

  this->status_clear_warning();

  // 如果您希望在每次讀取後讓傳感器進入最低功耗狀態，可以在這裡操作
  // 但這會增加下次喚醒的延遲和複雜性。
  // 例如，可以設置 PWR_CTRL 為 0x00，並將 PWR_CONF 設為 adv_power_save=1
  // if (this->power_save_mode_ == POWER_SAVE_MODE_SUSPEND_AFTER_UPDATE) {
  //    this->write_byte(BMI270_REG_PWR_CTRL, 0x00);
  //    this->write_byte(BMI270_REG_PWR_CONF, 0x01); // adv_power_save=1, fifo_self_wakeup=0
  //    this->sensors_active_ = false;
  // }
}

float BMI270Component::get_setup_priority() const { return setup_priority::DATA; }

}  // namespace bmi270
}  // namespace esphome
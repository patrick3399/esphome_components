#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <vector>

#include "esphome/components/camera/camera.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif

namespace esphome::mcu90640 {

// GY-MCU90640: MLX90640 (32x24) behind an onboard STM32 that streams
// pre-computed temperatures over UART at 115200 baud.
static const uint16_t MCU90640_COLS = 32;
static const uint16_t MCU90640_ROWS = 24;
static const uint16_t MCU90640_PIXEL_COUNT = MCU90640_COLS * MCU90640_ROWS;  // 768

// UART frame: 5A 5A <fn> <len> + 768 x int16 LE pixels + Ta (int16 LE) + checksum (uint16 LE)
static const size_t MCU90640_FRAME_SIZE = 1544;
static const size_t MCU90640_PIXEL_OFFSET = 4;
static const size_t MCU90640_TA_OFFSET = 1540;
static const size_t MCU90640_CHECKSUM_OFFSET = 1542;
static const size_t MCU90640_JPEG_BUF_SIZE = 32768;

class MCU90640CameraImage : public camera::CameraImage {
 public:
  MCU90640CameraImage(const uint8_t *data, size_t length, uint8_t requesters);
  ~MCU90640CameraImage() override;
  uint8_t *get_data_buffer() override {
    return this->data_;
  }
  size_t get_data_length() override {
    return this->length_;
  }
  bool valid() const {
    return this->data_ != nullptr;
  }
  bool was_requested_by(camera::CameraRequester requester) const override;

 protected:
  uint8_t *data_{nullptr};
  size_t length_;
  uint8_t requesters_;
};

class MCU90640CameraImageReader : public camera::CameraImageReader {
 public:
  void set_image(std::shared_ptr<camera::CameraImage> image) override;
  size_t available() const override;
  uint8_t *peek_data_buffer() override;
  void consume_data(size_t consumed) override;
  void return_image() override;

 protected:
  std::shared_ptr<MCU90640CameraImage> image_;
  size_t offset_{0};
};

class MCU90640Component : public camera::Camera, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override {
    return setup_priority::DATA;
  }

  void add_listener(camera::CameraListener *listener) override {
    this->listeners_.push_back(listener);
  }
  camera::CameraImageReader *create_image_reader() override;
  void request_image(camera::CameraRequester requester) override;
  void start_stream(camera::CameraRequester requester) override;
  void stop_stream(camera::CameraRequester requester) override;

  void set_update_interval(uint32_t interval) {
    this->update_interval_ = interval;
  }
  void set_idle_update_interval(uint32_t interval) {
    this->idle_update_interval_ = interval;
  }
  void set_jpeg_quality(uint8_t quality) {
    this->jpeg_quality_ = quality;
  }
  void set_output_size(uint16_t width, uint16_t height) {
    this->output_width_ = width;
    this->output_height_ = height;
  }
  void set_rotation(uint16_t rotation) {
    this->rotation_ = rotation;
  }
  void set_mirror_horizontal(bool mirror) {
    this->mirror_horizontal_ = mirror;
  }
  void set_mirror_vertical(bool mirror) {
    this->mirror_vertical_ = mirror;
  }
  void set_emissivity(float emissivity) {
    this->emissivity_ = emissivity;
  }

#ifdef USE_SENSOR
  void set_avg_temperature_sensor(sensor::Sensor *s) {
    this->avg_temp_sensor_ = s;
  }
  void set_min_temperature_sensor(sensor::Sensor *s) {
    this->min_temp_sensor_ = s;
  }
  void set_max_temperature_sensor(sensor::Sensor *s) {
    this->max_temp_sensor_ = s;
  }
  void set_ambient_temperature_sensor(sensor::Sensor *s) {
    this->ambient_temp_sensor_ = s;
  }
  void set_centroid_x_sensor(sensor::Sensor *s) {
    this->centroid_x_sensor_ = s;
  }
  void set_centroid_y_sensor(sensor::Sensor *s) {
    this->centroid_y_sensor_ = s;
  }
#endif

#ifdef USE_BINARY_SENSOR
  void set_presence_sensor(binary_sensor::BinarySensor *s) {
    this->presence_sensor_ = s;
  }
  void set_hot_spot_sensor(binary_sensor::BinarySensor *s) {
    this->hot_spot_sensor_ = s;
  }
  void set_presence_threshold(float t) {
    this->presence_threshold_ = t;
  }
  void set_presence_min_pixels(uint16_t n) {
    this->presence_min_pixels_ = n;
  }
  void set_hot_spot_threshold(float t) {
    this->hot_spot_threshold_ = t;
  }
#endif

 protected:
  // Module commands (0xA5 ...)
  void send_command_(uint8_t cmd, uint8_t value);

  // RX framing
  void feed_byte_(uint8_t byte);
  bool verify_checksum_();
  void decode_frame_();

  // Publishing / rendering
  void publish_frame_();
  void render_rgb_(float min_t, float max_t);
  bool encode_jpeg_(size_t &out_size);
  uint16_t rendered_width_() const;
  uint16_t rendered_height_() const;
  static float bilinear_sample_(const float *grid, int cols, int rows, float x, float y);
  static void iron_color_(float t, uint8_t &r, uint8_t &g, uint8_t &b);

  // Config
  uint32_t update_interval_{1000};
  uint32_t idle_update_interval_{60000};
  uint16_t rotation_{0};
  bool mirror_horizontal_{false};
  bool mirror_vertical_{false};
  uint8_t jpeg_quality_{80};
  uint16_t output_width_{64};
  uint16_t output_height_{48};
  float emissivity_{1.0f};

  // RX state
  size_t rx_pos_{0};
  uint32_t last_frame_time_{0};
  uint32_t last_recovery_attempt_{0};
  uint32_t frame_count_{0};
  uint32_t checksum_fail_count_{0};
  bool checksum_logged_{false};

  // Publish state
  uint32_t last_publish_{0};
  float ta_{NAN};

  std::atomic<uint8_t> single_requesters_{0};
  std::atomic<uint8_t> stream_requesters_{0};
  std::shared_ptr<MCU90640CameraImage> current_image_;
  std::vector<camera::CameraListener *> listeners_;

  // Heap buffers (allocated once in setup())
  uint8_t *rx_buf_{nullptr};  // MCU90640_FRAME_SIZE raw UART frame
  float *pixels_{nullptr};  // MCU90640_PIXEL_COUNT floats, °C
  uint8_t *rgb_buf_{nullptr};
  uint8_t *jpeg_buf_{nullptr};
  size_t rgb_buf_size_{0};

#ifdef USE_SENSOR
  sensor::Sensor *avg_temp_sensor_{nullptr};
  sensor::Sensor *min_temp_sensor_{nullptr};
  sensor::Sensor *max_temp_sensor_{nullptr};
  sensor::Sensor *ambient_temp_sensor_{nullptr};
  sensor::Sensor *centroid_x_sensor_{nullptr};
  sensor::Sensor *centroid_y_sensor_{nullptr};
#endif

#ifdef USE_BINARY_SENSOR
  binary_sensor::BinarySensor *presence_sensor_{nullptr};
  binary_sensor::BinarySensor *hot_spot_sensor_{nullptr};
  float presence_threshold_{3.0f};
  float hot_spot_threshold_{50.0f};
  uint16_t presence_min_pixels_{5};
  std::array<float, MCU90640_PIXEL_COUNT> background_{};
  bool background_initialized_{false};
#endif
};

}  // namespace esphome::mcu90640

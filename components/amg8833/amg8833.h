#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <vector>

#include "esphome/components/camera/camera.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif

#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif

namespace esphome {
namespace amg8833 {

static const uint16_t AMG8833_PIXEL_COUNT = 64;
static const size_t AMG8833_JPEG_BUF_SIZE = 16384;

class AMG8833CameraImage : public camera::CameraImage {
 public:
  AMG8833CameraImage(const uint8_t *data, size_t length, uint8_t requesters);
  ~AMG8833CameraImage() override;
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

class AMG8833CameraImageReader : public camera::CameraImageReader {
 public:
  void set_image(std::shared_ptr<camera::CameraImage> image) override;
  size_t available() const override;
  uint8_t *peek_data_buffer() override;
  void consume_data(size_t consumed) override;
  void return_image() override;

 protected:
  std::shared_ptr<AMG8833CameraImage> image_;
  size_t offset_{0};
};

class AMG8833Component : public camera::Camera, public i2c::I2CDevice {
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
  void set_thermistor_sensor(sensor::Sensor *s) {
    this->thermistor_sensor_ = s;
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
  void set_presence_min_pixels(uint8_t n) {
    this->presence_min_pixels_ = n;
  }
  void set_hot_spot_threshold(float t) {
    this->hot_spot_threshold_ = t;
  }
#endif

 protected:
  void finish_setup_();
  void read_and_publish_();
  bool read_pixels_(float pixels[AMG8833_PIXEL_COUNT]);
  bool read_thermistor_(float &temp);
  bool encode_jpeg_(size_t &out_size);
  void render_rgb_(const float pixels[AMG8833_PIXEL_COUNT], float min_t, float max_t);
  static float bilinear_sample_(const float *grid, float x, float y);
  static void iron_color_(float t, uint8_t &r, uint8_t &g, uint8_t &b);

  uint32_t update_interval_{1000};
  uint32_t idle_update_interval_{60000};
  uint16_t rotation_{0};
  uint32_t last_update_{0};
  uint8_t jpeg_quality_{80};
  uint16_t output_width_{64};
  uint16_t output_height_{64};

  std::atomic<uint8_t> single_requesters_{0};
  std::atomic<uint8_t> stream_requesters_{0};
  std::shared_ptr<AMG8833CameraImage> current_image_;
  std::vector<camera::CameraListener *> listeners_;

  uint8_t *rgb_buf_{nullptr};
  uint8_t *jpeg_buf_{nullptr};
  size_t rgb_buf_size_{0};
  bool setup_complete_{false};

#ifdef USE_SENSOR
  sensor::Sensor *avg_temp_sensor_{nullptr};
  sensor::Sensor *min_temp_sensor_{nullptr};
  sensor::Sensor *max_temp_sensor_{nullptr};
  sensor::Sensor *thermistor_sensor_{nullptr};
  sensor::Sensor *centroid_x_sensor_{nullptr};
  sensor::Sensor *centroid_y_sensor_{nullptr};
#endif

#ifdef USE_BINARY_SENSOR
  binary_sensor::BinarySensor *presence_sensor_{nullptr};
  binary_sensor::BinarySensor *hot_spot_sensor_{nullptr};
  float presence_threshold_{3.0f};
  float hot_spot_threshold_{50.0f};
  uint8_t presence_min_pixels_{2};
  std::array<float, AMG8833_PIXEL_COUNT> background_{};
  bool background_initialized_{false};
#endif
};

}  // namespace amg8833
}  // namespace esphome

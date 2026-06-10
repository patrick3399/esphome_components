#include "amg8833.h"

#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32
#include <esp_camera.h>
#endif

namespace esphome {
namespace amg8833 {

static const char *const TAG = "amg8833";

// AMG8833 register addresses
static const uint8_t REG_PCTL = 0x00;
static const uint8_t REG_RST = 0x01;
static const uint8_t REG_FPSC = 0x02;
static const uint8_t REG_TTHL = 0x0E;
static const uint8_t REG_T01L = 0x80;

// AMG8833 register values
static const uint8_t PCTL_NORMAL_MODE = 0x00;
static const uint8_t RST_INITIAL = 0x3F;
static const uint8_t FPSC_10FPS = 0x00;

// Iron palette control points: {position, R, G, B}
static const float IRON_STOPS[] = {0.0f, 0.2f, 0.4f, 0.6f, 0.8f, 1.0f};
static const uint8_t IRON_COLORS[][3] = {
    {0, 0, 0},  // black (cold)
    {0, 0, 200},  // dark blue
    {180, 0, 140},  // purple-magenta
    {255, 80, 0},  // orange-red
    {255, 220, 0},  // yellow
    {255, 255, 255},  // white (hot)
};
static const int IRON_STOP_COUNT = 6;

/* -------------------- JPEG write callback -------------------- */

#ifdef USE_ESP32
struct JpegWriteCtx {
  uint8_t *buf;
  size_t capacity;
  size_t written;
};

static size_t jpeg_write_cb(void *arg, size_t index, const void *data, size_t len) {
  auto *ctx = static_cast<JpegWriteCtx *>(arg);
  if (index + len > ctx->capacity) {
    return 0;
  }
  memcpy(ctx->buf + index, data, len);
  size_t end = index + len;
  if (end > ctx->written) {
    ctx->written = end;
  }
  return len;
}
#endif

/* -------------------- AMG8833CameraImage -------------------- */

AMG8833CameraImage::AMG8833CameraImage(const uint8_t *data, size_t length, uint8_t requesters)
    : length_(length), requesters_(requesters) {
  // Per-frame heap allocation is unavoidable here: multiple API connections may
  // hold shared_ptr references to different images simultaneously, so a single
  // static buffer cannot be used.
  this->data_ = new uint8_t[length];
  if (this->data_ != nullptr) {
    memcpy(this->data_, data, length);
  }
}

AMG8833CameraImage::~AMG8833CameraImage() {
  delete[] this->data_;
}

bool AMG8833CameraImage::was_requested_by(camera::CameraRequester requester) const {
  return (this->requesters_ & static_cast<uint8_t>(requester)) != 0;
}

/* -------------------- AMG8833CameraImageReader -------------------- */

void AMG8833CameraImageReader::set_image(std::shared_ptr<camera::CameraImage> image) {
  this->image_ = std::static_pointer_cast<AMG8833CameraImage>(image);
  this->offset_ = 0;
}

size_t AMG8833CameraImageReader::available() const {
  if (!this->image_) {
    return 0;
  }
  return this->image_->get_data_length() - this->offset_;
}

uint8_t *AMG8833CameraImageReader::peek_data_buffer() {
  if (!this->image_) {
    return nullptr;
  }
  return this->image_->get_data_buffer() + this->offset_;
}

void AMG8833CameraImageReader::consume_data(size_t consumed) {
  this->offset_ += consumed;
}

void AMG8833CameraImageReader::return_image() {
  this->image_.reset();
  this->offset_ = 0;
}

/* -------------------- AMG8833Component -------------------- */

void AMG8833Component::setup() {
  if (!this->write_byte(REG_PCTL, PCTL_NORMAL_MODE)) {
    ESP_LOGE(TAG, "Failed to set normal mode");
    this->mark_failed();
    return;
  }
  if (!this->write_byte(REG_RST, RST_INITIAL)) {
    ESP_LOGE(TAG, "Failed to reset device");
    this->mark_failed();
    return;
  }
  // AMG8833 datasheet: wait >= 2 ms after reset; 50 ms is safe
  delay(50);
  if (!this->write_byte(REG_FPSC, FPSC_10FPS)) {
    ESP_LOGE(TAG, "Failed to set frame rate");
    this->mark_failed();
    return;
  }

  this->rgb_buf_size_ = this->output_width_ * this->output_height_ * 3;
  this->rgb_buf_ = new uint8_t[this->rgb_buf_size_];
  this->jpeg_buf_ = new uint8_t[AMG8833_JPEG_BUF_SIZE];
  if (this->rgb_buf_ == nullptr || this->jpeg_buf_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate image buffers");
    this->mark_failed();
    return;
  }

  this->last_update_ = millis();
  ESP_LOGD(TAG, "AMG8833 setup complete, output %ux%u", this->output_width_, this->output_height_);
}

void AMG8833Component::loop() {
  uint32_t now = millis();
  bool has_requesters = (this->single_requesters_ != 0) || (this->stream_requesters_ != 0);
  // Sensor and binary_sensor consumers must poll at update_interval regardless of camera
  // activity; presence detection breaks if throttled to the 60s idle rate. The idle interval
  // is only used when the component is in pure camera-only mode with no active requesters.
  bool has_consumers = false;
#ifdef USE_SENSOR
  has_consumers = this->avg_temp_sensor_ || this->min_temp_sensor_ || this->max_temp_sensor_ ||
                  this->thermistor_sensor_ || this->centroid_x_sensor_ || this->centroid_y_sensor_;
#endif
#ifdef USE_BINARY_SENSOR
  has_consumers = has_consumers || this->presence_sensor_ || this->hot_spot_sensor_;
#endif
  uint32_t interval = (has_consumers || has_requesters) ? this->update_interval_ : this->idle_update_interval_;
  if (now - this->last_update_ < interval) {
    return;
  }
  this->last_update_ = now;
  this->read_and_publish_();
}

void AMG8833Component::dump_config() {
  ESP_LOGCONFIG(TAG, "AMG8833:");
  LOG_I2C_DEVICE(this);
  ESP_LOGCONFIG(TAG, "  Output: %ux%u, JPEG quality: %u, Rotation: %u°", this->output_width_, this->output_height_,
                this->jpeg_quality_, this->rotation_);
  ESP_LOGCONFIG(TAG, "  Update interval: %u ms, Idle interval: %u ms", this->update_interval_,
                this->idle_update_interval_);
#ifdef USE_SENSOR
  LOG_SENSOR("  ", "Avg Temperature", this->avg_temp_sensor_);
  LOG_SENSOR("  ", "Min Temperature", this->min_temp_sensor_);
  LOG_SENSOR("  ", "Max Temperature", this->max_temp_sensor_);
  LOG_SENSOR("  ", "Thermistor", this->thermistor_sensor_);
  LOG_SENSOR("  ", "Centroid X", this->centroid_x_sensor_);
  LOG_SENSOR("  ", "Centroid Y", this->centroid_y_sensor_);
#endif
#ifdef USE_BINARY_SENSOR
  LOG_BINARY_SENSOR("  ", "Presence", this->presence_sensor_);
  if (this->presence_sensor_ != nullptr) {
    ESP_LOGCONFIG(TAG, "    Threshold: %.1f°C, Min pixels: %u", this->presence_threshold_, this->presence_min_pixels_);
  }
  LOG_BINARY_SENSOR("  ", "Hot Spot", this->hot_spot_sensor_);
  if (this->hot_spot_sensor_ != nullptr) {
    ESP_LOGCONFIG(TAG, "    Threshold: %.1f°C", this->hot_spot_threshold_);
  }
#endif
}

camera::CameraImageReader *AMG8833Component::create_image_reader() {
  return new AMG8833CameraImageReader();
}

void AMG8833Component::request_image(camera::CameraRequester requester) {
  this->single_requesters_ |= static_cast<uint8_t>(requester);
}

void AMG8833Component::start_stream(camera::CameraRequester requester) {
  this->stream_requesters_ |= static_cast<uint8_t>(requester);
  for (auto *listener : this->listeners_) {
    listener->on_stream_start();
  }
}

void AMG8833Component::stop_stream(camera::CameraRequester requester) {
  this->stream_requesters_ &= ~static_cast<uint8_t>(requester);
  if (this->stream_requesters_ == 0) {
    for (auto *listener : this->listeners_) {
      listener->on_stream_stop();
    }
  }
}

/* -------------------- Internal -------------------- */

void AMG8833Component::read_and_publish_() {
  float pixels[AMG8833_PIXEL_COUNT];
  if (!this->read_pixels_(pixels)) {
    return;
  }

  // Always compute min/max/avg — cheap (64 iterations) and shared by sensors,
  // presence detection, centroid, and camera auto-range.
  float sum = 0.0f;
  float min_t = pixels[0];
  float max_t = pixels[0];
  for (uint16_t i = 0; i < AMG8833_PIXEL_COUNT; i++) {
    sum += pixels[i];
    if (pixels[i] < min_t) {
      min_t = pixels[i];
    }
    if (pixels[i] > max_t) {
      max_t = pixels[i];
    }
  }
  float avg_t = sum / static_cast<float>(AMG8833_PIXEL_COUNT);

#ifdef USE_SENSOR
  if (this->avg_temp_sensor_ != nullptr) {
    this->avg_temp_sensor_->publish_state(avg_t);
  }
  if (this->min_temp_sensor_ != nullptr) {
    this->min_temp_sensor_->publish_state(min_t);
  }
  if (this->max_temp_sensor_ != nullptr) {
    this->max_temp_sensor_->publish_state(max_t);
  }
  if (this->thermistor_sensor_ != nullptr) {
    float therm_t;
    if (this->read_thermistor_(therm_t)) {
      this->thermistor_sensor_->publish_state(therm_t);
    }
  }

  // Thermal centroid: weighted average position using (temp - min) as weight.
  // Gives the "center of mass" of the hot region (range 0.0–7.0 in pixel units).
  if (this->centroid_x_sensor_ != nullptr || this->centroid_y_sensor_ != nullptr) {
    float total_weight = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    for (int row = 0; row < 8; row++) {
      for (int col = 0; col < 8; col++) {
        float w = pixels[row * 8 + col] - min_t;
        total_weight += w;
        cx += w * static_cast<float>(col);
        cy += w * static_cast<float>(row);
      }
    }
    if (total_weight > 0.0f) {
      if (this->centroid_x_sensor_ != nullptr) {
        this->centroid_x_sensor_->publish_state(cx / total_weight);
      }
      if (this->centroid_y_sensor_ != nullptr) {
        this->centroid_y_sensor_->publish_state(cy / total_weight);
      }
    }
  }
#endif

#ifdef USE_BINARY_SENSOR
  // Adaptive background EMA presence detection.
  // Background tracks the empty-room baseline; a human body creates a local
  // temperature rise well above that baseline even in warm ambient conditions.
  if (this->presence_sensor_ != nullptr) {
    if (!this->background_initialized_) {
      for (uint16_t i = 0; i < AMG8833_PIXEL_COUNT; i++) {
        this->background_[i] = pixels[i];
      }
      this->background_initialized_ = true;
    } else {
      uint8_t above = 0;
      for (uint16_t i = 0; i < AMG8833_PIXEL_COUNT; i++) {
        if (pixels[i] - this->background_[i] > this->presence_threshold_) {
          above++;
        }
      }
      bool presence = above >= this->presence_min_pixels_;
      // Slow alpha when occupied prevents the background from drifting toward
      // body temperature; fast alpha when empty recovers quickly from transients.
      float alpha = presence ? 0.01f : 0.1f;
      for (uint16_t i = 0; i < AMG8833_PIXEL_COUNT; i++) {
        this->background_[i] += alpha * (pixels[i] - this->background_[i]);
      }
      this->presence_sensor_->publish_state(presence);
    }
  }

  if (this->hot_spot_sensor_ != nullptr) {
    bool hot = false;
    for (uint16_t i = 0; i < AMG8833_PIXEL_COUNT; i++) {
      if (pixels[i] >= this->hot_spot_threshold_) {
        hot = true;
        break;
      }
    }
    this->hot_spot_sensor_->publish_state(hot);
  }
#endif

  // Camera JPEG — only encode when there are active requesters to avoid
  // unnecessary CPU load at the idle update rate.
  uint8_t single = this->single_requesters_.load();
  uint8_t stream = this->stream_requesters_.load();
  if (single == 0 && stream == 0) {
    return;
  }

  this->render_rgb_(pixels, min_t, max_t);

  size_t jpeg_len = 0;
  if (!this->encode_jpeg_(jpeg_len)) {
    return;
  }

  uint8_t requesters = single | stream;
  auto image = std::make_shared<AMG8833CameraImage>(this->jpeg_buf_, jpeg_len, requesters);

  // Reset single-shot requesters; stream requesters remain until stop_stream()
  this->single_requesters_ = 0;

  this->current_image_ = image;
  for (auto *listener : this->listeners_) {
    listener->on_camera_image(image);
  }
}

bool AMG8833Component::read_pixels_(float pixels[AMG8833_PIXEL_COUNT]) {
  uint8_t buf[128];
  if (!this->read_bytes(REG_T01L, buf, 128)) {
    ESP_LOGE(TAG, "I2C read of pixel registers failed");
    return false;
  }
  for (int i = 0; i < 64; i++) {
    uint8_t lo = buf[2 * i];
    uint8_t hi = buf[2 * i + 1];
    // 12-bit two's complement, sign in bit 11
    int16_t raw = (static_cast<int16_t>(hi & 0x0F) << 8) | lo;
    if (raw & 0x0800) {
      raw |= 0xF000;
    }
    pixels[i] = raw * 0.25f;
  }
  return true;
}

bool AMG8833Component::read_thermistor_(float &temp) {
  uint8_t buf[2];
  if (!this->read_bytes(REG_TTHL, buf, 2)) {
    ESP_LOGE(TAG, "I2C read of thermistor register failed");
    return false;
  }
  // 12-bit two's complement, 0.0625 °C/LSB
  int16_t raw = (static_cast<int16_t>(buf[1] & 0x0F) << 8) | buf[0];
  if (raw & 0x0800) {
    raw |= 0xF000;
  }
  temp = raw * 0.0625f;
  return true;
}

void AMG8833Component::render_rgb_(const float pixels[AMG8833_PIXEL_COUNT], float min_t, float max_t) {
  float range = max_t - min_t;
  // Guard against flat scenes (all pixels at identical temperature)
  if (range < 0.1f) {
    range = 0.1f;
  }

  uint8_t *dst = this->rgb_buf_;
  float w = static_cast<float>(this->output_width_);
  float h = static_cast<float>(this->output_height_);

  for (int py = 0; py < this->output_height_; py++) {
    for (int px = 0; px < this->output_width_; px++) {
      // Map output pixel coords to 8×8 source grid (0..7 range)
      float src_x = (w > 1.0f) ? (px * 7.0f / (w - 1.0f)) : 0.0f;
      float src_y = (h > 1.0f) ? (py * 7.0f / (h - 1.0f)) : 0.0f;
      switch (this->rotation_) {
        case 90: {
          float tmp = src_x;
          src_x = src_y;
          src_y = 7.0f - tmp;
          break;
        }
        case 180:
          src_x = 7.0f - src_x;
          src_y = 7.0f - src_y;
          break;
        case 270: {
          float tmp = src_x;
          src_x = 7.0f - src_y;
          src_y = tmp;
          break;
        }
        default:
          break;
      }
      float temp = bilinear_sample_(pixels, src_x, src_y);
      float t = (temp - min_t) / range;
      if (t < 0.0f) {
        t = 0.0f;
      }
      if (t > 1.0f) {
        t = 1.0f;
      }
      uint8_t r, g, b;
      iron_color_(t, r, g, b);
      // fmt2jpg_cb with PIXFORMAT_RGB888 expects bytes in B, G, R order
      *dst++ = b;
      *dst++ = g;
      *dst++ = r;
    }
  }
}

float AMG8833Component::bilinear_sample_(const float *grid, float x, float y) {
  int x0 = static_cast<int>(x);
  int y0 = static_cast<int>(y);
  int x1 = x0 + 1;
  int y1 = y0 + 1;
  if (x1 > 7) {
    x1 = 7;
  }
  if (y1 > 7) {
    y1 = 7;
  }
  float fx = x - x0;
  float fy = y - y0;
  float v00 = grid[y0 * 8 + x0];
  float v10 = grid[y0 * 8 + x1];
  float v01 = grid[y1 * 8 + x0];
  float v11 = grid[y1 * 8 + x1];
  return v00 * (1.0f - fx) * (1.0f - fy) + v10 * fx * (1.0f - fy) + v01 * (1.0f - fx) * fy + v11 * fx * fy;
}

void AMG8833Component::iron_color_(float t, uint8_t &r, uint8_t &g, uint8_t &b) {
  int i = 0;
  while (i < IRON_STOP_COUNT - 2 && t >= IRON_STOPS[i + 1]) {
    i++;
  }
  float range = IRON_STOPS[i + 1] - IRON_STOPS[i];
  float lt = (range > 0.0f) ? ((t - IRON_STOPS[i]) / range) : 0.0f;
  if (lt < 0.0f) {
    lt = 0.0f;
  }
  if (lt > 1.0f) {
    lt = 1.0f;
  }
  r = static_cast<uint8_t>(IRON_COLORS[i][0] + lt * static_cast<float>(IRON_COLORS[i + 1][0] - IRON_COLORS[i][0]));
  g = static_cast<uint8_t>(IRON_COLORS[i][1] + lt * static_cast<float>(IRON_COLORS[i + 1][1] - IRON_COLORS[i][1]));
  b = static_cast<uint8_t>(IRON_COLORS[i][2] + lt * static_cast<float>(IRON_COLORS[i + 1][2] - IRON_COLORS[i][2]));
}

bool AMG8833Component::encode_jpeg_(size_t &out_size) {
#ifdef USE_ESP32
  JpegWriteCtx ctx{this->jpeg_buf_, AMG8833_JPEG_BUF_SIZE, 0};
  bool ok = fmt2jpg_cb(this->rgb_buf_, this->rgb_buf_size_, this->output_width_, this->output_height_, PIXFORMAT_RGB888,
                       this->jpeg_quality_, jpeg_write_cb, &ctx);
  if (!ok || ctx.written == 0) {
    ESP_LOGE(TAG, "JPEG encoding failed");
    return false;
  }
  out_size = ctx.written;
  return true;
#else
  ESP_LOGE(TAG, "JPEG encoding requires ESP32");
  return false;
#endif
}

}  // namespace amg8833
}  // namespace esphome

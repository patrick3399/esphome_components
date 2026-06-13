#include "mcu90640.h"

#include <cmath>
#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32
#include <esp_camera.h>
#endif

namespace esphome {
namespace mcu90640 {

static const char *const TAG = "mcu90640";

// GY-MCU90640 command set: 0xA5 <cmd> <value> <checksum=sum&0xFF>
static const uint8_t CMD_HEADER = 0xA5;
static const uint8_t CMD_BAUD = 0x15;
static const uint8_t CMD_REFRESH_RATE = 0x25;  // value 0x01 = 4 Hz (community-verified)
static const uint8_t CMD_OUTPUT_MODE = 0x35;  // 0x01 = query (stop), 0x02 = automatic streaming
static const uint8_t CMD_SET_EMISSIVITY = 0x45;  // value = emissivity * 100
static const uint8_t CMD_SAVE_SETTINGS = 0x65;

static const uint8_t FRAME_SYNC = 0x5A;

// Iron palette (cold→hot): black → blue → magenta → orange → yellow → white
static const float IRON_STOPS[] = {0.0f, 0.2f, 0.4f, 0.6f, 0.8f, 1.0f};
static const uint8_t IRON_COLORS[][3] = {
    {0, 0, 0}, {0, 0, 200}, {180, 0, 140}, {255, 80, 0}, {255, 220, 0}, {255, 255, 255},
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

/* -------------------- MCU90640CameraImage -------------------- */

MCU90640CameraImage::MCU90640CameraImage(const uint8_t *data, size_t length, uint8_t requesters)
    : length_(length), requesters_(requesters) {
  // Per-frame heap allocation is unavoidable: concurrent API connections may hold
  // shared_ptr references to different images simultaneously.
  RAMAllocator<uint8_t> allocator;
  this->data_ = allocator.allocate(length);
  if (this->data_ != nullptr) {
    memcpy(this->data_, data, length);
  } else {
    this->length_ = 0;
  }
}

MCU90640CameraImage::~MCU90640CameraImage() {
  if (this->data_ != nullptr) {
    RAMAllocator<uint8_t> allocator;
    allocator.deallocate(this->data_, this->length_);
  }
}

bool MCU90640CameraImage::was_requested_by(camera::CameraRequester requester) const {
  return (this->requesters_ & static_cast<uint8_t>(requester)) != 0;
}

/* -------------------- MCU90640CameraImageReader -------------------- */

void MCU90640CameraImageReader::set_image(std::shared_ptr<camera::CameraImage> image) {
  this->image_ = std::static_pointer_cast<MCU90640CameraImage>(image);
  this->offset_ = 0;
}

size_t MCU90640CameraImageReader::available() const {
  if (!this->image_) {
    return 0;
  }
  return this->image_->get_data_length() - this->offset_;
}

uint8_t *MCU90640CameraImageReader::peek_data_buffer() {
  if (!this->image_) {
    return nullptr;
  }
  return this->image_->get_data_buffer() + this->offset_;
}

void MCU90640CameraImageReader::consume_data(size_t consumed) {
  this->offset_ += consumed;
}

void MCU90640CameraImageReader::return_image() {
  this->image_.reset();
  this->offset_ = 0;
}

/* -------------------- Setup / Loop -------------------- */

void MCU90640Component::setup() {
  this->rx_buf_ = new uint8_t[MCU90640_FRAME_SIZE];
  this->pixels_ = new float[MCU90640_PIXEL_COUNT];
  this->rgb_buf_size_ = this->rendered_width_() * this->rendered_height_() * 3;
  this->rgb_buf_ = new uint8_t[this->rgb_buf_size_];
  this->jpeg_buf_ = new uint8_t[MCU90640_JPEG_BUF_SIZE];

  if (!this->rx_buf_ || !this->pixels_ || !this->rgb_buf_ || !this->jpeg_buf_) {
    ESP_LOGE(TAG, "Failed to allocate buffers");
    this->mark_failed();
    return;
  }
  memset(this->pixels_, 0, MCU90640_PIXEL_COUNT * sizeof(float));

  // Configure the module: emissivity (optional), 4 Hz refresh, automatic streaming.
  if (this->emissivity_ < 1.0f) {
    this->send_command_(CMD_SET_EMISSIVITY, static_cast<uint8_t>(this->emissivity_ * 100.0f + 0.5f));
  }
  this->send_command_(CMD_REFRESH_RATE, 0x01);  // 4 Hz
  this->send_command_(CMD_OUTPUT_MODE, 0x02);  // automatic streaming
}

void MCU90640Component::send_command_(uint8_t cmd, uint8_t value) {
  uint8_t frame[4] = {CMD_HEADER, cmd, value, 0};
  frame[3] = static_cast<uint8_t>(frame[0] + frame[1] + frame[2]);
  this->write_array(frame, sizeof(frame));
}

void MCU90640Component::loop() {
  // Drain RX; cap per-loop work to stay under the loop time budget.
  // At 4 Hz x 1544 bytes = ~6.2 kB/s; a 256-byte cap drains the FIFO easily.
  int budget = 256;
  uint8_t byte;
  while (budget-- > 0 && this->available() > 0) {
    if (!this->read_byte(&byte)) {
      break;
    }
    this->feed_byte_(byte);
  }
}

void MCU90640Component::feed_byte_(uint8_t byte) {
  // Sync on 5A 5A, then accumulate a full frame.
  if (this->rx_pos_ == 0 && byte != FRAME_SYNC) {
    return;
  }
  if (this->rx_pos_ == 1 && byte != FRAME_SYNC) {
    this->rx_pos_ = 0;
    return;
  }
  this->rx_buf_[this->rx_pos_++] = byte;
  if (this->rx_pos_ < MCU90640_FRAME_SIZE) {
    return;
  }
  this->rx_pos_ = 0;

  if (!this->verify_checksum_()) {
    this->checksum_fail_count_++;
    if ((this->checksum_fail_count_ & 0x0F) == 1) {
      ESP_LOGW(TAG, "Frame checksum mismatch (%u total)", this->checksum_fail_count_);
    }
    return;
  }
  this->decode_frame_();
}

bool MCU90640Component::verify_checksum_() {
  uint16_t expected = static_cast<uint16_t>(this->rx_buf_[MCU90640_CHECKSUM_OFFSET]) |
                      (static_cast<uint16_t>(this->rx_buf_[MCU90640_CHECKSUM_OFFSET + 1]) << 8);
  // Byte-sum over everything before the checksum field.
  uint32_t byte_sum = 0;
  for (size_t i = 0; i < MCU90640_CHECKSUM_OFFSET; i++) {
    byte_sum += this->rx_buf_[i];
  }
  if (static_cast<uint16_t>(byte_sum) == expected) {
    if (!this->checksum_logged_) {
      ESP_LOGD(TAG, "Checksum verified (byte-sum)");
      this->checksum_logged_ = true;
    }
    return true;
  }
  // Some firmware revisions reportedly sum 16-bit LE words instead.
  uint32_t word_sum = 0;
  for (size_t i = 0; i < MCU90640_CHECKSUM_OFFSET; i += 2) {
    word_sum += static_cast<uint16_t>(this->rx_buf_[i]) | (static_cast<uint16_t>(this->rx_buf_[i + 1]) << 8);
  }
  if (static_cast<uint16_t>(word_sum) == expected) {
    if (!this->checksum_logged_) {
      ESP_LOGD(TAG, "Checksum verified (word-sum)");
      this->checksum_logged_ = true;
    }
    return true;
  }
  return false;
}

void MCU90640Component::decode_frame_() {
  // Mirroring is applied at decode time so the camera image, centroid and
  // presence detection all share the same (corrected) coordinate space.
  const uint8_t *p = this->rx_buf_ + MCU90640_PIXEL_OFFSET;
  for (int i = 0; i < MCU90640_PIXEL_COUNT; i++) {
    int16_t raw = static_cast<int16_t>(static_cast<uint16_t>(p[i * 2]) | (static_cast<uint16_t>(p[i * 2 + 1]) << 8));
    int row = i / MCU90640_COLS;
    int col = i % MCU90640_COLS;
    if (this->mirror_horizontal_) {
      col = (MCU90640_COLS - 1) - col;
    }
    if (this->mirror_vertical_) {
      row = (MCU90640_ROWS - 1) - row;
    }
    this->pixels_[row * MCU90640_COLS + col] = raw / 100.0f;
  }
  int16_t ta_raw = static_cast<int16_t>(static_cast<uint16_t>(this->rx_buf_[MCU90640_TA_OFFSET]) |
                                        (static_cast<uint16_t>(this->rx_buf_[MCU90640_TA_OFFSET + 1]) << 8));
  this->ta_ = ta_raw / 100.0f;

  this->frame_count_++;
  this->last_frame_time_ = millis();
  if (this->frame_count_ == 1) {
    ESP_LOGI(TAG, "First frame received (Ta=%.2f °C)", this->ta_);
  }

  // Publish throttling: full rate while anything consumes data, idle rate otherwise.
  bool has_consumers = false;
#ifdef USE_SENSOR
  has_consumers = this->avg_temp_sensor_ || this->min_temp_sensor_ || this->max_temp_sensor_ ||
                  this->ambient_temp_sensor_ || this->centroid_x_sensor_ || this->centroid_y_sensor_;
#endif
#ifdef USE_BINARY_SENSOR
  has_consumers = has_consumers || this->presence_sensor_ || this->hot_spot_sensor_;
#endif
  bool has_requesters = (this->single_requesters_ != 0) || (this->stream_requesters_ != 0);
  uint32_t interval = (has_consumers || has_requesters) ? this->update_interval_ : this->idle_update_interval_;

  uint32_t now = millis();
  if (now - this->last_publish_ < interval) {
    return;
  }
  this->last_publish_ = now;
  this->publish_frame_();
}

void MCU90640Component::dump_config() {
  ESP_LOGCONFIG(TAG, "GY-MCU90640:");
  ESP_LOGCONFIG(TAG, "  Native resolution: %ux%u", MCU90640_COLS, MCU90640_ROWS);
  ESP_LOGCONFIG(TAG, "  Output: %ux%u, JPEG quality: %u, Rotation: %u°", this->rendered_width_(),
                this->rendered_height_(), this->jpeg_quality_, this->rotation_);
  ESP_LOGCONFIG(TAG, "  Mirror: horizontal=%s, vertical=%s", YESNO(this->mirror_horizontal_),
                YESNO(this->mirror_vertical_));
  ESP_LOGCONFIG(TAG, "  Emissivity: %.2f", this->emissivity_);
  ESP_LOGCONFIG(TAG, "  Update interval: %u ms, Idle interval: %u ms", this->update_interval_,
                this->idle_update_interval_);
  this->check_uart_settings(115200);
#ifdef USE_SENSOR
  LOG_SENSOR("  ", "Avg Temperature", this->avg_temp_sensor_);
  LOG_SENSOR("  ", "Min Temperature", this->min_temp_sensor_);
  LOG_SENSOR("  ", "Max Temperature", this->max_temp_sensor_);
  LOG_SENSOR("  ", "Ambient Temperature", this->ambient_temp_sensor_);
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

camera::CameraImageReader *MCU90640Component::create_image_reader() {
  return new MCU90640CameraImageReader();
}

void MCU90640Component::request_image(camera::CameraRequester requester) {
  this->single_requesters_ |= static_cast<uint8_t>(requester);
}

void MCU90640Component::start_stream(camera::CameraRequester requester) {
  this->stream_requesters_ |= static_cast<uint8_t>(requester);
  for (auto *listener : this->listeners_) {
    listener->on_stream_start();
  }
}

void MCU90640Component::stop_stream(camera::CameraRequester requester) {
  this->stream_requesters_ &= ~static_cast<uint8_t>(requester);
  if (this->stream_requesters_ == 0) {
    for (auto *listener : this->listeners_) {
      listener->on_stream_stop();
    }
  }
}

/* -------------------- Publish -------------------- */

void MCU90640Component::publish_frame_() {
  float sum = 0.0f;
  float min_t = this->pixels_[0];
  float max_t = this->pixels_[0];
  for (int i = 0; i < MCU90640_PIXEL_COUNT; i++) {
    float t = this->pixels_[i];
    sum += t;
    if (t < min_t)
      min_t = t;
    if (t > max_t)
      max_t = t;
  }
  float avg_t = sum / static_cast<float>(MCU90640_PIXEL_COUNT);

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
  if (this->ambient_temp_sensor_ != nullptr) {
    this->ambient_temp_sensor_->publish_state(this->ta_);
  }

  if (this->centroid_x_sensor_ != nullptr || this->centroid_y_sensor_ != nullptr) {
    float total_w = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    for (int row = 0; row < MCU90640_ROWS; row++) {
      for (int col = 0; col < MCU90640_COLS; col++) {
        float w = this->pixels_[row * MCU90640_COLS + col] - min_t;
        total_w += w;
        cx += w * static_cast<float>(col);
        cy += w * static_cast<float>(row);
      }
    }
    if (total_w > 0.0f) {
      if (this->centroid_x_sensor_ != nullptr) {
        this->centroid_x_sensor_->publish_state(cx / total_w);
      }
      if (this->centroid_y_sensor_ != nullptr) {
        this->centroid_y_sensor_->publish_state(cy / total_w);
      }
    }
  }
#endif

#ifdef USE_BINARY_SENSOR
  if (this->presence_sensor_ != nullptr) {
    if (!this->background_initialized_) {
      for (int i = 0; i < MCU90640_PIXEL_COUNT; i++) {
        this->background_[i] = this->pixels_[i];
      }
      this->background_initialized_ = true;
    } else {
      uint16_t above = 0;
      for (int i = 0; i < MCU90640_PIXEL_COUNT; i++) {
        if (this->pixels_[i] - this->background_[i] > this->presence_threshold_) {
          above++;
        }
      }
      bool presence = above >= this->presence_min_pixels_;
      float alpha = presence ? 0.01f : 0.1f;
      for (int i = 0; i < MCU90640_PIXEL_COUNT; i++) {
        this->background_[i] += alpha * (this->pixels_[i] - this->background_[i]);
      }
      this->presence_sensor_->publish_state(presence);
    }
  }

  if (this->hot_spot_sensor_ != nullptr) {
    bool hot = false;
    for (int i = 0; i < MCU90640_PIXEL_COUNT; i++) {
      if (this->pixels_[i] >= this->hot_spot_threshold_) {
        hot = true;
        break;
      }
    }
    this->hot_spot_sensor_->publish_state(hot);
  }
#endif

  uint8_t single = this->single_requesters_.load();
  uint8_t stream = this->stream_requesters_.load();
  if (single == 0 && stream == 0) {
    return;
  }

  this->render_rgb_(min_t, max_t);

  size_t jpeg_len = 0;
  if (!this->encode_jpeg_(jpeg_len)) {
    return;
  }

  uint8_t requesters = single | stream;
  auto image = std::make_shared<MCU90640CameraImage>(this->jpeg_buf_, jpeg_len, requesters);
  if (!image->valid()) {
    ESP_LOGW(TAG, "Failed to allocate %u-byte camera frame", static_cast<unsigned>(jpeg_len));
    this->status_set_warning();
    return;
  }
  this->single_requesters_ = 0;
  this->status_clear_warning();
  this->current_image_ = image;
  for (auto *listener : this->listeners_) {
    listener->on_camera_image(image);
  }
}

/* -------------------- Rendering -------------------- */

void MCU90640Component::render_rgb_(float min_t, float max_t) {
  float range = max_t - min_t;
  if (range < 0.1f) {
    range = 0.1f;
  }

  const uint16_t rendered_width = this->rendered_width_();
  const uint16_t rendered_height = this->rendered_height_();
  const float w = static_cast<float>(rendered_width);
  const float h = static_cast<float>(rendered_height);
  uint8_t *dst = this->rgb_buf_;

  for (uint16_t py = 0; py < rendered_height; py++) {
    for (uint16_t px = 0; px < rendered_width; px++) {
      const float nx = w > 1.0f ? static_cast<float>(px) / (w - 1.0f) : 0.0f;
      const float ny = h > 1.0f ? static_cast<float>(py) / (h - 1.0f) : 0.0f;
      float src_x;
      float src_y;
      switch (this->rotation_) {
        case 90:
          src_x = ny * (MCU90640_COLS - 1.0f);
          src_y = (1.0f - nx) * (MCU90640_ROWS - 1.0f);
          break;
        case 180:
          src_x = (1.0f - nx) * (MCU90640_COLS - 1.0f);
          src_y = (1.0f - ny) * (MCU90640_ROWS - 1.0f);
          break;
        case 270:
          src_x = (1.0f - ny) * (MCU90640_COLS - 1.0f);
          src_y = nx * (MCU90640_ROWS - 1.0f);
          break;
        default:
          src_x = nx * (MCU90640_COLS - 1.0f);
          src_y = ny * (MCU90640_ROWS - 1.0f);
          break;
      }

      float temp = bilinear_sample_(this->pixels_, MCU90640_COLS, MCU90640_ROWS, src_x, src_y);
      float t = (temp - min_t) / range;
      t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
      uint8_t r, g, b;
      iron_color_(t, r, g, b);
      // fmt2jpg_cb with PIXFORMAT_RGB888 expects bytes in B, G, R order
      *dst++ = b;
      *dst++ = g;
      *dst++ = r;
    }
  }
}

uint16_t MCU90640Component::rendered_width_() const {
  return this->rotation_ == 90 || this->rotation_ == 270 ? this->output_height_ : this->output_width_;
}

uint16_t MCU90640Component::rendered_height_() const {
  return this->rotation_ == 90 || this->rotation_ == 270 ? this->output_width_ : this->output_height_;
}

float MCU90640Component::bilinear_sample_(const float *grid, int cols, int rows, float x, float y) {
  int x0 = static_cast<int>(x);
  int y0 = static_cast<int>(y);
  int x1 = x0 + 1;
  int y1 = y0 + 1;
  if (x1 >= cols)
    x1 = cols - 1;
  if (y1 >= rows)
    y1 = rows - 1;
  float fx = x - x0;
  float fy = y - y0;
  float v00 = grid[y0 * cols + x0];
  float v10 = grid[y0 * cols + x1];
  float v01 = grid[y1 * cols + x0];
  float v11 = grid[y1 * cols + x1];
  return v00 * (1.0f - fx) * (1.0f - fy) + v10 * fx * (1.0f - fy) + v01 * (1.0f - fx) * fy + v11 * fx * fy;
}

void MCU90640Component::iron_color_(float t, uint8_t &r, uint8_t &g, uint8_t &b) {
  int i = 0;
  while (i < IRON_STOP_COUNT - 2 && t >= IRON_STOPS[i + 1]) {
    i++;
  }
  float seg = IRON_STOPS[i + 1] - IRON_STOPS[i];
  float lt = (seg > 0.0f) ? ((t - IRON_STOPS[i]) / seg) : 0.0f;
  lt = lt < 0.0f ? 0.0f : (lt > 1.0f ? 1.0f : lt);
  r = static_cast<uint8_t>(IRON_COLORS[i][0] + lt * static_cast<float>(IRON_COLORS[i + 1][0] - IRON_COLORS[i][0]));
  g = static_cast<uint8_t>(IRON_COLORS[i][1] + lt * static_cast<float>(IRON_COLORS[i + 1][1] - IRON_COLORS[i][1]));
  b = static_cast<uint8_t>(IRON_COLORS[i][2] + lt * static_cast<float>(IRON_COLORS[i + 1][2] - IRON_COLORS[i][2]));
}

bool MCU90640Component::encode_jpeg_(size_t &out_size) {
#ifdef USE_ESP32
  JpegWriteCtx ctx{this->jpeg_buf_, MCU90640_JPEG_BUF_SIZE, 0};
  bool ok = fmt2jpg_cb(this->rgb_buf_, this->rgb_buf_size_, this->rendered_width_(), this->rendered_height_(),
                       PIXFORMAT_RGB888, this->jpeg_quality_, jpeg_write_cb, &ctx);
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

}  // namespace mcu90640
}  // namespace esphome

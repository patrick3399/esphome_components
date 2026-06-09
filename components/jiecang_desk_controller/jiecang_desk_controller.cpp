#include "jiecang_desk_controller.h"
#include "esphome/core/log.h"

namespace esphome {
namespace jiecang_desk_controller {

static const char *const TAG = "jiecang_desk_controller";

// ─── Protocol constants ───────────────────────────────────────────────────────

static constexpr uint8_t FRAME_RX_HEADER = 0xF2;
static constexpr uint8_t FRAME_TX_HEADER = 0xF1;
static constexpr uint8_t FRAME_END = 0x7E;

// RX command codes (desk → controller)
static constexpr uint8_t RX_CMD_HEIGHT = 0x01;
static constexpr uint8_t RX_CMD_PHYSICAL_LIMITS = 0x07;
static constexpr uint8_t RX_CMD_TARGET_HEIGHT = 0x1B;
static constexpr uint8_t RX_CMD_LIMIT_FLAGS = 0x20;
static constexpr uint8_t RX_CMD_LIMIT_MAX = 0x21;
static constexpr uint8_t RX_CMD_LIMIT_MIN = 0x22;
static constexpr uint8_t RX_CMD_M1 = 0x25;
static constexpr uint8_t RX_CMD_M2 = 0x26;
static constexpr uint8_t RX_CMD_M3 = 0x27;
static constexpr uint8_t RX_CMD_M4 = 0x28;

// TX command codes (controller → desk)
static constexpr uint8_t TX_CMD_STEP_UP = 0x01;
static constexpr uint8_t TX_CMD_STEP_DOWN = 0x02;
static constexpr uint8_t TX_CMD_SAVE_M1 = 0x03;
static constexpr uint8_t TX_CMD_SAVE_M2 = 0x04;
static constexpr uint8_t TX_CMD_GOTO_M1 = 0x05;
static constexpr uint8_t TX_CMD_GOTO_M2 = 0x06;
static constexpr uint8_t TX_CMD_QUERY_SETTINGS = 0x07;
static constexpr uint8_t TX_CMD_QUERY_PHYS_LIMITS = 0x0C;
static constexpr uint8_t TX_CMD_GOTO_HEIGHT = 0x1B;
static constexpr uint8_t TX_CMD_QUERY_LIMITS = 0x20;
static constexpr uint8_t TX_CMD_SAVE_M3 = 0x25;
static constexpr uint8_t TX_CMD_SAVE_M4 = 0x26;
static constexpr uint8_t TX_CMD_GOTO_M3 = 0x27;
static constexpr uint8_t TX_CMD_GOTO_M4 = 0x28;
static constexpr uint8_t TX_CMD_STOP = 0x2B;

static constexpr uint32_t WAKE_IDLE_MS = 5000;
static constexpr uint32_t WAKE_DELAY_MS = 100;
// After boot, wait this long for the desk to reply before flagging a warning.
// Queries are sent at ~0.5–0.9 s; a real desk responds within 1–2 s.
static constexpr uint32_t NO_DATA_WARN_MS = 10000;

// ─── Component lifecycle ──────────────────────────────────────────────────────

void JiecangDeskController::setup() {
  this->boot_ms_ = millis();
  this->last_rx_ms_ = this->boot_ms_;

  this->set_timeout(500, [this]() {
    this->write_raw_(TX_CMD_QUERY_PHYS_LIMITS);
    this->set_timeout(200, [this]() {
      this->write_raw_(TX_CMD_QUERY_LIMITS);
      this->set_timeout(200, [this]() { this->write_raw_(TX_CMD_QUERY_SETTINGS); });
    });
  });
}

void JiecangDeskController::loop() {
  while (this->available()) {
    uint8_t b;
    if (!this->read_byte(&b))
      break;
    this->handle_byte_(b);
  }

  // No-data watchdog: if the desk hasn't responded to our boot queries within
  // NO_DATA_WARN_MS, surface a warning so misconfigured wiring is visible in HA.
  // Cleared on the first valid frame received (see handle_message_).
  if (!this->initial_contact_ && millis() - this->boot_ms_ > NO_DATA_WARN_MS) {
    if (!this->no_data_warned_) {
      ESP_LOGW(TAG, "No data received from desk — check UART wiring and baud rate (9600 8N1)");
      this->no_data_warned_ = true;
    }
    this->status_set_warning();
  }
}

void JiecangDeskController::dump_config() {
  ESP_LOGCONFIG(TAG, "Jiecang Desk Controller:");
  this->check_uart_settings(9600);
  ESP_LOGCONFIG(TAG, "  Height: %.1f cm", this->current_height_);
  if (this->limits_known_) {
    ESP_LOGCONFIG(TAG, "  Physical limits: min=%.1f cm  max=%.1f cm", this->physical_min_, this->physical_max_);
    ESP_LOGCONFIG(TAG, "  Effective limits: min=%.1f cm  max=%.1f cm", this->limit_min_, this->limit_max_);
  }
  LOG_SENSOR("  ", "Height", this->height_sensor_);
  LOG_SENSOR("  ", "Height Min", this->height_min_sensor_);
  LOG_SENSOR("  ", "Height Max", this->height_max_sensor_);
  LOG_SENSOR("  ", "Height %", this->height_pct_sensor_);
  LOG_SENSOR("  ", "M1", this->m1_sensor_);
  LOG_SENSOR("  ", "M2", this->m2_sensor_);
  LOG_SENSOR("  ", "M3", this->m3_sensor_);
  LOG_SENSOR("  ", "M4", this->m4_sensor_);
}

// ─── UART RX parser ───────────────────────────────────────────────────────────

void JiecangDeskController::handle_byte_(uint8_t b) {
  switch (this->parse_state_) {
    case ParseState::WAIT_FIRST_F2:
      if (b == FRAME_RX_HEADER)
        this->parse_state_ = ParseState::WAIT_SECOND_F2;
      break;

    case ParseState::WAIT_SECOND_F2:
      if (b == FRAME_RX_HEADER) {
        this->rx_len_ = 0;
        this->parse_state_ = ParseState::BUFFERING;
      } else {
        this->parse_state_ = ParseState::WAIT_FIRST_F2;
      }
      break;

    case ParseState::BUFFERING:
      // 0x7E is treated as frame terminator; if the desk ever embeds 0x7E in a
      // payload the frame will end early. Known Jiecang commands don't do this.
      if (b == FRAME_END) {
        this->handle_message_();
        this->parse_state_ = ParseState::WAIT_FIRST_F2;
      } else if (this->rx_len_ < this->rx_buf_.size()) {
        this->rx_buf_[this->rx_len_++] = b;
      } else {
        ESP_LOGW(TAG, "RX buffer overflow — dropping frame");
        this->parse_state_ = ParseState::WAIT_FIRST_F2;
      }
      break;
  }
}

void JiecangDeskController::handle_message_() {
  if (this->rx_len_ < 3) {
    ESP_LOGW(TAG, "Frame too short (%u bytes) — ignoring", this->rx_len_);
    return;
  }

  const uint8_t cmd = this->rx_buf_[0];
  const uint8_t param_cnt = this->rx_buf_[1];

  if (this->rx_len_ < static_cast<uint8_t>(3 + param_cnt)) {
    ESP_LOGW(TAG, "Frame truncated: cmd=0x%02X expected %u bytes got %u", cmd, 3 + param_cnt, this->rx_len_);
    return;
  }

  const uint8_t *p = &this->rx_buf_[2];
  const uint8_t received_checksum = this->rx_buf_[2 + param_cnt];
  uint8_t computed_checksum = cmd + param_cnt;
  for (uint8_t i = 0; i < param_cnt; i++)
    computed_checksum += p[i];

  if (received_checksum != computed_checksum) {
    ESP_LOGW(TAG, "Checksum mismatch: cmd=0x%02X received=0x%02X computed=0x%02X", cmd, received_checksum,
             computed_checksum);
    return;
  }

  this->last_rx_ms_ = millis();
  if (!this->initial_contact_) {
    this->initial_contact_ = true;
    this->status_clear_warning();
  }

  ESP_LOGV(TAG, "RX cmd=0x%02X param_cnt=%u", cmd, param_cnt);

  switch (cmd) {
    case RX_CMD_HEIGHT: {
      if (param_cnt < 2)
        break;
      const float h = decode_height_(p[0], p[1]);
      if (h == this->current_height_)
        break;
      this->current_height_ = h;
      if (this->height_sensor_ != nullptr)
        this->height_sensor_->publish_state(h);
      if (this->height_number_ != nullptr)
        this->height_number_->publish_state(h);
      if (this->limits_known_) {
        const float range = this->limit_max_ - this->limit_min_;
        if (range > 0.0f) {
          const float pct = (h - this->limit_min_) / range * 100.0f;
          if (this->height_pct_sensor_ != nullptr)
            this->height_pct_sensor_->publish_state(pct);
          if (this->height_pct_number_ != nullptr)
            this->height_pct_number_->publish_state(pct);
        }
      }
      break;
    }

    case RX_CMD_PHYSICAL_LIMITS: {
      if (param_cnt < 4)
        break;
      this->physical_max_ = decode_height_(p[0], p[1]);
      this->physical_min_ = decode_height_(p[2], p[3]);
      this->limit_max_ = this->physical_max_;
      this->limit_min_ = this->physical_min_;
      this->limits_known_ = true;
      if (this->height_max_sensor_ != nullptr)
        this->height_max_sensor_->publish_state(this->limit_max_);
      if (this->height_min_sensor_ != nullptr)
        this->height_min_sensor_->publish_state(this->limit_min_);
      if (this->height_number_ != nullptr) {
        this->height_number_->traits.set_min_value(this->limit_min_);
        this->height_number_->traits.set_max_value(this->limit_max_);
      }
      ESP_LOGD(TAG, "Physical limits — min: %.1f cm  max: %.1f cm", this->physical_min_, this->physical_max_);
      break;
    }

    case RX_CMD_TARGET_HEIGHT: {
      if (param_cnt < 2)
        break;
      ESP_LOGV(TAG, "Target height: %.1f cm", decode_height_(p[0], p[1]));
      break;
    }

    case RX_CMD_LIMIT_FLAGS: {
      if (param_cnt < 1)
        break;
      const bool has_max = (p[0] & 0x01) != 0;
      const bool has_min = (p[0] >> 4) != 0;
      if (!has_max && this->physical_max_ > 0.0f) {
        this->limit_max_ = this->physical_max_;
        if (this->height_max_sensor_ != nullptr)
          this->height_max_sensor_->publish_state(this->limit_max_);
        if (this->height_number_ != nullptr)
          this->height_number_->traits.set_max_value(this->limit_max_);
      }
      if (!has_min && this->physical_min_ > 0.0f) {
        this->limit_min_ = this->physical_min_;
        if (this->height_min_sensor_ != nullptr)
          this->height_min_sensor_->publish_state(this->limit_min_);
        if (this->height_number_ != nullptr)
          this->height_number_->traits.set_min_value(this->limit_min_);
      }
      ESP_LOGD(TAG, "Limit flags: max_set=%s min_set=%s", has_max ? "yes" : "no", has_min ? "yes" : "no");
      break;
    }

    case RX_CMD_LIMIT_MAX: {
      if (param_cnt < 2)
        break;
      this->limit_max_ = decode_height_(p[0], p[1]);
      if (this->height_max_sensor_ != nullptr)
        this->height_max_sensor_->publish_state(this->limit_max_);
      if (this->height_number_ != nullptr)
        this->height_number_->traits.set_max_value(this->limit_max_);
      break;
    }

    case RX_CMD_LIMIT_MIN: {
      if (param_cnt < 2)
        break;
      this->limit_min_ = decode_height_(p[0], p[1]);
      if (this->height_min_sensor_ != nullptr)
        this->height_min_sensor_->publish_state(this->limit_min_);
      if (this->height_number_ != nullptr)
        this->height_number_->traits.set_min_value(this->limit_min_);
      break;
    }

    case RX_CMD_M1:
      if (param_cnt >= 2 && this->m1_sensor_ != nullptr)
        this->m1_sensor_->publish_state(decode_height_(p[0], p[1]));
      break;

    case RX_CMD_M2:
      if (param_cnt >= 2 && this->m2_sensor_ != nullptr)
        this->m2_sensor_->publish_state(decode_height_(p[0], p[1]));
      break;

    case RX_CMD_M3:
      if (param_cnt >= 2 && this->m3_sensor_ != nullptr)
        this->m3_sensor_->publish_state(decode_height_(p[0], p[1]));
      break;

    case RX_CMD_M4:
      if (param_cnt >= 2 && this->m4_sensor_ != nullptr)
        this->m4_sensor_->publish_state(decode_height_(p[0], p[1]));
      break;

    default:
      ESP_LOGV(TAG, "Unhandled RX cmd=0x%02X", cmd);
      break;
  }
}

float JiecangDeskController::decode_height_(uint8_t hi, uint8_t lo) {
  return static_cast<float>((static_cast<uint16_t>(hi) << 8) | lo) / 10.0f;
}

// ─── TX helpers ───────────────────────────────────────────────────────────────

void JiecangDeskController::write_raw_(uint8_t cmd) {
  const std::array<uint8_t, 6> frame = {FRAME_TX_HEADER, FRAME_TX_HEADER, cmd, 0x00, cmd, FRAME_END};
  this->write_array(frame);
  ESP_LOGV(TAG, "TX cmd=0x%02X", cmd);
}

void JiecangDeskController::send_cmd_(uint8_t cmd) {
  if (millis() - this->last_rx_ms_ >= WAKE_IDLE_MS) {
    ESP_LOGD(TAG, "Desk idle — sending wake-up before cmd=0x%02X", cmd);
    this->write_raw_(TX_CMD_STOP);
    this->set_timeout(WAKE_DELAY_MS, [this, cmd]() { this->write_raw_(cmd); });
  } else {
    this->write_raw_(cmd);
  }
}

void JiecangDeskController::send_goto_height_(float height_cm) {
  const uint16_t raw = static_cast<uint16_t>(height_cm * 10.0f);
  const uint8_t hi = static_cast<uint8_t>(raw >> 8);
  const uint8_t lo = static_cast<uint8_t>(raw & 0xFF);
  const uint8_t checksum = static_cast<uint8_t>((TX_CMD_GOTO_HEIGHT + 0x02u + hi + lo) & 0xFF);
  const std::array<uint8_t, 8> frame = {FRAME_TX_HEADER, FRAME_TX_HEADER, TX_CMD_GOTO_HEIGHT, 0x02, hi, lo,
                                        checksum,        FRAME_END};
  this->write_array(frame);
  ESP_LOGD(TAG, "TX goto_height %.1f cm (raw=%u)", height_cm, raw);
}

// ─── Public command API ───────────────────────────────────────────────────────

void JiecangDeskController::step_up() {
  this->send_cmd_(TX_CMD_STEP_UP);
}

void JiecangDeskController::step_down() {
  this->send_cmd_(TX_CMD_STEP_DOWN);
}

void JiecangDeskController::stop() {
  this->write_raw_(TX_CMD_STOP);
}

void JiecangDeskController::move_up() {
  if (!this->limits_known_) {
    ESP_LOGW(TAG, "move_up: limits not yet known");
    return;
  }
  this->goto_height(this->limit_max_);
}

void JiecangDeskController::move_down() {
  if (!this->limits_known_) {
    ESP_LOGW(TAG, "move_down: limits not yet known");
    return;
  }
  this->goto_height(this->limit_min_);
}

void JiecangDeskController::goto_preset(uint8_t preset) {
  uint8_t cmd;
  switch (preset) {
    case 1:
      cmd = TX_CMD_GOTO_M1;
      break;
    case 2:
      cmd = TX_CMD_GOTO_M2;
      break;
    case 3:
      cmd = TX_CMD_GOTO_M3;
      break;
    case 4:
      cmd = TX_CMD_GOTO_M4;
      break;
    default:
      ESP_LOGW(TAG, "goto_preset: invalid preset %u", preset);
      return;
  }
  this->send_cmd_(cmd);
}

void JiecangDeskController::save_preset(uint8_t preset) {
  uint8_t cmd;
  switch (preset) {
    case 1:
      cmd = TX_CMD_SAVE_M1;
      break;
    case 2:
      cmd = TX_CMD_SAVE_M2;
      break;
    case 3:
      cmd = TX_CMD_SAVE_M3;
      break;
    case 4:
      cmd = TX_CMD_SAVE_M4;
      break;
    default:
      ESP_LOGW(TAG, "save_preset: invalid preset %u", preset);
      return;
  }
  this->send_cmd_(cmd);
}

void JiecangDeskController::goto_height(float height_cm) {
  if (millis() - this->last_rx_ms_ >= WAKE_IDLE_MS) {
    ESP_LOGD(TAG, "Desk idle — sending wake-up before goto_height");
    this->write_raw_(TX_CMD_STOP);
    this->set_timeout(WAKE_DELAY_MS, [this, height_cm]() { this->send_goto_height_(height_cm); });
  } else {
    this->send_goto_height_(height_cm);
  }
}

void JiecangDeskController::goto_height_pct(float pct) {
  if (!this->limits_known_) {
    ESP_LOGW(TAG, "goto_height_pct: limits not yet known");
    return;
  }
  this->goto_height(this->limit_min_ + (this->limit_max_ - this->limit_min_) * pct / 100.0f);
}

}  // namespace jiecang_desk_controller
}  // namespace esphome

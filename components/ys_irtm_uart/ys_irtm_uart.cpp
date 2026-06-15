#include "ys_irtm_uart.h"

namespace esphome {
namespace ys_irtm_uart {

static const char *const TAG = "ys_irtm_uart";

// NEC spec requires ≥108 ms between frames; 110 ms adds 2 ms margin.
static const uint32_t NEC_REPEAT_INTERVAL_MS = 110;

static const uint32_t BAUD_TABLE[] = {0, 4800, 9600, 19200, 57600};

void YSIrtmUartComponent::setup() {}

void YSIrtmUartComponent::loop() {
  // Drain ACK bytes first. The module echoes the command byte (~27 ms after TX,
  // once the IR burst completes). Consuming them here prevents the single ACK byte
  // from prepending an incoming IR frame and causing a spurious on_ir_receive trigger.
  while (this->pending_acks_ > 0 && this->available() > 0) {
    uint8_t ack;
    this->read_byte(&ack);
    this->pending_acks_--;
    ESP_LOGV(TAG, "TX ACK: 0x%02X", ack);
  }

  // RX: consume complete 3-byte IR receive frames
  // Protocol note: no framing header — a stray byte from noise will misalign subsequent reads
  // until the module is power-cycled. Acceptable for this module's typical usage pattern.
  while (this->available() >= 3) {
    uint8_t buf[3];
    this->read_array(buf, 3);
    ESP_LOGD(TAG, "IR RX: UserCode=0x%02X%02X  KeyCode=0x%02X", buf[0], buf[1], buf[2]);
    this->ir_receive_callback_.call(buf[0], buf[1], buf[2]);
  }

  // TX: fire pending NEC repeat frames without blocking.
  // Uses millis()-difference to handle uint32_t overflow correctly.
  if (this->pending_.has_value()) {
    auto &tx = *this->pending_;
    if (millis() - tx.last_sent_ms >= NEC_REPEAT_INTERVAL_MS) {
      this->send_command_(0xF1, tx.user_code_hi, tx.user_code_lo, tx.key_code);
      ESP_LOGD(TAG, "NEC TX repeat: UserCode=0x%02X%02X  KeyCode=0x%02X  (%u remaining)", tx.user_code_hi,
               tx.user_code_lo, tx.key_code, tx.remaining);
      if (tx.remaining == 0) {
        this->pending_.reset();
      } else {
        tx.remaining--;
        tx.last_sent_ms = millis();
      }
    }
  }
}

void YSIrtmUartComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "YS-IRTM UART:");
  ESP_LOGCONFIG(TAG, "  Device Address: 0x%02X", this->device_address_);
  this->check_uart_settings(this->parent_->get_baud_rate());
}

void YSIrtmUartComponent::send_command_(uint8_t cmd, uint8_t d1, uint8_t d2, uint8_t d3) {
  uint8_t frame[5] = {this->device_address_, cmd, d1, d2, d3};
  this->write_array(frame, 5);
  // No flush(): 5 bytes fit in the TX FIFO and write_array returns immediately.
  // Module will echo cmd as a 1-byte ACK after the IR burst (~27 ms); track it for loop().
  this->pending_acks_++;
}

bool YSIrtmUartComponent::send_raw_hex(const std::string &data) {
  constexpr size_t MAX_BYTES = 64;
  uint8_t buf[MAX_BYTES];
  size_t len = 0;

  auto hex_val = [](char c) -> int {
    if (c >= '0' && c <= '9')
      return c - '0';
    if (c >= 'A' && c <= 'F')
      return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
      return c - 'a' + 10;
    return -1;
  };

  int high = -1;
  for (char c : data) {
    if (c == ' ' || c == ',' || c == ':' || c == '-' || c == '\t' || c == '\n' || c == '\r')
      continue;
    const int v = hex_val(c);
    if (v < 0) {
      ESP_LOGW(TAG, "Invalid hex char: %c", c);
      return false;
    }
    if (high < 0) {
      high = v;
    } else {
      if (len >= MAX_BYTES) {
        ESP_LOGW(TAG, "Raw payload too long (max %u bytes)", (unsigned)MAX_BYTES);
        return false;
      }
      buf[len++] = static_cast<uint8_t>((high << 4) | v);
      high = -1;
    }
  }

  if (high >= 0 || len == 0) {
    ESP_LOGW(TAG, "Invalid hex payload (odd nibble count or empty)");
    return false;
  }

  this->write_array(buf, len);
  ESP_LOGI(TAG, "Sent %u raw bytes via UART", (unsigned)len);
  return true;
}

bool YSIrtmUartComponent::send_nec(uint8_t user_code_hi, uint8_t user_code_lo, uint8_t key_code, uint8_t repeats) {
  if (this->pending_.has_value() && this->pending_->remaining > 0) {
    ESP_LOGW(TAG, "TX busy: replacing %u pending repeat(s) with new command", this->pending_->remaining);
  }

  this->send_command_(0xF1, user_code_hi, user_code_lo, key_code);
  ESP_LOGI(TAG, "NEC TX: UserCode=0x%02X%02X  KeyCode=0x%02X  (1/%u)", user_code_hi, user_code_lo, key_code,
           repeats + 1);

  if (repeats > 0) {
    this->pending_ = PendingNec{user_code_hi, user_code_lo, key_code, static_cast<uint8_t>(repeats - 1), millis()};
  } else {
    this->pending_.reset();
  }
  return true;
}

bool YSIrtmUartComponent::send_proxy_packet(const std::string &protocol, uint32_t address, uint32_t command,
                                            uint8_t repeats) {
  const uint8_t user_code_hi = static_cast<uint8_t>(address & 0xFF);
  const uint8_t user_code_lo = static_cast<uint8_t>((address >> 8) & 0xFF);
  const uint8_t key_code = static_cast<uint8_t>(command & 0xFF);

  ESP_LOGI(TAG, "Proxy → NEC: protocol=%s  address=0x%04X  command=0x%02X  repeats=%u", protocol.c_str(),
           static_cast<unsigned int>(address & 0xFFFF), static_cast<unsigned int>(key_code), repeats);

  return this->send_nec(user_code_hi, user_code_lo, key_code, repeats);
}

bool YSIrtmUartComponent::set_module_address(uint8_t new_address) {
  ESP_LOGI(TAG, "Setting module address: 0x%02X → 0x%02X", this->device_address_, new_address);
  this->send_command_(0xF2, new_address, 0x00, 0x00);
  this->device_address_ = new_address;
  return true;
}

bool YSIrtmUartComponent::set_module_baudrate(YSIrtmBaudRate baud_id) {
  const uint32_t baud = (baud_id >= 1 && baud_id <= 4) ? BAUD_TABLE[baud_id] : 0;
  ESP_LOGI(TAG, "Setting module baud rate: ID=0x%02X (%u bps) — takes effect after power cycle", baud_id,
           static_cast<unsigned int>(baud));
  this->send_command_(0xF3, static_cast<uint8_t>(baud_id), 0x00, 0x00);
  this->flush();  // must drain before the host baud rate changes
  return true;
}

// ---------------------------------------------------------------------------
// YsIrtmInfrared — infrared platform implementation
// ---------------------------------------------------------------------------
#ifdef USE_IR_RF

void YsIrtmInfrared::setup() {
  // YS-IRTM is TX-only; RX is handled by ir_rf_proxy + remote_receiver.
  // Do NOT call Infrared::setup() — it checks transmitter_/receiver_ pointers
  // which are unused here (we drive UART directly).
  this->traits_.set_supports_transmitter(true);
  this->traits_.set_supports_receiver(false);
}

void YsIrtmInfrared::dump_config() {
  ESP_LOGCONFIG(TAG, "YS-IRTM Infrared TX (NEC only):");
  ESP_LOGCONFIG(TAG, "  Name: %s", this->get_name().c_str());
}

void YsIrtmInfrared::control(const infrared::InfraredCall &call) {
  if (this->ys_ == nullptr) {
    ESP_LOGW(TAG, "infrared: no ys_irtm_uart configured");
    return;
  }
  if (!call.has_raw_timings()) {
    ESP_LOGE(TAG, "infrared: no raw timings in call");
    return;
  }

  // Decode raw timings from InfraredCall (supports packed / base64url / vector).
  // We stage into RemoteTransmitData first so we can run NECProtocol::decode()
  // regardless of which encoding HA sent.
  remote_base::RemoteTransmitData tx_buf;
  if (call.is_packed()) {
    tx_buf.set_data_from_packed_sint32(call.get_packed_data(), call.get_packed_length(), call.get_packed_count());
  } else if (call.is_base64url()) {
    if (!tx_buf.set_data_from_base64url(call.get_base64url_data())) {
      ESP_LOGE(TAG, "infrared: invalid base64url timings");
      return;
    }
  } else {
    tx_buf.set_data(call.get_raw_timings());
  }

  remote_base::RemoteReceiveData rx_data(tx_buf.get_data(), 25, remote_base::TOLERANCE_MODE_PERCENTAGE);
  auto nec = remote_base::NECProtocol().decode(rx_data);
  if (!nec.has_value()) {
    ESP_LOGW(TAG, "infrared: could not decode NEC — YS-IRTM supports NEC only");
    return;
  }

  // NEC address is 16-bit; map to YS-IRTM user_code bytes (lo byte first).
  const uint8_t user_code_hi = static_cast<uint8_t>(nec->address & 0xFF);
  const uint8_t user_code_lo = static_cast<uint8_t>((nec->address >> 8) & 0xFF);
  const uint8_t key_code = static_cast<uint8_t>(nec->command & 0xFF);
  // repeat_count ≥ 1 (1 = single transmission); convert to YS-IRTM repeat frames.
  const uint8_t repeats = call.get_repeat_count() > 1 ? static_cast<uint8_t>(call.get_repeat_count() - 1) : 0;

  ESP_LOGI(TAG, "infrared → NEC TX: address=0x%04X  command=0x%04X  repeats=%u", static_cast<unsigned>(nec->address),
           static_cast<unsigned>(nec->command), repeats);
  this->ys_->send_nec(user_code_hi, user_code_lo, key_code, repeats);
}

#endif  // USE_IR_RF

}  // namespace ys_irtm_uart
}  // namespace esphome

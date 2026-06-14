#include "esphome/core/defines.h"
#if defined(USE_ESP32) && defined(WLED_BRIDGE_REALTIME)
#include "wled_udp.h"
#include "wled_bridge.h"
#include "wled_color.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <cerrno>
#include <cstdio>

namespace esphome {
namespace wled_bridge {

static const char *const TAG = "wled_bridge.realtime";
static constexpr uint32_t REALTIME_SOURCE_REJECT_LOG_INTERVAL_MS = 5000;

static void format_source_ip(uint32_t source_ip, char *out, size_t out_len) {
  if (out == nullptr || out_len == 0)
    return;
  struct in_addr addr = {};
  addr.s_addr = source_ip;
  if (inet_ntop(AF_INET, &addr, out, out_len) == nullptr)
    snprintf(out, out_len, "unknown");
}

static bool realtime_source_allowed(const char *protocol, uint32_t source_ip, bool receiving, uint32_t *locked_source,
                                    uint32_t *last_reject_log_ms) {
  if (locked_source == nullptr || last_reject_log_ms == nullptr)
    return true;
  if (!receiving || *locked_source == 0 || *locked_source == source_ip) {
    *locked_source = source_ip;
    return true;
  }

  uint32_t now = millis();
  if (*last_reject_log_ms == 0 || now - *last_reject_log_ms >= REALTIME_SOURCE_REJECT_LOG_INTERVAL_MS) {
    char incoming[16];
    char active[16];
    format_source_ip(source_ip, incoming, sizeof(incoming));
    format_source_ip(*locked_source, active, sizeof(active));
    ESP_LOGW(TAG, "%s: ignoring realtime packet from %s while locked to %s", protocol, incoming, active);
    *last_reject_log_ms = now;
  }
  return false;
}

// ============================================================
// DDP (Distributed Display Protocol) receiver — UDP port 4048
// ============================================================

static constexpr size_t DDP_HEADER_LEN = 10;
static constexpr uint8_t DDP_FLAGS_VER1 = 0x40;
static constexpr uint8_t DDP_FLAGS_PUSH = 0x01;
static constexpr uint8_t DDP_FLAGS_QUERY = 0x02;
static constexpr uint8_t DDP_FLAGS_REPLY = 0x04;
static constexpr uint8_t DDP_FLAGS_STORAGE = 0x08;
static constexpr uint8_t DDP_ID_DISPLAY = 1;
static constexpr uint8_t DDP_ID_ALL = 255;
static constexpr uint32_t DDP_TIMEOUT_MS = 2500;

void WLEDDdpReceiver::setup(WLEDBridgeComponent *comp, bool enabled) {
  this->comp_ = comp;
  this->enabled_ = enabled;
  if (enabled)
    this->open_socket_();
  ESP_LOGCONFIG(TAG, "DDP receiver on port %u: %s", WLED_DDP_PORT, enabled ? "enabled" : "disabled");
}

void WLEDDdpReceiver::set_enabled(bool enabled) {
  this->enabled_ = enabled;
  if (enabled)
    this->open_socket_();
  else
    this->close_socket_();
}

void WLEDDdpReceiver::open_socket_() {
  if (this->fd_ >= 0)
    return;
  int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0) {
    ESP_LOGW(TAG, "DDP: could not create socket: %d", errno);
    return;
  }
  int enable = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
  int flags = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, flags | O_NONBLOCK);

  struct sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(WLED_DDP_PORT);
  if (bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    ESP_LOGW(TAG, "DDP: bind failed on port %u: %d", WLED_DDP_PORT, errno);
    close(fd);
    return;
  }
  this->fd_ = fd;
}

void WLEDDdpReceiver::close_socket_() {
  if (this->fd_ >= 0) {
    close(this->fd_);
    this->fd_ = -1;
  }
  this->receiving_ = false;
  this->source_ip_ = 0;
}

void WLEDDdpReceiver::loop() {
  if (!this->enabled_ || this->fd_ < 0)
    return;

  uint8_t buf[1472];
  struct sockaddr_in src = {};
  socklen_t src_len = sizeof(src);

  for (int i = 0; i < 8; i++) {
    ssize_t len = recvfrom(this->fd_, buf, sizeof(buf), 0, reinterpret_cast<struct sockaddr *>(&src), &src_len);
    if (len <= 0)
      break;
    if (static_cast<size_t>(len) >= DDP_HEADER_LEN)
      this->process_packet_(buf, static_cast<size_t>(len), src.sin_addr.s_addr);
  }

  if (this->receiving_ && (millis() - this->last_receive_ms_ > DDP_TIMEOUT_MS)) {
    this->receiving_ = false;
    this->source_ip_ = 0;
    this->comp_->clear_pixel_overrides();
    ESP_LOGD(TAG, "DDP: stream timeout, returning to effects");
  }
}

void WLEDDdpReceiver::process_packet_(const uint8_t *buf, size_t len, uint32_t source_ip) {
  if (buf == nullptr || len < DDP_HEADER_LEN)
    return;
  uint8_t flags = buf[0];

  if ((flags & DDP_FLAGS_VER1) == 0)
    return;
  if (flags & (DDP_FLAGS_QUERY | DDP_FLAGS_REPLY | DDP_FLAGS_STORAGE))
    return;

  uint8_t dest = buf[3];
  if (dest != DDP_ID_DISPLAY && dest != DDP_ID_ALL)
    return;
  if (!realtime_source_allowed("DDP", source_ip, this->receiving_, &this->source_ip_,
                               &this->last_source_reject_log_ms_))
    return;

  uint32_t channel_offset = (static_cast<uint32_t>(buf[4]) << 24) | (static_cast<uint32_t>(buf[5]) << 16) |
                            (static_cast<uint32_t>(buf[6]) << 8) | buf[7];
  uint16_t data_len = (static_cast<uint16_t>(buf[8]) << 8) | buf[9];

  if (DDP_HEADER_LEN + data_len > len)
    data_len = static_cast<uint16_t>(len - DDP_HEADER_LEN);

  const uint8_t *pixel_data = buf + DDP_HEADER_LEN;

  uint8_t data_type = buf[2];
  uint8_t channels_per_pixel = ((data_type >> 3) & 0x07);
  if (channels_per_pixel < 3)
    channels_per_pixel = 3;

  uint32_t led_count = this->comp_->get_led_count();
  uint32_t start_led = channel_offset / channels_per_pixel;
  uint32_t pixel_count = data_len / channels_per_pixel;

  bool changed = false;
  for (uint32_t i = 0; i < pixel_count && (start_led + i) < led_count; i++) {
    const uint8_t *p = pixel_data + i * channels_per_pixel;
    uint8_t r = p[0];
    uint8_t g = p[1];
    uint8_t b = p[2];
    uint8_t w = (channels_per_pixel >= 4) ? p[3] : 0;
    changed |= this->comp_->set_pixel_override(0, start_led + i, RGBW32(r, g, b, w), false);
  }
  if (changed)
    this->comp_->mark_pixel_overrides_changed();

  this->last_receive_ms_ = millis();
  if (!this->receiving_) {
    ESP_LOGD(TAG, "DDP: receiving pixel data (offset=%u, %u pixels)", start_led, pixel_count);
    this->receiving_ = true;
  }

  // DDP PUSH → display the (possibly multi-packet) frame immediately instead of
  // waiting for the next frame tick. If the sender never uses PUSH, fall back to
  // rendering each packet so the stream isn't stalled.
  if (flags & DDP_FLAGS_PUSH) {
    this->push_seen_ = true;
    this->comp_->request_realtime_render();
  } else if (!this->push_seen_) {
    this->comp_->request_realtime_render();
  }
}

// ============================================================
// E1.31 (sACN) receiver — UDP port 5568
// ============================================================

static constexpr size_t E131_MIN_PACKET = 126;
static constexpr size_t E131_UNIVERSE_OFFSET = 113;
static constexpr size_t E131_SEQUENCE_OFFSET = 111;
static constexpr size_t E131_OPTIONS_OFFSET = 112;
static constexpr size_t E131_DMX_START_CODE_OFFSET = 125;
static constexpr size_t E131_DMX_DATA_OFFSET = 126;
static constexpr uint16_t E131_CHANNELS_PER_UNIVERSE = 512;
static constexpr uint16_t E131_PIXELS_PER_UNIVERSE = 170;
static constexpr uint32_t E131_TIMEOUT_MS = 2500;
// ACN packet identifier (bytes 4-15)
static constexpr uint8_t E131_ACN_ID[] = {0x41, 0x53, 0x43, 0x2d, 0x45, 0x31, 0x2e, 0x31, 0x37, 0x00, 0x00, 0x00};

void WLEDE131Receiver::setup(WLEDBridgeComponent *comp, bool enabled, uint16_t start_universe, uint8_t universe_count) {
  this->comp_ = comp;
  this->enabled_ = enabled;
  this->start_universe_ = start_universe;
  this->universe_count_ = universe_count;
  if (enabled)
    this->open_socket_();
  ESP_LOGCONFIG(TAG, "E1.31 receiver on port %u: %s (universe %u-%u)", WLED_E131_PORT, enabled ? "enabled" : "disabled",
                start_universe, start_universe + universe_count - 1);
}

void WLEDE131Receiver::set_enabled(bool enabled) {
  this->enabled_ = enabled;
  if (enabled)
    this->open_socket_();
  else
    this->close_socket_();
}

void WLEDE131Receiver::open_socket_() {
  if (this->fd_ >= 0)
    return;
  int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0) {
    ESP_LOGW(TAG, "E1.31: could not create socket: %d", errno);
    return;
  }
  int enable = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
  int flags = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, flags | O_NONBLOCK);

  struct sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(WLED_E131_PORT);
  if (bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    ESP_LOGW(TAG, "E1.31: bind failed on port %u: %d", WLED_E131_PORT, errno);
    close(fd);
    return;
  }

  // Join multicast groups for configured universes
  for (uint8_t i = 0; i < this->universe_count_; i++) {
    uint16_t uni = this->start_universe_ + i;
    struct ip_mreq mreq = {};
    uint16_t uni_minus_1 = uni - 1;
    mreq.imr_multiaddr.s_addr = htonl(0xEFFF0000u | uni_minus_1);  // 239.255.x.y
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));
  }

  this->fd_ = fd;
}

void WLEDE131Receiver::close_socket_() {
  if (this->fd_ >= 0) {
    close(this->fd_);
    this->fd_ = -1;
  }
  this->receiving_ = false;
  this->source_ip_ = 0;
}

void WLEDE131Receiver::loop() {
  if (!this->enabled_ || this->fd_ < 0)
    return;

  uint8_t buf[638];
  struct sockaddr_in src = {};
  socklen_t src_len = sizeof(src);

  for (int i = 0; i < 8; i++) {
    ssize_t len = recvfrom(this->fd_, buf, sizeof(buf), 0, reinterpret_cast<struct sockaddr *>(&src), &src_len);
    if (len <= 0)
      break;
    if (static_cast<size_t>(len) >= E131_MIN_PACKET)
      this->process_packet_(buf, static_cast<size_t>(len), src.sin_addr.s_addr);
  }

  if (this->receiving_ && (millis() - this->last_receive_ms_ > E131_TIMEOUT_MS)) {
    this->receiving_ = false;
    this->source_ip_ = 0;
    this->comp_->clear_pixel_overrides();
    ESP_LOGD(TAG, "E1.31: stream timeout, returning to effects");
  }
}

void WLEDE131Receiver::process_packet_(const uint8_t *buf, size_t len, uint32_t source_ip) {
  if (buf == nullptr || len < E131_MIN_PACKET)
    return;
  // Verify ACN packet identifier
  if (memcmp(buf + 4, E131_ACN_ID, sizeof(E131_ACN_ID)) != 0)
    return;

  // Check options — bit 7 = preview (skip), bit 6 = stream terminated (skip)
  uint8_t options = buf[E131_OPTIONS_OFFSET];
  if (options & 0xC0)
    return;

  // Check DMX start code (must be 0x00 for standard DMX512)
  if (buf[E131_DMX_START_CODE_OFFSET] != 0x00)
    return;

  uint16_t universe = (static_cast<uint16_t>(buf[E131_UNIVERSE_OFFSET]) << 8) | buf[E131_UNIVERSE_OFFSET + 1];
  if (universe < this->start_universe_ || universe >= this->start_universe_ + this->universe_count_)
    return;
  if (!realtime_source_allowed("E1.31", source_ip, this->receiving_, &this->source_ip_,
                               &this->last_source_reject_log_ms_))
    return;

  uint8_t uni_index = static_cast<uint8_t>(universe - this->start_universe_);

  // Sequence check (wrapping 1-255, 0 = no sequence)
  uint8_t seq = buf[E131_SEQUENCE_OFFSET];
  if (seq != 0 && uni_index < 8) {
    uint8_t last = this->last_seq_[uni_index];
    if (last != 0) {
      int8_t diff = static_cast<int8_t>(seq - last);
      if (diff < 0 && diff > -20)
        return;  // out of order
    }
    this->last_seq_[uni_index] = seq;
  }

  // Extract DMX channel data
  size_t avail_channels = len - E131_DMX_DATA_OFFSET;
  if (avail_channels > E131_CHANNELS_PER_UNIVERSE)
    avail_channels = E131_CHANNELS_PER_UNIVERSE;
  const uint8_t *dmx = buf + E131_DMX_DATA_OFFSET;

  uint32_t led_count = this->comp_->get_led_count();
  uint32_t start_led = static_cast<uint32_t>(uni_index) * E131_PIXELS_PER_UNIVERSE;
  uint32_t pixel_count = static_cast<uint32_t>(avail_channels / 3);

  bool changed = false;
  for (uint32_t i = 0; i < pixel_count && (start_led + i) < led_count; i++) {
    uint8_t r = dmx[i * 3];
    uint8_t g = dmx[i * 3 + 1];
    uint8_t b = dmx[i * 3 + 2];
    changed |= this->comp_->set_pixel_override(0, start_led + i, RGBW32(r, g, b, 0), false);
  }
  if (changed)
    this->comp_->mark_pixel_overrides_changed();

  this->last_receive_ms_ = millis();
  if (!this->receiving_) {
    ESP_LOGD(TAG, "E1.31: receiving on universe %u (%u pixels)", universe, pixel_count);
    this->receiving_ = true;
  }
}

// ============================================================
// Art-Net receiver — UDP port 6454
// ============================================================

static constexpr size_t ARTNET_MIN_PACKET = 18;
static constexpr size_t ARTNET_DMX_DATA_OFFSET = 18;
static constexpr uint16_t ARTNET_OPCODE_DMX = 0x5000;  // little-endian on wire: 00 50
static constexpr uint16_t ARTNET_PIXELS_PER_UNIVERSE = 170;
static constexpr uint32_t ARTNET_TIMEOUT_MS = 2500;
static constexpr uint8_t ARTNET_ID[] = {'A', 'r', 't', '-', 'N', 'e', 't', 0x00};

void WLEDArtNetReceiver::setup(WLEDBridgeComponent *comp, bool enabled, uint16_t start_universe,
                               uint8_t universe_count) {
  this->comp_ = comp;
  this->enabled_ = enabled;
  this->start_universe_ = start_universe;
  this->universe_count_ = universe_count;
  if (enabled)
    this->open_socket_();
  ESP_LOGCONFIG(TAG, "Art-Net receiver on port %u: %s (universe %u-%u)", WLED_ARTNET_PORT,
                enabled ? "enabled" : "disabled", start_universe, start_universe + universe_count - 1);
}

void WLEDArtNetReceiver::set_enabled(bool enabled) {
  this->enabled_ = enabled;
  if (enabled)
    this->open_socket_();
  else
    this->close_socket_();
}

void WLEDArtNetReceiver::open_socket_() {
  if (this->fd_ >= 0)
    return;
  int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0) {
    ESP_LOGW(TAG, "Art-Net: could not create socket: %d", errno);
    return;
  }
  int enable = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
  int flags = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, flags | O_NONBLOCK);

  struct sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(WLED_ARTNET_PORT);
  if (bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    ESP_LOGW(TAG, "Art-Net: bind failed on port %u: %d", WLED_ARTNET_PORT, errno);
    close(fd);
    return;
  }
  this->fd_ = fd;
}

void WLEDArtNetReceiver::close_socket_() {
  if (this->fd_ >= 0) {
    close(this->fd_);
    this->fd_ = -1;
  }
  this->receiving_ = false;
  this->source_ip_ = 0;
}

void WLEDArtNetReceiver::loop() {
  if (!this->enabled_ || this->fd_ < 0)
    return;

  uint8_t buf[600];
  struct sockaddr_in src = {};
  socklen_t src_len = sizeof(src);

  for (int i = 0; i < 8; i++) {
    ssize_t len = recvfrom(this->fd_, buf, sizeof(buf), 0, reinterpret_cast<struct sockaddr *>(&src), &src_len);
    if (len <= 0)
      break;
    if (static_cast<size_t>(len) >= ARTNET_MIN_PACKET)
      this->process_packet_(buf, static_cast<size_t>(len), src.sin_addr.s_addr);
  }

  if (this->receiving_ && (millis() - this->last_receive_ms_ > ARTNET_TIMEOUT_MS)) {
    this->receiving_ = false;
    this->source_ip_ = 0;
    this->comp_->clear_pixel_overrides();
    ESP_LOGD(TAG, "Art-Net: stream timeout, returning to effects");
  }
}

void WLEDArtNetReceiver::process_packet_(const uint8_t *buf, size_t len, uint32_t source_ip) {
  if (buf == nullptr || len < ARTNET_MIN_PACKET)
    return;
  if (memcmp(buf, ARTNET_ID, sizeof(ARTNET_ID)) != 0)
    return;

  uint16_t opcode = static_cast<uint16_t>(buf[8]) | (static_cast<uint16_t>(buf[9]) << 8);
  if (opcode != ARTNET_OPCODE_DMX)
    return;

  uint16_t protocol_version = (static_cast<uint16_t>(buf[10]) << 8) | buf[11];
  if (protocol_version < 14)
    return;

  uint16_t universe = static_cast<uint16_t>(buf[14]) | (static_cast<uint16_t>(buf[15]) << 8);
  if (universe < this->start_universe_ || universe >= this->start_universe_ + this->universe_count_)
    return;
  if (!realtime_source_allowed("Art-Net", source_ip, this->receiving_, &this->source_ip_,
                               &this->last_source_reject_log_ms_))
    return;

  uint8_t uni_index = static_cast<uint8_t>(universe - this->start_universe_);
  uint8_t seq = buf[12];
  if (seq != 0 && uni_index < 8) {
    uint8_t last = this->last_seq_[uni_index];
    if (last != 0) {
      int8_t diff = static_cast<int8_t>(seq - last);
      if (diff < 0 && diff > -20)
        return;
    }
    this->last_seq_[uni_index] = seq;
  }

  uint16_t data_len = (static_cast<uint16_t>(buf[16]) << 8) | buf[17];
  if (data_len > 512)
    data_len = 512;
  if (ARTNET_DMX_DATA_OFFSET + data_len > len)
    data_len = static_cast<uint16_t>(len - ARTNET_DMX_DATA_OFFSET);

  const uint8_t *dmx = buf + ARTNET_DMX_DATA_OFFSET;
  uint32_t led_count = this->comp_->get_led_count();
  uint32_t start_led = static_cast<uint32_t>(uni_index) * ARTNET_PIXELS_PER_UNIVERSE;
  uint32_t pixel_count = static_cast<uint32_t>(data_len / 3);

  bool changed = false;
  for (uint32_t i = 0; i < pixel_count && (start_led + i) < led_count; i++) {
    uint8_t r = dmx[i * 3];
    uint8_t g = dmx[i * 3 + 1];
    uint8_t b = dmx[i * 3 + 2];
    changed |= this->comp_->set_pixel_override(0, start_led + i, RGBW32(r, g, b, 0), false);
  }
  if (changed)
    this->comp_->mark_pixel_overrides_changed();

  this->last_receive_ms_ = millis();
  if (!this->receiving_) {
    ESP_LOGD(TAG, "Art-Net: receiving on universe %u (%u pixels)", universe, pixel_count);
    this->receiving_ = true;
  }
}

}  // namespace wled_bridge
}  // namespace esphome

#endif  // USE_ESP32 && WLED_BRIDGE_REALTIME

#ifdef USE_ESP32
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

namespace esphome {
namespace wled_bridge {

static const char *const TAG = "wled_bridge.udp";

static constexpr uint8_t WLED_NOTIFIER_PROTOCOL = 0;
static constexpr uint8_t WLED_NOTIFIER_COMPAT_VERSION = 12;
static constexpr uint8_t CALL_MODE_DIRECT_CHANGE = 1;
static constexpr size_t UDP_SEG_SIZE = 36;
static constexpr size_t SEG_OFFSET = 41;
static constexpr size_t NOTIFIER_PACKET_SIZE = SEG_OFFSET + WLED_MAX_SEGMENTS * UDP_SEG_SIZE;

// Minimum interval between outgoing notifications (ms), to avoid flooding.
static constexpr uint32_t SEND_THROTTLE_MS = 50;

void WLEDUdpSync::setup(WLEDBridgeComponent *comp, uint16_t port, uint16_t port2, bool do_send, bool do_receive) {
  this->comp_ = comp;
  this->port_ = port;
  this->port2_ = port2;
  this->do_send_ = do_send;
  this->do_receive_ = do_receive;

  if (do_send)
    this->open_send_socket_();
  if (do_receive) {
    this->open_recv_socket_(&this->recv_fd_, this->port_);
    if (this->port2_ != 0 && this->port2_ != this->port_)
      this->open_recv_socket_(&this->recv2_fd_, this->port2_);
  }

  ESP_LOGCONFIG(TAG, "UDP sync on ports %u/%u send=%s recv=%s", port, port2, do_send ? "yes" : "no",
                do_receive ? "yes" : "no");
}

void WLEDUdpSync::set_ports(uint16_t port, uint16_t port2) {
  bool changed = this->port_ != port || this->port2_ != port2;
  this->port_ = port;
  this->port2_ = port2;
  if (!changed || !this->do_receive_)
    return;
  this->close_recv_socket_(&this->recv_fd_);
  this->close_recv_socket_(&this->recv2_fd_);
  this->open_recv_socket_(&this->recv_fd_, this->port_);
  if (this->port2_ != 0 && this->port2_ != this->port_)
    this->open_recv_socket_(&this->recv2_fd_, this->port2_);
}

void WLEDUdpSync::set_send_enabled(bool enabled) {
  this->do_send_ = enabled;
  if (enabled && this->send_fd_ < 0)
    this->open_send_socket_();
  if (!enabled)
    this->close_send_socket_();
}

void WLEDUdpSync::set_receive_enabled(bool enabled) {
  this->do_receive_ = enabled;
  if (enabled) {
    this->open_recv_socket_(&this->recv_fd_, this->port_);
    if (this->port2_ != 0 && this->port2_ != this->port_)
      this->open_recv_socket_(&this->recv2_fd_, this->port2_);
  }
  if (!enabled) {
    this->close_recv_socket_(&this->recv_fd_);
    this->close_recv_socket_(&this->recv2_fd_);
  }
}

void WLEDUdpSync::open_send_socket_() {
  if (this->send_fd_ >= 0)
    return;
  int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0) {
    ESP_LOGW(TAG, "Could not create send socket: %d", errno);
    return;
  }
  int enable = 1;
  setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &enable, sizeof(enable));
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
  this->send_fd_ = fd;
}

void WLEDUdpSync::open_recv_socket_(int *target_fd, uint16_t port) {
  if (target_fd == nullptr || *target_fd >= 0)
    return;
  int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0) {
    ESP_LOGW(TAG, "Could not create recv socket: %d", errno);
    return;
  }
  int enable = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));

  // Non-blocking
  int flags = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, flags | O_NONBLOCK);

  struct sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(port);
  if (bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    ESP_LOGW(TAG, "Bind failed on port %u: %d", port, errno);
    close(fd);
    return;
  }
  *target_fd = fd;
}

void WLEDUdpSync::close_send_socket_() {
  if (this->send_fd_ >= 0) {
    close(this->send_fd_);
    this->send_fd_ = -1;
  }
}

void WLEDUdpSync::close_recv_socket_(int *target_fd) {
  if (target_fd != nullptr && *target_fd >= 0) {
    close(*target_fd);
    *target_fd = -1;
  }
}

void WLEDUdpSync::loop() {
  if (!this->do_receive_)
    return;

  this->drain_socket_(this->recv_fd_);
  this->drain_socket_(this->recv2_fd_);
}

void WLEDUdpSync::drain_socket_(int fd) {
  if (fd < 0)
    return;

  uint8_t buf[1472];
  struct sockaddr_in src = {};
  socklen_t src_len = sizeof(src);

  // Drain all available packets this loop tick
  for (int i = 0; i < 8; i++) {
    ssize_t len = recvfrom(fd, buf, sizeof(buf), 0, reinterpret_cast<struct sockaddr *>(&src), &src_len);
    if (len <= 0)
      break;
    this->apply_packet_(buf, static_cast<size_t>(len));
  }
}

void WLEDUdpSync::send_notification() {
  if (!this->do_send_ || this->send_fd_ < 0 || this->comp_ == nullptr)
    return;
  if (!this->comp_->get_udp_notify_direct())
    return;

  uint32_t now = millis();
  if (now - this->last_send_ms_ < SEND_THROTTLE_MS)
    return;
  this->last_send_ms_ = now;

  if (this->comp_->get_udp_sync_groups() == 0)
    return;

  uint8_t buf[NOTIFIER_PACKET_SIZE] = {};
  buf[0] = WLED_NOTIFIER_PROTOCOL;
  buf[1] = CALL_MODE_DIRECT_CHANGE;
  buf[2] = this->comp_->get_brightness();

  uint32_t col0 = this->comp_->get_params().colors[0];
  uint32_t col1 = this->comp_->get_params().colors[1];
  uint32_t col2 = this->comp_->get_params().colors[2];

  buf[3] = R(col0);
  buf[4] = G(col0);
  buf[5] = B(col0);
  buf[6] = this->comp_->is_nightlight_active() ? 1 : 0;
  buf[7] = this->comp_->get_nightlight_duration_min();
  buf[8] = this->comp_->get_effect_index();
  buf[9] = this->comp_->get_params().speed;
  buf[10] = W(col0);
  buf[11] = WLED_NOTIFIER_COMPAT_VERSION;
  buf[12] = R(col1);
  buf[13] = G(col1);
  buf[14] = B(col1);
  buf[15] = W(col1);
  buf[16] = this->comp_->get_params().intensity;
  uint16_t transition = this->comp_->get_transition_ms();
  buf[17] = transition & 0xFF;
  buf[18] = (transition >> 8) & 0xFF;
  buf[19] = this->comp_->get_params().palette_id;
  buf[20] = R(col2);
  buf[21] = G(col2);
  buf[22] = B(col2);
  buf[23] = W(col2);

  uint32_t t = millis();
  buf[25] = (t >> 24) & 0xFF;
  buf[26] = (t >> 16) & 0xFF;
  buf[27] = (t >> 8) & 0xFF;
  buf[28] = t & 0xFF;
  buf[36] = this->comp_->get_udp_sync_groups();
  buf[37] = 255;  // no CCT/Kelvin payload

  uint8_t sent_segments = 0;
  for (uint8_t id = 0; id < this->comp_->get_segment_count() && sent_segments < WLED_MAX_SEGMENTS; id++) {
    WLEDBridgeComponent::SegmentReadView view;
    if (!this->comp_->get_segment_view(id, view) || view.stop <= view.start)
      continue;
    size_t ofs = SEG_OFFSET + sent_segments * UDP_SEG_SIZE;
    buf[ofs + 0] = id;
    buf[ofs + 1] = view.start >> 8;
    buf[ofs + 2] = view.start & 0xFF;
    buf[ofs + 3] = view.stop >> 8;
    buf[ofs + 4] = view.stop & 0xFF;
    buf[ofs + 5] = view.grouping > 255 ? 255 : static_cast<uint8_t>(view.grouping);
    buf[ofs + 6] = view.spacing > 255 ? 255 : static_cast<uint8_t>(view.spacing);
    uint8_t options = 0;
    if (view.selected)
      options |= 0x01;
    if (view.reverse)
      options |= 0x02;
    if (view.on)
      options |= 0x04;
    if (view.mirror)
      options |= 0x08;
    buf[ofs + 9] = options;
    buf[ofs + 10] = view.opacity;
    buf[ofs + 11] = view.mode;
    buf[ofs + 12] = view.speed;
    buf[ofs + 13] = view.intensity;
    buf[ofs + 14] = view.palette;
    for (uint8_t slot = 0; slot < 3; slot++) {
      uint32_t color = view.colors[slot];
      size_t c_ofs = ofs + 15 + slot * 4;
      buf[c_ofs + 0] = R(color);
      buf[c_ofs + 1] = G(color);
      buf[c_ofs + 2] = B(color);
      buf[c_ofs + 3] = W(color);
    }
    uint8_t checks =
        (view.custom3 & 0x1F) | (view.check1 ? 0x20 : 0) | (view.check2 ? 0x40 : 0) | (view.check3 ? 0x80 : 0);
    buf[ofs + 28] = 0;
    buf[ofs + 29] = view.custom1;
    buf[ofs + 30] = view.custom2;
    buf[ofs + 31] = checks;
    buf[ofs + 32] = 0;
    buf[ofs + 33] = 0;
    buf[ofs + 34] = 0;
    buf[ofs + 35] = 1;
    sent_segments++;
  }
  buf[39] = sent_segments;
  buf[40] = UDP_SEG_SIZE;

  struct sockaddr_in dest = {};
  dest.sin_family = AF_INET;
  dest.sin_addr.s_addr = htonl(INADDR_BROADCAST);
  dest.sin_port = htons(this->port_);

  size_t packet_size = SEG_OFFSET + static_cast<size_t>(sent_segments) * UDP_SEG_SIZE;
  uint16_t attempts = static_cast<uint16_t>(this->comp_->get_udp_retries()) + 1u;
  for (uint16_t i = 0; i < attempts; i++)
    sendto(this->send_fd_, buf, packet_size, 0, reinterpret_cast<struct sockaddr *>(&dest), sizeof(dest));
}

void WLEDUdpSync::apply_packet_(const uint8_t *buf, size_t len) {
  if (len < 11 || buf[0] != WLED_NOTIFIER_PROTOCOL)
    return;

  uint8_t call_mode = buf[1];
  if (call_mode == 0)
    return;

  uint8_t version = len > 11 ? buf[11] : 0;
  if (version > 8 && len > 36) {
    uint8_t groups = buf[36];
    if ((this->comp_->get_udp_receive_groups() & groups) == 0)
      return;
  } else if ((this->comp_->get_udp_receive_groups() & 0x01) == 0) {
    return;
  }

  uint8_t bri = buf[2];
  uint8_t fx = len > 8 ? buf[8] : this->comp_->get_effect_index();
  uint8_t spd = len > 9 ? buf[9] : this->comp_->get_params().speed;
  uint8_t ix = version > 2 && len > 16 ? buf[16] : this->comp_->get_params().intensity;
  uint8_t pal = version > 4 && len > 19 ? buf[19] : this->comp_->get_params().palette_id;

  uint32_t col0 = RGBW32(buf[3], buf[4], buf[5], version > 0 && len > 10 ? buf[10] : 0);
  uint32_t col1 =
      version > 1 && len > 15 ? RGBW32(buf[12], buf[13], buf[14], buf[15]) : this->comp_->get_params().colors[1];
  uint32_t col2 =
      version > 6 && len > 23 ? RGBW32(buf[20], buf[21], buf[22], buf[23]) : this->comp_->get_params().colors[2];

#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE
  ESP_LOGV(TAG, "UDP recv: fx=%u bri=%u pal=%u spd=%u ix=%u compat=%u", fx, bri, pal, spd, ix, version);
#endif

  if (this->comp_->get_udp_receive_brightness())
    this->comp_->set_brightness(bri);
  if (this->comp_->get_udp_receive_effects()) {
    this->comp_->set_effect(fx);
    this->comp_->set_speed(spd);
    this->comp_->set_intensity(ix);
  }
  if (this->comp_->get_udp_receive_palette())
    this->comp_->set_palette(pal);
  if (this->comp_->get_udp_receive_color()) {
    this->comp_->set_color(0, col0);
    this->comp_->set_color(1, col1);
    this->comp_->set_color(2, col2);
  }

  if (version > 3 && len > 18) {
    uint16_t transition = static_cast<uint16_t>(buf[17]) | (static_cast<uint16_t>(buf[18]) << 8);
    this->comp_->set_transition(transition);
  }

  bool receive_segment_payload = this->comp_->get_udp_receive_segments() ||
                                 this->comp_->get_udp_receive_segment_options() ||
                                 this->comp_->get_udp_receive_effects() || this->comp_->get_udp_receive_palette() ||
                                 this->comp_->get_udp_receive_color();
  if (receive_segment_payload && version > 10 && len > 40) {
    uint8_t count = buf[39];
    uint8_t seg_size = buf[40];
    if (seg_size >= UDP_SEG_SIZE) {
      for (uint8_t i = 0; i < count && i < WLED_MAX_SEGMENTS; i++) {
        size_t ofs = SEG_OFFSET + static_cast<size_t>(i) * seg_size;
        if (ofs + UDP_SEG_SIZE > len)
          break;
        uint8_t id = buf[ofs + 0];
        uint16_t start = (static_cast<uint16_t>(buf[ofs + 1]) << 8) | buf[ofs + 2];
        uint16_t stop = (static_cast<uint16_t>(buf[ofs + 3]) << 8) | buf[ofs + 4];
        if (this->comp_->get_udp_receive_segments())
          this->comp_->segment_set_bounds(id, start, stop);
        if (this->comp_->get_udp_receive_segment_options()) {
          this->comp_->segment_set_grouping(id, buf[ofs + 5], buf[ofs + 6]);
          uint8_t options = buf[ofs + 9];
          this->comp_->segment_set_reverse(id, (options & 0x02) != 0);
          this->comp_->segment_set_on(id, (options & 0x04) != 0);
          this->comp_->segment_set_mirror(id, (options & 0x08) != 0);
          this->comp_->segment_set_opacity(id, buf[ofs + 10]);
        }
        if (this->comp_->get_udp_receive_effects()) {
          this->comp_->segment_set_effect(id, buf[ofs + 11]);
          this->comp_->segment_set_speed(id, buf[ofs + 12]);
          this->comp_->segment_set_intensity(id, buf[ofs + 13]);
        }
        if (this->comp_->get_udp_receive_palette())
          this->comp_->segment_set_palette(id, buf[ofs + 14]);
        if (this->comp_->get_udp_receive_color()) {
          for (uint8_t slot = 0; slot < 3; slot++) {
            size_t c_ofs = ofs + 15 + slot * 4;
            this->comp_->segment_set_color(id, slot,
                                           RGBW32(buf[c_ofs + 0], buf[c_ofs + 1], buf[c_ofs + 2], buf[c_ofs + 3]));
          }
        }
        if (this->comp_->get_udp_receive_effects() && version > 11) {
          uint8_t checks = buf[ofs + 31];
          this->comp_->segment_set_custom(id, 1, buf[ofs + 29]);
          this->comp_->segment_set_custom(id, 2, buf[ofs + 30]);
          this->comp_->segment_set_custom(id, 3, checks & 0x1F);
          this->comp_->segment_set_check(id, 1, (checks & 0x20) != 0);
          this->comp_->segment_set_check(id, 2, (checks & 0x40) != 0);
          this->comp_->segment_set_check(id, 3, (checks & 0x80) != 0);
        }
      }
    }
  }

  // Incoming UDP updates should update HA state but should not be immediately
  // echoed back as a fresh notifier packet.
  this->last_send_ms_ = millis();
  this->comp_->publish_light_state();
}

}  // namespace wled_bridge
}  // namespace esphome
#endif  // USE_ESP32

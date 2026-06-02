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

static constexpr uint8_t WLED_NOTIFIER_VERSION = 2;
static constexpr size_t NOTIFIER_PACKET_SIZE = 23;

// Minimum interval between outgoing notifications (ms), to avoid flooding.
static constexpr uint32_t SEND_THROTTLE_MS = 50;

void WLEDUdpSync::setup(WLEDBridgeComponent *comp, uint16_t port, bool do_send, bool do_receive) {
  this->comp_ = comp;
  this->port_ = port;
  this->do_send_ = do_send;
  this->do_receive_ = do_receive;

  if (do_send)
    this->open_send_socket_();
  if (do_receive)
    this->open_recv_socket_();

  ESP_LOGCONFIG(TAG, "UDP sync on port %u send=%s recv=%s", port, do_send ? "yes" : "no", do_receive ? "yes" : "no");
}

void WLEDUdpSync::open_send_socket_() {
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

void WLEDUdpSync::open_recv_socket_() {
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
  addr.sin_port = htons(this->port_);
  if (bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    ESP_LOGW(TAG, "Bind failed on port %u: %d", this->port_, errno);
    close(fd);
    return;
  }
  this->recv_fd_ = fd;
}

void WLEDUdpSync::loop() {
  if (!this->do_receive_ || this->recv_fd_ < 0)
    return;

  uint8_t buf[64];
  struct sockaddr_in src = {};
  socklen_t src_len = sizeof(src);

  // Drain all available packets this loop tick
  for (int i = 0; i < 8; i++) {
    ssize_t len = recvfrom(this->recv_fd_, buf, sizeof(buf), 0, reinterpret_cast<struct sockaddr *>(&src), &src_len);
    if (len <= 0)
      break;
    this->apply_packet_(buf, static_cast<size_t>(len));
  }
}

void WLEDUdpSync::send_notification() {
  if (!this->do_send_ || this->send_fd_ < 0 || this->comp_ == nullptr)
    return;

  uint32_t now = millis();
  if (now - this->last_send_ms_ < SEND_THROTTLE_MS)
    return;
  this->last_send_ms_ = now;

  uint8_t buf[NOTIFIER_PACKET_SIZE] = {};
  buf[0] = WLED_NOTIFIER_VERSION;
  buf[1] = 1;  // callMode = CALL_MODE_SET_VALUE
  buf[2] = this->comp_->get_effect_index();

  uint32_t col0 = this->comp_->get_params().colors[0];
  uint32_t col1 = this->comp_->get_params().colors[1];
  uint32_t col2 = this->comp_->get_params().colors[2];

  buf[3] = R(col0);
  buf[4] = G(col0);
  buf[5] = B(col0);
  buf[6] = this->comp_->get_brightness();
  buf[7] = this->comp_->is_nightlight_active() ? 1 : 0;
  buf[8] = 0;
  buf[9] = this->comp_->get_params().palette_id;
  buf[10] = R(col1);
  buf[11] = G(col1);
  buf[12] = B(col1);
  buf[13] = R(col2);
  buf[14] = G(col2);
  buf[15] = B(col2);
  buf[16] = this->comp_->get_params().speed;
  buf[17] = this->comp_->get_params().intensity;
  buf[18] = W(col0);
  buf[19] = W(col1);
  buf[20] = W(col2);
  buf[21] = this->comp_->get_params().custom1;
  buf[22] = this->comp_->get_params().custom2;

  struct sockaddr_in dest = {};
  dest.sin_family = AF_INET;
  dest.sin_addr.s_addr = htonl(INADDR_BROADCAST);
  dest.sin_port = htons(this->port_);

  sendto(this->send_fd_, buf, NOTIFIER_PACKET_SIZE, 0, reinterpret_cast<struct sockaddr *>(&dest), sizeof(dest));
}

void WLEDUdpSync::apply_packet_(const uint8_t *buf, size_t len) {
  if (len < NOTIFIER_PACKET_SIZE || buf[0] != WLED_NOTIFIER_VERSION)
    return;

  uint8_t call_mode = buf[1];
  // Skip packets originating from our own send (callMode 1 = SET_VALUE we just sent).
  // Accept callMode 4 (CALL_MODE_NOTIFICATION from another WLED) and others.
  if (call_mode == 1 && this->do_send_)
    return;

  uint8_t fx = buf[2];
  uint8_t bri = buf[6];
  uint8_t pal = buf[9];
  uint8_t spd = buf[16];
  uint8_t ix = buf[17];
  uint8_t c1 = buf[21];
  uint8_t c2 = buf[22];

  uint32_t col0 = RGBW32(buf[3], buf[4], buf[5], buf[18]);
  uint32_t col1 = RGBW32(buf[10], buf[11], buf[12], buf[19]);
  uint32_t col2 = RGBW32(buf[13], buf[14], buf[15], buf[20]);

  ESP_LOGV(TAG, "UDP recv: fx=%u bri=%u pal=%u spd=%u ix=%u", fx, bri, pal, spd, ix);

  this->comp_->set_brightness(bri);
  this->comp_->set_effect(fx);
  this->comp_->set_palette(pal);
  this->comp_->set_speed(spd);
  this->comp_->set_intensity(ix);
  this->comp_->set_custom1(c1);
  this->comp_->set_custom2(c2);
  this->comp_->set_color(0, col0);
  this->comp_->set_color(1, col1);
  this->comp_->set_color(2, col2);
}

}  // namespace wled_bridge
}  // namespace esphome
#endif  // USE_ESP32

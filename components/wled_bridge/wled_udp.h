#pragma once
#ifdef USE_ESP32

#include <stdint.h>
#include <stddef.h>

namespace esphome {
namespace wled_bridge {

class WLEDBridgeComponent;

// WLED UDP Notifier sync (port 21324, WLED protocol 0 / compatibility v12).
// Enables this device to participate in a WLED network:
//   send=true  — broadcast a notifier packet whenever local state changes
//   receive=true — apply state from incoming notifier packets
class WLEDUdpSync {
 public:
  void setup(WLEDBridgeComponent *comp, uint16_t port, uint16_t port2, bool do_send, bool do_receive);
  void set_ports(uint16_t port, uint16_t port2);
  void set_send_enabled(bool enabled);
  void set_receive_enabled(bool enabled);
  void loop();
  void send_notification();

 protected:
  void open_send_socket_();
  void open_recv_socket_(int *target_fd, uint16_t port);
  void close_send_socket_();
  void close_recv_socket_(int *target_fd);
  void drain_socket_(int fd);
  void apply_packet_(const uint8_t *buf, size_t len);

  WLEDBridgeComponent *comp_{nullptr};
  uint16_t port_{21324};
  uint16_t port2_{65506};
  bool do_send_{false};
  bool do_receive_{false};
  int send_fd_{-1};
  int recv_fd_{-1};
  int recv2_fd_{-1};
  uint32_t last_send_ms_{0};
};

}  // namespace wled_bridge
}  // namespace esphome

#endif  // USE_ESP32

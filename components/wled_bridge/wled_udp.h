#pragma once
#ifdef USE_ESP32

#include <stdint.h>
#include <stddef.h>

namespace esphome {
namespace wled_bridge {

class WLEDBridgeComponent;

// WLED UDP Notifier sync (port 21324, protocol v2).
// Enables this device to participate in a WLED network:
//   send=true  — broadcast a notifier packet whenever local state changes
//   receive=true — apply state from incoming notifier packets
//
// Notifier packet layout (23 bytes, WLED protocol v2):
//   [0]     version (2)
//   [1]     callMode
//   [2]     effect FX
//   [3-5]   RGB of color 1
//   [6]     brightness
//   [7]     nightlight active
//   [8]     unused
//   [9]     palette
//   [10-12] RGB of color 2
//   [13-15] RGB of color 3
//   [16]    speed
//   [17]    intensity
//   [18-20] W channels (color 1, 2, 3)
//   [21]    custom1
//   [22]    custom2
class WLEDUdpSync {
 public:
  void setup(WLEDBridgeComponent *comp, uint16_t port, bool do_send, bool do_receive);
  void loop();
  void send_notification();

 protected:
  void open_send_socket_();
  void open_recv_socket_();
  void apply_packet_(const uint8_t *buf, size_t len);

  WLEDBridgeComponent *comp_{nullptr};
  uint16_t port_{21324};
  bool do_send_{false};
  bool do_receive_{false};
  int send_fd_{-1};
  int recv_fd_{-1};
  uint32_t last_send_ms_{0};
};

}  // namespace wled_bridge
}  // namespace esphome

#endif  // USE_ESP32

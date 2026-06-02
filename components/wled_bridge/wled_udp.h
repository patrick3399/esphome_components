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

// DDP (Distributed Display Protocol) realtime pixel receiver.
// Listens on UDP 4048 for pixel data from Hyperion / Prismatik / xLights.
// Writes directly to the bridge's pixel override buffer; when the push flag
// arrives, the bridge renders the frame immediately.
class WLEDDdpReceiver {
 public:
  void setup(WLEDBridgeComponent *comp, bool enabled);
  void set_enabled(bool enabled);
  bool is_enabled() const {
    return this->enabled_;
  }
  bool is_receiving() const {
    return this->receiving_;
  }
  void loop();

 protected:
  void open_socket_();
  void close_socket_();
  void process_packet_(const uint8_t *buf, size_t len);

  WLEDBridgeComponent *comp_{nullptr};
  bool enabled_{false};
  bool receiving_{false};
  int fd_{-1};
  uint32_t last_receive_ms_{0};
  bool push_seen_{false};
};

// E1.31 (sACN / Streaming ACN) realtime pixel receiver.
// Listens on UDP 5568 for multi-universe DMX data from xLights / QLC+ / sACN sources.
// Each universe carries 512 channels (170 RGB pixels). Multiple universes are
// mapped sequentially to the LED strip starting from a configurable start universe.
class WLEDE131Receiver {
 public:
  void setup(WLEDBridgeComponent *comp, bool enabled, uint16_t start_universe, uint8_t universe_count);
  void set_enabled(bool enabled);
  bool is_enabled() const {
    return this->enabled_;
  }
  bool is_receiving() const {
    return this->receiving_;
  }
  void loop();

 protected:
  void open_socket_();
  void close_socket_();
  void process_packet_(const uint8_t *buf, size_t len);

  WLEDBridgeComponent *comp_{nullptr};
  bool enabled_{false};
  bool receiving_{false};
  int fd_{-1};
  uint16_t start_universe_{1};
  uint8_t universe_count_{1};
  uint32_t last_receive_ms_{0};
  uint8_t last_seq_[8]{};
};

}  // namespace wled_bridge
}  // namespace esphome

#endif  // USE_ESP32

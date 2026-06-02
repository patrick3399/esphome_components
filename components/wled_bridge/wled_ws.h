#pragma once
#ifdef USE_ESP32

#include <esp_http_server.h>
#include <vector>
#include <string>

namespace esphome {
namespace wled_bridge {

class WLEDBridgeComponent;
class WLEDJsonHandler;

// WebSocket handler for WLED's /ws endpoint.
// Uses IDF 5.x native WebSocket support — bypasses the ESPHome AsyncWebHandler
// shim so we can keep connections alive and push state changes.
class WLEDWsHandler {
 public:
  WLEDWsHandler(WLEDBridgeComponent *comp, WLEDJsonHandler *json_handler, httpd_handle_t server);

  // Register the /ws URI handler on the httpd server. Called once from setup().
  void register_handler();

  // Called from WLEDBridgeComponent::loop() when state changes.
  void broadcast_state();

  bool has_clients() const {
    return !this->client_fds_.empty();
  }
  size_t client_count() const {
    return this->client_fds_.size();
  }

 protected:
  static esp_err_t ws_handler_(httpd_req_t *req);

  void on_client_connect_(int fd);
  void send_to_client_(int fd, const std::string &json);

  WLEDBridgeComponent *comp_;
  WLEDJsonHandler *json_handler_;
  httpd_handle_t server_;
  std::vector<int> client_fds_;
};

}  // namespace wled_bridge
}  // namespace esphome

#endif  // USE_ESP32

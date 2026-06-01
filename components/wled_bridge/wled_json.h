#pragma once
#include "esphome/core/defines.h"

#ifdef USE_ESP32
#include "esphome/components/web_server_idf/web_server_idf.h"
#else
#include <ESPAsyncWebServer.h>
#endif

#include <string>
#include <vector>

namespace esphome {
namespace wled_bridge {

class WLEDBridgeComponent;  // forward

// ---------- JSON request handler ----------
// Handles GET/POST for WLED-style JSON and legacy HTTP endpoints.
class WLEDJsonHandler : public web_server_idf::AsyncWebHandler {
 public:
  explicit WLEDJsonHandler(WLEDBridgeComponent *comp) : comp_(comp) {}

  bool canHandle(web_server_idf::AsyncWebServerRequest *request) const override;
  void handleRequest(web_server_idf::AsyncWebServerRequest *request) override;
  void handleBody(web_server_idf::AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index,
                  size_t total) override;

 protected:
  void handle_get_state_(web_server_idf::AsyncWebServerRequest *request);
  void handle_get_info_(web_server_idf::AsyncWebServerRequest *request);
  void handle_get_effects_(web_server_idf::AsyncWebServerRequest *request);
  void handle_get_palettes_(web_server_idf::AsyncWebServerRequest *request);
  void handle_get_config_(web_server_idf::AsyncWebServerRequest *request);
  void handle_get_network_(web_server_idf::AsyncWebServerRequest *request);
  void handle_get_pins_(web_server_idf::AsyncWebServerRequest *request);
  void handle_get_presets_(web_server_idf::AsyncWebServerRequest *request);
  void handle_post_presets_(web_server_idf::AsyncWebServerRequest *request, const std::string &body);
  void handle_win_(web_server_idf::AsyncWebServerRequest *request, const std::string &url);
  void handle_post_state_(web_server_idf::AsyncWebServerRequest *request, const std::string &body);

  WLEDBridgeComponent *comp_;
  std::string post_body_;
};

// ---------- Live state snapshot handler ----------
// Provides a versioned polling endpoint for UI state sync. Full websocket/SSE
// streaming remains a later integration point for the ESPHome IDF web stack.
class WLEDSseHandler : public web_server_idf::AsyncWebHandler {
 public:
  explicit WLEDSseHandler(WLEDBridgeComponent *comp) : comp_(comp) {}

  bool canHandle(web_server_idf::AsyncWebServerRequest *request) const override;
  void handleRequest(web_server_idf::AsyncWebServerRequest *request) override;

  // Called from WLEDBridgeComponent::loop() when state changes.
  void broadcast_state();

 protected:
  WLEDBridgeComponent *comp_;
  std::string pending_state_;
  uint32_t pending_version_{0};
  bool has_pending_{false};
};

// ---------- UI handler ----------
// Serves the WLED web interface at / and /index.htm
class WLEDUiHandler : public web_server_idf::AsyncWebHandler {
 public:
  bool canHandle(web_server_idf::AsyncWebServerRequest *request) const override;
  void handleRequest(web_server_idf::AsyncWebServerRequest *request) override;
};

// ---------- JSON builder helpers (non-member) ----------
std::string build_state_json(const WLEDBridgeComponent *comp);
std::string build_info_json(const WLEDBridgeComponent *comp);
std::string build_effects_json();
std::string build_fxdata_json();
std::string build_palettes_json();
std::string build_config_json(const WLEDBridgeComponent *comp);
std::string build_network_json(const WLEDBridgeComponent *comp);
std::string build_pins_json();
std::string build_presets_json(const WLEDBridgeComponent *comp);
std::string build_live_json(const WLEDBridgeComponent *comp);

}  // namespace wled_bridge
}  // namespace esphome

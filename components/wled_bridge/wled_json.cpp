#include "wled_json.h"
#include "wled_ui_data.h"
#include "wled_bridge.h"
#include "wled_effects.h"
#include "wled_palette.h"
#include "esphome/core/log.h"
#include "esphome/components/json/json_util.h"
#include <stdio.h>
#include <string.h>

namespace esphome {
namespace wled_bridge {

static const char *const TAG = "wled_bridge.json";

// ============================================================
// JSON builder helpers
// ============================================================
std::string build_state_json(const WLEDBridgeComponent *c) {
  const EffectParams &p = c->get_params();
  char buf[1024];
  snprintf(buf, sizeof(buf),
           "{"
           "\"on\":%s,"
           "\"bri\":%u,"
           "\"transition\":7,"
           "\"ps\":-1,"
           "\"pl\":-1,"
           "\"nl\":{\"on\":false,\"dur\":60,\"mode\":1,\"tbri\":0,\"rem\":-1},"
           "\"udpn\":{\"send\":false,\"recv\":false,\"sgrp\":1,\"rgrp\":1},"
           "\"lor\":0,"
           "\"mainseg\":0,"
           "\"seg\":[{"
           "\"id\":0,"
           "\"start\":0,"
           "\"stop\":%u,"
           "\"len\":%u,"
           "\"grp\":1,"
           "\"spc\":0,"
           "\"of\":0,"
           "\"on\":%s,"
           "\"frz\":false,"
           "\"bri\":255,"
           "\"cct\":127,"
           "\"set\":0,"
           "\"col\":[[%u,%u,%u],[%u,%u,%u],[%u,%u,%u]],"
           "\"fx\":%u,"
           "\"sx\":%u,"
           "\"ix\":%u,"
           "\"pal\":%u,"
           "\"c1\":%u,"
           "\"c2\":%u,"
           "\"c3\":%u,"
           "\"sel\":true,"
           "\"rev\":false,"
           "\"mi\":false"
           "}]}",
           c->is_on() ? "true" : "false", c->get_brightness(), c->get_led_count(), c->get_led_count(),
           c->is_on() ? "true" : "false", R(p.colors[0]), G(p.colors[0]), B(p.colors[0]), R(p.colors[1]),
           G(p.colors[1]), B(p.colors[1]), R(p.colors[2]), G(p.colors[2]), B(p.colors[2]), c->get_effect_index(),
           p.speed, p.intensity, p.palette_id, p.custom1, p.custom2, p.custom3);
  return std::string(buf);
}

std::string build_info_json(const WLEDBridgeComponent *c) {
  char buf[512];
  snprintf(buf, sizeof(buf),
           "{"
           "\"ver\":\"0.1.0\","
           "\"vid\":2605010,"
           "\"leds\":{"
           "\"count\":%u,"
           "\"pwr\":%u,"
           "\"fps\":%u,"
           "\"maxpwr\":%u,"
           "\"maxseg\":1,"
           "\"cco\":0"
           "},"
           "\"str\":false,"
           "\"name\":\"WLED Bridge\","
           "\"udp\":0,"
           "\"live\":false,"
           "\"lm\":\"\","
           "\"lip\":\"\","
           "\"ws\":0,"
           "\"fxcount\":%zu,"
           "\"palcount\":%zu,"
           "\"wifi\":{"
           "\"bssid\":\"\","
           "\"rssi\":-50,"
           "\"signal\":90,"
           "\"channel\":1"
           "},"
           "\"arch\":\"esp32s3\","
           "\"core\":\"idf\","
           "\"freeheap\":%u,"
           "\"uptime\":%u,"
           "\"opt\":511,"
           "\"brand\":\"ESPHome\","
           "\"product\":\"WLED Bridge\","
           "\"mac\":\"\""
           "}",
           c->get_led_count(), c->get_current_ma(), WLED_FPS, c->get_max_ma(), WLED_EFFECT_COUNT, WLED_PALETTE_COUNT,
           static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
           static_cast<uint32_t>(millis() / 1000u));
  return std::string(buf);
}

std::string build_effects_json() {
  std::string out = "[";
  for (size_t i = 0; i < WLED_EFFECT_COUNT; i++) {
    if (i > 0)
      out += ",";
    out += "\"";
    out += WLED_EFFECTS[i].meta;
    out += "\"";
  }
  out += "]";
  return out;
}

std::string build_palettes_json() {
  std::string out = "[";
  for (size_t i = 0; i < WLED_PALETTE_COUNT; i++) {
    if (i > 0)
      out += ",";
    out += "\"";
    out += WLED_PALETTES[i].name;
    out += "\"";
  }
  out += "]";
  return out;
}

// ============================================================
// WLEDJsonHandler
// ============================================================
bool WLEDJsonHandler::canHandle(web_server_idf::AsyncWebServerRequest *request) const {
  char url_buf[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
  auto url = request->url_to(url_buf);
  return (strncmp(url.c_str(), "/json", 5) == 0);
}

void WLEDJsonHandler::handleBody(web_server_idf::AsyncWebServerRequest *request, uint8_t *data, size_t len,
                                 size_t index, size_t /*total*/) {
  if (index == 0)
    this->post_body_.clear();
  this->post_body_.append(reinterpret_cast<const char *>(data), len);
}

void WLEDJsonHandler::handleRequest(web_server_idf::AsyncWebServerRequest *request) {
  char url_buf[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
  auto url = request->url_to(url_buf);

  if (request->method() == HTTP_POST) {
    handle_post_state_(request, this->post_body_);
    this->post_body_.clear();
    return;
  }

  // GET routing
  if (strcmp(url.c_str(), "/json") == 0 || strcmp(url.c_str(), "/json/") == 0) {
    // Combined state+info
    std::string body = "{\"state\":" + build_state_json(comp_) + ",\"info\":" + build_info_json(comp_) + "}";
    auto *resp = request->beginResponse(200, "application/json", body);
    request->send(resp);
  } else if (strcmp(url.c_str(), "/json/state") == 0) {
    handle_get_state_(request);
  } else if (strcmp(url.c_str(), "/json/info") == 0) {
    handle_get_info_(request);
  } else if (strcmp(url.c_str(), "/json/effects") == 0 || strcmp(url.c_str(), "/json/eff") == 0) {
    handle_get_effects_(request);
  } else if (strcmp(url.c_str(), "/json/palettes") == 0 || strcmp(url.c_str(), "/json/pal") == 0) {
    handle_get_palettes_(request);
  } else {
    request->send(404, "application/json", "{\"error\":\"not found\"}");
  }
}

void WLEDJsonHandler::handle_get_state_(web_server_idf::AsyncWebServerRequest *request) {
  auto body = build_state_json(comp_);
  auto *resp = request->beginResponse(200, "application/json", body);
  request->send(resp);
}

void WLEDJsonHandler::handle_get_info_(web_server_idf::AsyncWebServerRequest *request) {
  auto body = build_info_json(comp_);
  auto *resp = request->beginResponse(200, "application/json", body);
  request->send(resp);
}

void WLEDJsonHandler::handle_get_effects_(web_server_idf::AsyncWebServerRequest *request) {
  auto body = build_effects_json();
  auto *resp = request->beginResponse(200, "application/json", body);
  request->send(resp);
}

void WLEDJsonHandler::handle_get_palettes_(web_server_idf::AsyncWebServerRequest *request) {
  auto body = build_palettes_json();
  auto *resp = request->beginResponse(200, "application/json", body);
  request->send(resp);
}

void WLEDJsonHandler::handle_post_state_(web_server_idf::AsyncWebServerRequest *request, const std::string &body) {
  if (body.empty()) {
    request->send(200, "application/json", "{}");
    return;
  }

  // Parse JSON using ESPHome's ArduinoJson wrapper
  auto doc = json::parse_json(body);
  if (doc.isNull()) {
    request->send(400, "application/json", "{\"error\":\"invalid json\"}");
    return;
  }

  // on / bri
  if (doc.containsKey("on"))
    comp_->set_on(doc["on"].as<bool>());
  if (doc.containsKey("bri"))
    comp_->set_brightness(doc["bri"].as<uint8_t>());
  if (doc.containsKey("transition"))
    comp_->set_transition(doc["transition"].as<uint16_t>());

  // Segment 0 params (WLED-compatible)
  JsonVariant seg0;
  if (doc.containsKey("seg") && doc["seg"].is<JsonArray>() && doc["seg"].size() > 0) {
    seg0 = doc["seg"][0];
  } else {
    seg0 = doc;  // flat form (some clients send params at root level)
  }

  if (!seg0.isNull()) {
    if (seg0.containsKey("fx"))
      comp_->set_effect(seg0["fx"].as<uint8_t>());
    if (seg0.containsKey("sx"))
      comp_->set_speed(seg0["sx"].as<uint8_t>());
    if (seg0.containsKey("ix"))
      comp_->set_intensity(seg0["ix"].as<uint8_t>());
    if (seg0.containsKey("pal"))
      comp_->set_palette(seg0["pal"].as<uint8_t>());
    if (seg0.containsKey("c1"))
      comp_->set_speed(seg0["c1"].as<uint8_t>());  // custom1
    if (seg0.containsKey("c2"))
      comp_->set_speed(seg0["c2"].as<uint8_t>());  // custom2
    if (seg0.containsKey("c3"))
      comp_->set_speed(seg0["c3"].as<uint8_t>());  // custom3

    // Colors: col is array of [r,g,b] arrays
    if (seg0.containsKey("col") && seg0["col"].is<JsonArray>()) {
      JsonArray cols = seg0["col"].as<JsonArray>();
      for (size_t i = 0; i < cols.size() && i < 3; i++) {
        if (cols[i].is<JsonArray>() && cols[i].size() >= 3) {
          uint8_t r = cols[i][0];
          uint8_t g = cols[i][1];
          uint8_t b = cols[i][2];
          comp_->set_color(static_cast<uint8_t>(i), RGBW32(r, g, b));
        }
      }
    }

    if (seg0.containsKey("on"))
      comp_->set_on(seg0["on"].as<bool>());
    if (seg0.containsKey("bri"))
      comp_->set_brightness(seg0["bri"].as<uint8_t>());
  }

  // Return updated state
  auto state_body = build_state_json(comp_);
  auto *resp = request->beginResponse(200, "application/json", state_body);
  request->send(resp);
}

// ============================================================
// WLEDSseHandler
// ============================================================
bool WLEDSseHandler::canHandle(web_server_idf::AsyncWebServerRequest *request) const {
  char url_buf[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
  auto url = request->url_to(url_buf);
  return strcmp(url.c_str(), "/wled_events") == 0 || strcmp(url.c_str(), "/json/live") == 0;
}

void WLEDSseHandler::handleRequest(web_server_idf::AsyncWebServerRequest *request) {
  // Serve current state as a regular JSON response (polling fallback).
  // Full SSE streaming requires esp_http_server async chunked responses
  // which are not supported by the ESPHome IDF shim in this release.
  auto body = build_state_json(comp_);
  auto *resp = request->beginResponse(200, "application/json", body);
  request->send(resp);
}

void WLEDSseHandler::broadcast_state() {
  // In polling mode, just update the cached state — next /wled_events GET returns it.
  this->pending_state_ = build_state_json(comp_);
  this->has_pending_ = true;
  ESP_LOGV(TAG, "State broadcast: %s", this->pending_state_.c_str());
}

// ============================================================
// WLEDUiHandler
// ============================================================
bool WLEDUiHandler::canHandle(web_server_idf::AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET)
    return false;
  char url_buf[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
  auto url = request->url_to(url_buf);
  return (strcmp(url.c_str(), "/") == 0 || strcmp(url.c_str(), "/index.htm") == 0 ||
          strcmp(url.c_str(), "/index.html") == 0);
}

void WLEDUiHandler::handleRequest(web_server_idf::AsyncWebServerRequest *request) {
  if (WLED_INDEX_GZ_SIZE > 0) {
    auto *resp = request->beginResponse(200, "text/html", WLED_INDEX_GZ, WLED_INDEX_GZ_SIZE);
    resp->addHeader("Content-Encoding", "gzip");
    request->send(resp);
  } else {
    // Minimal fallback page
    request->send(200, "text/html",
                  "<html><head><title>WLED Bridge</title></head><body>"
                  "<h1>WLED Bridge</h1>"
                  "<p>JSON API available at <a href='/json'>/json</a></p>"
                  "</body></html>");
  }
}

}  // namespace wled_bridge
}  // namespace esphome

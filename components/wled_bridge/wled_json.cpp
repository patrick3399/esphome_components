#include "wled_json.h"
#include "wled_ui_data.h"
#include "wled_bridge.h"
#include "wled_effects.h"
#include "wled_palette.h"
#include "esphome/core/log.h"
#include "esphome/components/json/json_util.h"
#include <algorithm>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

namespace esphome {
namespace wled_bridge {

static const char *const TAG = "wled_bridge.json";

static bool json_bool(JsonVariant value) {
  if (value.is<bool>())
    return value.as<bool>();
  if (value.is<int>())
    return value.as<int>() != 0;
  if (value.is<const char *>()) {
    const char *text = value.as<const char *>();
    return strcmp(text, "true") == 0 || strcmp(text, "1") == 0 || strcmp(text, "on") == 0;
  }
  return false;
}

static uint8_t json_u8(JsonVariant value, uint8_t fallback = 0) {
  if (value.isNull())
    return fallback;
  int v = value.as<int>();
  if (v < 0)
    return 0;
  if (v > 255)
    return 255;
  return static_cast<uint8_t>(v);
}

static bool parse_u16(const std::string &value, uint16_t *out) {
  if (value.empty())
    return false;
  char *end = nullptr;
  long parsed = strtol(value.c_str(), &end, 10);
  if (end == value.c_str() || *end != '\0')
    return false;
  if (parsed < 0)
    parsed = 0;
  if (parsed > 65535)
    parsed = 65535;
  *out = static_cast<uint16_t>(parsed);
  return true;
}

static bool request_arg_u16(web_server_idf::AsyncWebServerRequest *request, const char *name, uint16_t *out) {
  if (!request->hasArg(name))
    return false;
  return parse_u16(request->arg(name), out);
}

static bool legacy_path_arg_u16(const std::string &url, const char *name, uint16_t *out) {
  std::string token = std::string("&") + name + "=";
  size_t pos = url.find(token);
  if (pos == std::string::npos)
    return false;
  pos += token.size();
  size_t end = url.find('&', pos);
  return parse_u16(url.substr(pos, end == std::string::npos ? std::string::npos : end - pos), out);
}

static bool win_arg_u16(web_server_idf::AsyncWebServerRequest *request, const std::string &url, const char *name,
                        uint16_t *out) {
  return request_arg_u16(request, name, out) || legacy_path_arg_u16(url, name, out);
}

static uint16_t transition_tenths_to_ms(uint16_t tenths) {
  return static_cast<uint16_t>(std::min<uint32_t>(tenths, 655u) * 100u);
}

static std::string json_escape(const char *text) {
  std::string out;
  if (text == nullptr)
    return out;
  for (const char *p = text; *p != '\0'; p++) {
    if (*p == '"' || *p == '\\')
      out += '\\';
    if (static_cast<unsigned char>(*p) >= 0x20)
      out += *p;
  }
  return out;
}

static std::string build_preset_validity_json(const WLEDBridgeComponent *c) {
  std::string out = "[";
  for (uint8_t id = 1; id <= WLED_PRESET_COUNT; id++) {
    if (id > 1)
      out += ",";
    out += c->is_preset_valid(id) ? "true" : "false";
  }
  out += "]";
  return out;
}

static std::string string_sprintf(const char *format, ...) {
  va_list args;
  va_start(args, format);
  va_list args_copy;
  va_copy(args_copy, args);
  int len = vsnprintf(nullptr, 0, format, args);
  va_end(args);
  if (len <= 0) {
    va_end(args_copy);
    return "";
  }

  std::vector<char> buf(static_cast<size_t>(len) + 1);
  vsnprintf(buf.data(), buf.size(), format, args_copy);
  va_end(args_copy);
  return std::string(buf.data(), static_cast<size_t>(len));
}

// ============================================================
// JSON builder helpers
// ============================================================
std::string build_state_json(const WLEDBridgeComponent *c) {
  const EffectParams &p = c->get_params();
  uint16_t transition_tenths = c->get_transition_ms() / 100u;
  std::string presets = build_preset_validity_json(c);
  return string_sprintf("{"
                        "\"on\":%s,"
                        "\"bri\":%u,"
                        "\"transition\":%u,"
                        "\"tt\":%u,"
                        "\"ps\":%u,"
                        "\"pl\":-1,"
                        "\"presets\":%s,"
                        "\"nl\":{\"on\":false,\"dur\":60,\"mode\":1,\"tbri\":0,\"rem\":-1},"
                        "\"udpn\":{\"send\":false,\"recv\":false,\"sgrp\":1,\"rgrp\":1},"
                        "\"time\":0,"
                        "\"lor\":0,"
                        "\"mainseg\":0,"
                        "\"seg\":[{"
                        "\"id\":0,"
                        "\"start\":%u,"
                        "\"stop\":%u,"
                        "\"len\":%u,"
                        "\"grp\":1,"
                        "\"spc\":0,"
                        "\"of\":0,"
                        "\"on\":%s,"
                        "\"frz\":false,"
                        "\"bri\":%u,"
                        "\"cct\":127,"
                        "\"set\":0,"
                        "\"n\":\"Main\","
                        "\"col\":[[%u,%u,%u,%u],[%u,%u,%u,%u],[%u,%u,%u,%u]],"
                        "\"fx\":%u,"
                        "\"sx\":%u,"
                        "\"ix\":%u,"
                        "\"pal\":%u,"
                        "\"c1\":%u,"
                        "\"c2\":%u,"
                        "\"c3\":%u,"
                        "\"o1\":%s,"
                        "\"o2\":%s,"
                        "\"o3\":%s,"
                        "\"sel\":true,"
                        "\"rev\":%s,"
                        "\"mi\":%s,"
                        "\"m12\":0"
                        "}]}",
                        c->is_on() ? "true" : "false", c->get_brightness(), transition_tenths, transition_tenths,
                        c->get_active_preset(), presets.c_str(), c->get_segment_start(), c->get_segment_stop(),
                        c->get_segment_length(), c->is_on() ? "true" : "false", c->get_brightness(), R(p.colors[0]),
                        G(p.colors[0]), B(p.colors[0]), W(p.colors[0]), R(p.colors[1]), G(p.colors[1]), B(p.colors[1]),
                        W(p.colors[1]), R(p.colors[2]), G(p.colors[2]), B(p.colors[2]), W(p.colors[2]),
                        c->get_effect_index(), p.speed, p.intensity, p.palette_id, p.custom1, p.custom2, p.custom3,
                        p.check1 ? "true" : "false", p.check2 ? "true" : "false", p.check3 ? "true" : "false",
                        c->is_segment_reversed() ? "true" : "false", c->is_segment_mirrored() ? "true" : "false");
}

std::string build_info_json(const WLEDBridgeComponent *c) {
  return string_sprintf("{"
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
                        "\"udpport\":21324,"
                        "\"udp\":0,"
                        "\"live\":false,"
                        "\"liveseg\":-1,"
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
                        "\"ndc\":0,"
                        "\"cn\":\"ESPHome\","
                        "\"brand\":\"ESPHome\","
                        "\"product\":\"WLED Bridge\","
                        "\"btype\":\"ESPHome external component\","
                        "\"release\":\"0.1.0\","
                        "\"mac\":\"\""
                        "}",
                        c->get_led_count(), c->get_current_ma(), WLED_FPS, c->get_max_ma(), WLED_EFFECT_COUNT,
                        WLED_PALETTE_COUNT, static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
                        static_cast<uint32_t>(millis() / 1000u));
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

std::string build_config_json(const WLEDBridgeComponent *c) {
  return string_sprintf(
      "{"
      "\"rev\":[1,0],"
      "\"vid\":2605010,"
      "\"id\":{"
      "\"mdns\":\"wled-bridge\","
      "\"name\":\"WLED Bridge\","
      "\"inv\":\"\","
      "\"brand\":\"ESPHome\","
      "\"product\":\"WLED Bridge\""
      "},"
      "\"nw\":{"
      "\"ins\":[{\"ssid\":\"\",\"pskl\":0,\"ip\":[0,0,0,0],\"gw\":[0,0,0,0],\"sn\":[0,0,0,0]}]"
      "},"
      "\"ap\":{\"ssid\":\"\",\"pskl\":0,\"chan\":1,\"hide\":0,\"behav\":0},"
      "\"wifi\":{\"sleep\":true,\"phy\":3},"
      "\"hw\":{"
      "\"led\":{"
      "\"total\":%u,"
      "\"maxpwr\":%u,"
      "\"ledma\":%u,"
      "\"rgbwm\":0,"
      "\"cct\":false,"
      "\"cr\":false,"
      "\"cb\":0,"
      "\"ins\":[{\"start\":0,\"len\":%u,\"pin\":[-1],\"order\":0,\"rev\":false,\"skip\":0,\"type\":22,\"ref\":false}]"
      "},"
      "\"btn\":{\"max\":0,\"ins\":[]},"
      "\"ir\":{\"pin\":-1,\"type\":0},"
      "\"relay\":{\"pin\":-1,\"rev\":false}"
      "},"
      "\"light\":{"
      "\"scale-bri\":100,"
      "\"pal-mode\":0,"
      "\"gc\":{\"bri\":1.0,\"col\":1.0,\"val\":2.8},"
      "\"tr\":{\"mode\":true,\"dur\":%u}"
      "},"
      "\"def\":{\"ps\":%u,\"on\":%s,\"bri\":%u,\"fx\":%u,\"sx\":%u,\"ix\":%u,\"pal\":%u},"
      "\"if\":{"
      "\"sync\":{\"port0\":21324,\"port1\":65506,\"recv\":{\"bri\":true,\"col\":true,\"fx\":true},\"send\":{\"dir\":"
      "false}},"
      "\"live\":{\"en\":false,\"mso\":false,\"port\":5568},"
      "\"va\":{\"alexa\":false,\"macros\":[0,0]},"
      "\"mqtt\":{\"en\":false}"
      "},"
      "\"ol\":{\"clock\":0,\"cntdwn\":false,\"min\":0,\"max\":29},"
      "\"timers\":{\"cntdwn\":{\"goal\":[20,1,1,0,0,0],\"macro\":0},\"ins\":[]}"
      "}",
      c->get_led_count(), c->get_max_ma(), c->get_led_count() == 0 ? 0 : c->get_max_ma() / c->get_led_count(),
      c->get_led_count(), c->get_transition_ms() / 100u, c->get_active_preset(), c->is_on() ? "true" : "false",
      c->get_brightness(), c->get_effect_index(), c->get_params().speed, c->get_params().intensity,
      c->get_params().palette_id);
}

std::string build_network_json(const WLEDBridgeComponent *c) {
  return string_sprintf("{"
                        "\"nodes\":[],"
                        "\"info\":{"
                        "\"ip\":\"\","
                        "\"name\":\"WLED Bridge\","
                        "\"type\":\"ESPHome\","
                        "\"vid\":2605010,"
                        "\"live\":false,"
                        "\"lm\":\"\","
                        "\"lip\":\"\","
                        "\"fxcount\":%zu,"
                        "\"palcount\":%zu,"
                        "\"leds\":{\"count\":%u,\"pwr\":%u,\"maxpwr\":%u}"
                        "}"
                        "}",
                        WLED_EFFECT_COUNT, WLED_PALETTE_COUNT, c->get_led_count(), c->get_current_ma(),
                        c->get_max_ma());
}

std::string build_presets_json(const WLEDBridgeComponent *c) {
  std::string out = "{";
  bool first = true;
  for (uint8_t id = 1; id <= WLED_PRESET_COUNT; id++) {
    const auto *preset = c->get_preset(id);
    if (preset == nullptr || preset->valid == 0)
      continue;

    std::string name = json_escape(preset->name[0] != '\0' ? preset->name : "Preset");
    std::string preset_json = string_sprintf(
        "%s\"%u\":{"
        "\"n\":\"%s\","
        "\"on\":%s,"
        "\"bri\":%u,"
        "\"transition\":%u,"
        "\"tt\":%u,"
        "\"seg\":[{"
        "\"id\":0,"
        "\"start\":%u,"
        "\"stop\":%u,"
        "\"on\":%s,"
        "\"bri\":%u,"
        "\"col\":[[%u,%u,%u,%u],[%u,%u,%u,%u],[%u,%u,%u,%u]],"
        "\"fx\":%u,"
        "\"sx\":%u,"
        "\"ix\":%u,"
        "\"pal\":%u,"
        "\"c1\":%u,"
        "\"c2\":%u,"
        "\"c3\":%u,"
        "\"o1\":%s,"
        "\"o2\":%s,"
        "\"o3\":%s,"
        "\"rev\":%s,"
        "\"mi\":%s"
        "}]}",
        first ? "" : ",", id, name.c_str(), preset->on != 0 ? "true" : "false", preset->brightness,
        preset->transition_ms / 100u, preset->transition_ms / 100u, preset->segment_start, preset->segment_stop,
        preset->on != 0 ? "true" : "false", preset->brightness, R(preset->colors[0]), G(preset->colors[0]),
        B(preset->colors[0]), W(preset->colors[0]), R(preset->colors[1]), G(preset->colors[1]), B(preset->colors[1]),
        W(preset->colors[1]), R(preset->colors[2]), G(preset->colors[2]), B(preset->colors[2]), W(preset->colors[2]),
        preset->effect, preset->speed, preset->intensity, preset->palette, preset->custom1, preset->custom2,
        preset->custom3, preset->check1 != 0 ? "true" : "false", preset->check2 != 0 ? "true" : "false",
        preset->check3 != 0 ? "true" : "false", preset->reverse != 0 ? "true" : "false",
        preset->mirror != 0 ? "true" : "false");
    out += preset_json;
    first = false;
  }
  out += "}";
  return out;
}

// ============================================================
// WLEDJsonHandler
// ============================================================
bool WLEDJsonHandler::canHandle(web_server_idf::AsyncWebServerRequest *request) const {
  char url_buf[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
  auto url = request->url_to(url_buf);
  return strncmp(url.c_str(), "/json", 5) == 0 || strcmp(url.c_str(), "/presets.json") == 0 ||
         strcmp(url.c_str(), "/win") == 0 || strncmp(url.c_str(), "/win&", 5) == 0 ||
         strcmp(url.c_str(), "/version") == 0 || strcmp(url.c_str(), "/freeheap") == 0;
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
  std::string url_string(url.c_str());

  if (strcmp(url.c_str(), "/win") == 0 || strncmp(url.c_str(), "/win&", 5) == 0) {
    handle_win_(request, url_string);
    return;
  }

  if (strcmp(url.c_str(), "/presets.json") == 0) {
    if (request->method() == HTTP_GET) {
      handle_get_presets_(request);
    } else {
      request->send(405, "application/json", "{\"error\":\"method not allowed\"}");
    }
    return;
  }

  if (request->method() == HTTP_POST) {
    handle_post_state_(request, this->post_body_);
    this->post_body_.clear();
    return;
  }

  // GET routing
  if (strcmp(url.c_str(), "/json") == 0 || strcmp(url.c_str(), "/json/") == 0) {
    // Full WLED combined response: state + info + effects + palettes
    std::string body = "{\"state\":" + build_state_json(comp_) + ",\"info\":" + build_info_json(comp_) +
                       ",\"effects\":" + build_effects_json() + ",\"palettes\":" + build_palettes_json() + "}";
    auto *resp = request->beginResponse(200, "application/json", body);
    request->send(resp);
  } else if (strcmp(url.c_str(), "/json/si") == 0) {
    // State + info only (WLED /json/si endpoint)
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
  } else if (strcmp(url.c_str(), "/json/cfg") == 0) {
    handle_get_config_(request);
  } else if (strcmp(url.c_str(), "/json/net") == 0 || strcmp(url.c_str(), "/json/nodes") == 0) {
    handle_get_network_(request);
  } else if (strcmp(url.c_str(), "/version") == 0) {
    request->send(200, "application/json", "\"WLED Bridge 0.1.0\"");
  } else if (strcmp(url.c_str(), "/freeheap") == 0) {
    auto body = std::to_string(static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
    auto *resp = request->beginResponse(200, "text/plain", body);
    request->send(resp);
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

void WLEDJsonHandler::handle_get_config_(web_server_idf::AsyncWebServerRequest *request) {
  auto body = build_config_json(comp_);
  auto *resp = request->beginResponse(200, "application/json", body);
  request->send(resp);
}

void WLEDJsonHandler::handle_get_network_(web_server_idf::AsyncWebServerRequest *request) {
  auto body = build_network_json(comp_);
  auto *resp = request->beginResponse(200, "application/json", body);
  request->send(resp);
}

void WLEDJsonHandler::handle_get_presets_(web_server_idf::AsyncWebServerRequest *request) {
  auto body = build_presets_json(comp_);
  auto *resp = request->beginResponse(200, "application/json", body);
  request->send(resp);
}

void WLEDJsonHandler::handle_win_(web_server_idf::AsyncWebServerRequest *request, const std::string &url) {
  uint16_t value = 0;
  bool changed = false;

  if (win_arg_u16(request, url, "T", &value)) {
    if (value == 0)
      comp_->set_on(false);
    else if (value == 1)
      comp_->set_on(true);
    else
      comp_->set_on(!comp_->is_on());
    changed = true;
  }
  if (win_arg_u16(request, url, "A", &value)) {
    comp_->set_brightness(static_cast<uint8_t>(value > 255 ? 255 : value));
    changed = true;
  }
  if (win_arg_u16(request, url, "FX", &value)) {
    comp_->set_effect(static_cast<uint8_t>(value > 255 ? 255 : value));
    changed = true;
  }
  if (win_arg_u16(request, url, "SX", &value)) {
    comp_->set_speed(static_cast<uint8_t>(value > 255 ? 255 : value));
    changed = true;
  }
  if (win_arg_u16(request, url, "IX", &value)) {
    comp_->set_intensity(static_cast<uint8_t>(value > 255 ? 255 : value));
    changed = true;
  }
  if (win_arg_u16(request, url, "FP", &value)) {
    comp_->set_palette(static_cast<uint8_t>(value > 255 ? 255 : value));
    changed = true;
  }
  if (win_arg_u16(request, url, "TT", &value)) {
    comp_->set_transition(transition_tenths_to_ms(value));
    changed = true;
  }
  if (win_arg_u16(request, url, "PL", &value)) {
    if (comp_->load_preset(static_cast<uint8_t>(value > 255 ? 255 : value)))
      changed = true;
  }
  if (win_arg_u16(request, url, "PS", &value)) {
    if (comp_->save_preset(static_cast<uint8_t>(value > 255 ? 255 : value)))
      changed = true;
  }
  if (win_arg_u16(request, url, "PD", &value)) {
    if (comp_->delete_preset(static_cast<uint8_t>(value > 255 ? 255 : value)))
      changed = true;
  }

  uint16_t r = R(comp_->get_params().colors[0]);
  uint16_t g = G(comp_->get_params().colors[0]);
  uint16_t b = B(comp_->get_params().colors[0]);
  uint16_t w = W(comp_->get_params().colors[0]);
  bool color_changed = false;
  if (win_arg_u16(request, url, "R", &value)) {
    r = value > 255 ? 255 : value;
    color_changed = true;
  }
  if (win_arg_u16(request, url, "G", &value)) {
    g = value > 255 ? 255 : value;
    color_changed = true;
  }
  if (win_arg_u16(request, url, "B", &value)) {
    b = value > 255 ? 255 : value;
    color_changed = true;
  }
  if (win_arg_u16(request, url, "W", &value)) {
    w = value > 255 ? 255 : value;
    color_changed = true;
  }
  if (color_changed) {
    comp_->set_color(0, RGBW32(r, g, b, w));
    changed = true;
  }

  if (changed)
    comp_->publish_light_state();

  auto state_body = build_state_json(comp_);
  auto *resp = request->beginResponse(200, "application/json", state_body);
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
  if (!doc["on"].isNull())
    comp_->set_on(json_bool(doc["on"]));
  if (!doc["bri"].isNull())
    comp_->set_brightness(json_u8(doc["bri"], comp_->get_brightness()));
  if (!doc["transition"].isNull())
    comp_->set_transition(transition_tenths_to_ms(doc["transition"].as<uint16_t>()));
  if (!doc["tt"].isNull())
    comp_->set_transition(transition_tenths_to_ms(doc["tt"].as<uint16_t>()));
  if (!doc["ps"].isNull())
    comp_->load_preset(json_u8(doc["ps"]));
  if (!doc["pdel"].isNull())
    comp_->delete_preset(json_u8(doc["pdel"]));

  // Segment 0 params (WLED-compatible)
  JsonVariant seg0;
  if (doc["seg"].is<JsonArray>() && doc["seg"].size() > 0) {
    seg0 = doc["seg"][0];
  } else if (doc["seg"].is<JsonObject>()) {
    seg0 = doc["seg"];
  } else {
    seg0 = doc;  // flat form (some clients send params at root level)
  }

  if (!seg0.isNull()) {
    if (!seg0["fx"].isNull())
      comp_->set_effect(json_u8(seg0["fx"], comp_->get_effect_index()));
    if (!seg0["sx"].isNull())
      comp_->set_speed(json_u8(seg0["sx"], comp_->get_params().speed));
    if (!seg0["ix"].isNull())
      comp_->set_intensity(json_u8(seg0["ix"], comp_->get_params().intensity));
    if (!seg0["pal"].isNull())
      comp_->set_palette(json_u8(seg0["pal"], comp_->get_params().palette_id));
    if (!seg0["c1"].isNull())
      comp_->set_custom1(json_u8(seg0["c1"], comp_->get_params().custom1));
    if (!seg0["c2"].isNull())
      comp_->set_custom2(json_u8(seg0["c2"], comp_->get_params().custom2));
    if (!seg0["c3"].isNull())
      comp_->set_custom3(json_u8(seg0["c3"], comp_->get_params().custom3));
    if (!seg0["o1"].isNull())
      comp_->set_check1(json_bool(seg0["o1"]));
    if (!seg0["o2"].isNull())
      comp_->set_check2(json_bool(seg0["o2"]));
    if (!seg0["o3"].isNull())
      comp_->set_check3(json_bool(seg0["o3"]));
    if (!seg0["start"].isNull() || !seg0["stop"].isNull()) {
      uint32_t start = seg0["start"].isNull() ? comp_->get_segment_start() : seg0["start"].as<uint32_t>();
      uint32_t stop = seg0["stop"].isNull() ? comp_->get_segment_stop() : seg0["stop"].as<uint32_t>();
      comp_->set_segment_bounds(start, stop);
    }
    if (!seg0["rev"].isNull())
      comp_->set_segment_reverse(json_bool(seg0["rev"]));
    if (!seg0["mi"].isNull())
      comp_->set_segment_mirror(json_bool(seg0["mi"]));

    // Colors: col is array of [r,g,b] arrays
    if (seg0["col"].is<JsonArray>()) {
      JsonArray cols = seg0["col"].as<JsonArray>();
      for (size_t i = 0; i < cols.size() && i < 3; i++) {
        if (cols[i].is<JsonArray>() && cols[i].size() >= 3) {
          uint8_t r = json_u8(cols[i][0]);
          uint8_t g = json_u8(cols[i][1]);
          uint8_t b = json_u8(cols[i][2]);
          uint8_t w = cols[i].size() >= 4 ? json_u8(cols[i][3]) : 0;
          comp_->set_color(static_cast<uint8_t>(i), RGBW32(r, g, b, w));
        } else if (cols[i].is<uint32_t>()) {
          comp_->set_color(static_cast<uint8_t>(i), cols[i].as<uint32_t>());
        }
      }
    }

    if (!seg0["on"].isNull())
      comp_->set_on(json_bool(seg0["on"]));
    if (!seg0["bri"].isNull())
      comp_->set_brightness(json_u8(seg0["bri"], comp_->get_brightness()));
  }

  if (!doc["psave"].isNull())
    comp_->save_preset(json_u8(doc["psave"]), doc["n"].isNull() ? nullptr : doc["n"].as<const char *>());

  comp_->publish_light_state();

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
  std::string state = this->has_pending_ ? this->pending_state_ : build_state_json(comp_);
  uint32_t version = this->has_pending_ ? this->pending_version_ : comp_->get_state_version();
  std::string body = "{\"version\":" + std::to_string(version) + ",\"state\":" + state + "}";
  auto *resp = request->beginResponse(200, "application/json", body);
  request->send(resp);
}

void WLEDSseHandler::broadcast_state() {
  // In polling mode, cache the latest versioned state for the next /wled_events GET.
  this->pending_state_ = build_state_json(comp_);
  this->pending_version_ = comp_->get_state_version();
  this->has_pending_ = true;
  ESP_LOGV(TAG, "State broadcast v%u: %s", this->pending_version_, this->pending_state_.c_str());
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

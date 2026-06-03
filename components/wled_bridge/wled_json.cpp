#include "esphome/core/defines.h"
#include "wled_json.h"
#ifdef WLED_BRIDGE_WEB_UI
#include "wled_ui_data.h"
#endif
#include "wled_bridge.h"
#include "wled_color.h"
#include "wled_effects.h"
#include "wled_palette.h"
#include "esphome/core/log.h"
#include "esphome/components/json/json_util.h"
#include <algorithm>
#include <esp_random.h>
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
  if (request == nullptr)
    return false;
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

static bool legacy_path_arg_string(const std::string &url, const char *name, std::string *out) {
  std::string token = std::string("&") + name + "=";
  size_t pos = url.find(token);
  if (pos == std::string::npos)
    return false;
  pos += token.size();
  size_t end = url.find('&', pos);
  *out = url.substr(pos, end == std::string::npos ? std::string::npos : end - pos);
  return true;
}

static uint8_t clamp_segment_id(size_t id) {
  if (id >= WLED_MAX_SEGMENTS)
    return WLED_MAX_SEGMENTS - 1;
  return static_cast<uint8_t>(id);
}

static uint32_t json_color(JsonVariant value, uint32_t fallback) {
  if (value.is<JsonArray>()) {
    JsonArray arr = value.as<JsonArray>();
    if (arr.size() >= 3) {
      uint8_t r = json_u8(arr[0]);
      uint8_t g = json_u8(arr[1]);
      uint8_t b = json_u8(arr[2]);
      uint8_t w = arr.size() >= 4 ? json_u8(arr[3]) : 0;
      return RGBW32(r, g, b, w);
    }
  }
  if (value.is<uint32_t>())
    return value.as<uint32_t>();
  return fallback;
}

static int hex_nibble(char c) {
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'a' && c <= 'f')
    return c - 'a' + 10;
  if (c >= 'A' && c <= 'F')
    return c - 'A' + 10;
  return -1;
}

static bool parse_hex_color(const char *text, uint32_t *out) {
  if (text == nullptr || out == nullptr)
    return false;
  if (text[0] == '#')
    text++;
  size_t len = strlen(text);
  if (len != 6 && len != 8)
    return false;
  uint32_t value = 0;
  for (size_t i = 0; i < len; i++) {
    int nibble = hex_nibble(text[i]);
    if (nibble < 0)
      return false;
    value = (value << 4) | static_cast<uint32_t>(nibble);
  }
  if (len == 6) {
    *out = RGBW32(static_cast<uint8_t>((value >> 16) & 0xFF), static_cast<uint8_t>((value >> 8) & 0xFF),
                  static_cast<uint8_t>(value & 0xFF), 0);
  } else {
    *out = RGBW32(static_cast<uint8_t>((value >> 16) & 0xFF), static_cast<uint8_t>((value >> 8) & 0xFF),
                  static_cast<uint8_t>(value & 0xFF), static_cast<uint8_t>((value >> 24) & 0xFF));
  }
  return true;
}

static bool json_pixel_color(JsonVariant value, uint32_t *out) {
  if (out == nullptr || value.isNull())
    return false;
  if (value.is<JsonArray>()) {
    JsonArray arr = value.as<JsonArray>();
    if (arr.size() == 0 || arr.size() > 4)
      return false;
    uint8_t r = arr.size() > 0 ? json_u8(arr[0]) : 0;
    uint8_t g = arr.size() > 1 ? json_u8(arr[1]) : 0;
    uint8_t b = arr.size() > 2 ? json_u8(arr[2]) : 0;
    uint8_t w = arr.size() > 3 ? json_u8(arr[3]) : 0;
    *out = RGBW32(r, g, b, w);
    return true;
  }
  if (value.is<const char *>())
    return parse_hex_color(value.as<const char *>(), out);
  if (value.is<uint32_t>()) {
    *out = value.as<uint32_t>();
    return true;
  }
  return false;
}

static bool win_arg_u16(web_server_idf::AsyncWebServerRequest *request, const std::string &url, const char *name,
                        uint16_t *out) {
  return request_arg_u16(request, name, out) || legacy_path_arg_u16(url, name, out);
}

static bool legacy_path_has_arg(const std::string &url, const char *name) {
  std::string token = std::string("&") + name;
  size_t pos = url.find(token);
  if (pos == std::string::npos)
    return false;
  size_t end = pos + token.size();
  return end >= url.size() || url[end] == '&' || url[end] == '=';
}

static bool win_has_arg(web_server_idf::AsyncWebServerRequest *request, const std::string &url, const char *name) {
  return (request != nullptr && request->hasArg(name)) || legacy_path_has_arg(url, name);
}

static bool parse_bool_text(const std::string &value) {
  return value.empty() || value == "1" || value == "true" || value == "on" || value == "yes" || value == "checked";
}

static bool win_arg_bool(web_server_idf::AsyncWebServerRequest *request, const std::string &url, const char *name,
                         bool *out) {
  if (request != nullptr && request->hasArg(name)) {
    *out = parse_bool_text(request->arg(name));
    return true;
  }
  std::string value;
  if (legacy_path_arg_string(url, name, &value)) {
    *out = parse_bool_text(value);
    return true;
  }
  if (legacy_path_has_arg(url, name)) {
    *out = true;
    return true;
  }
  return false;
}

static bool json_on_is_toggle(JsonVariant value) {
  if (!value.is<const char *>())
    return false;
  const char *text = value.as<const char *>();
  return text != nullptr && (text[0] == 't' || text[0] == 'T');
}

// Parse WLED's incremental value syntax for fx/pal/ps:
//   "~"  = increment by 1 (wrap)
//   "~-" = decrement by 1 (wrap)
//   "r"  = random value in [0, max)
//   number = literal value
// Returns true if the value was handled; writes result to *out.
static bool json_incremental_u8(JsonVariant value, uint8_t current, uint8_t max_val, uint8_t *out) {
  if (value.isNull())
    return false;
  if (value.is<const char *>()) {
    const char *text = value.as<const char *>();
    if (text == nullptr || text[0] == '\0')
      return false;
    if (text[0] == '~') {
      if (text[1] == '-') {
        *out = current == 0 ? (max_val > 0 ? max_val - 1 : 0) : current - 1;
      } else {
        *out = (current + 1 >= max_val) ? 0 : current + 1;
      }
      return true;
    }
    if (text[0] == 'r' || text[0] == 'R') {
      *out = max_val > 0 ? static_cast<uint8_t>(esp_random() % max_val) : 0;
      return true;
    }
    // Might be a numeric string — fall through to numeric parse
    char *end = nullptr;
    long v = strtol(text, &end, 10);
    if (end != text) {
      *out = v < 0 ? 0 : (v > 255 ? 255 : static_cast<uint8_t>(v));
      return true;
    }
    return false;
  }
  // Plain numeric
  int v = value.as<int>();
  *out = v < 0 ? 0 : (v > 255 ? 255 : static_cast<uint8_t>(v));
  return true;
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
static std::string build_segment_object_json(const WLEDBridgeComponent::SegmentReadView &v, uint8_t id) {
  uint16_t len = v.stop > v.start ? static_cast<uint16_t>(v.stop - v.start) : 0;
  return string_sprintf("{"
                        "\"id\":%u,"
                        "\"start\":%u,"
                        "\"stop\":%u,"
                        "\"len\":%u,"
                        "\"grp\":%u,"
                        "\"spc\":%u,"
                        "\"of\":0,"
                        "\"on\":%s,"
                        "\"frz\":false,"
                        "\"bri\":%u,"
                        "\"cct\":127,"
                        "\"set\":0,"
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
                        "\"sel\":%s,"
                        "\"rev\":%s,"
                        "\"mi\":%s,"
                        "\"m12\":0"
                        "}",
                        id, v.start, v.stop, len, v.grouping, v.spacing, v.on ? "true" : "false", v.opacity,
                        R(v.colors[0]), G(v.colors[0]), B(v.colors[0]), W(v.colors[0]), R(v.colors[1]), G(v.colors[1]),
                        B(v.colors[1]), W(v.colors[1]), R(v.colors[2]), G(v.colors[2]), B(v.colors[2]), W(v.colors[2]),
                        v.mode, v.speed, v.intensity, v.palette, v.custom1, v.custom2, v.custom3,
                        v.check1 ? "true" : "false", v.check2 ? "true" : "false", v.check3 ? "true" : "false",
                        v.selected ? "true" : "false", v.reverse ? "true" : "false", v.mirror ? "true" : "false");
}

std::string build_state_json(const WLEDBridgeComponent *c) {
  uint16_t transition_tenths = c->get_transition_ms() / 100u;
  std::string presets = build_preset_validity_json(c);

  std::string seg_array = "[";
  uint8_t count = c->get_segment_count();
  for (uint8_t id = 0; id < count; id++) {
    WLEDBridgeComponent::SegmentReadView view;
    if (!c->get_segment_view(id, view))
      continue;
    if (id > 0)
      seg_array += ",";
    seg_array += build_segment_object_json(view, id);
  }
  seg_array += "]";

  std::string playlist = "{\"on\":";
  playlist += c->is_playlist_active() ? "true" : "false";
  playlist += ",\"index\":";
  playlist += std::to_string(c->get_playlist_index());
  playlist += ",\"repeat\":";
  playlist += std::to_string(c->get_playlist_repeat_remaining());
  playlist += ",\"ps\":[";
  for (uint8_t i = 0; i < c->get_playlist_count(); i++) {
    if (i > 0)
      playlist += ",";
    playlist += std::to_string(c->get_playlist_preset(i));
  }
  playlist += "],\"dur\":[";
  for (uint8_t i = 0; i < c->get_playlist_count(); i++) {
    if (i > 0)
      playlist += ",";
    playlist += std::to_string(c->get_playlist_duration(i));
  }
  playlist += "],\"transition\":[";
  for (uint8_t i = 0; i < c->get_playlist_count(); i++) {
    if (i > 0)
      playlist += ",";
    playlist += std::to_string(c->get_playlist_transition(i));
  }
  playlist += "]}";

  return string_sprintf(
      "{"
      "\"on\":%s,"
      "\"bri\":%u,"
      "\"transition\":%u,"
      "\"tt\":%u,"
      "\"ps\":%u,"
      "\"pl\":%d,"
      "\"playlist\":%s,"
      "\"presets\":%s,"
      "\"nl\":{\"on\":%s,\"dur\":%u,\"mode\":%u,\"tbri\":%u,\"rem\":%d},"
      "\"udpn\":{\"send\":%s,\"recv\":%s,\"sgrp\":%u,\"rgrp\":%u,"
      "\"rb\":%s,\"rc\":%s,\"rx\":%s,\"rp\":%s,\"so\":%s,\"sg\":%s,"
      "\"dir\":%s,\"btn\":%s,\"va\":%s,\"hue\":%s,\"ret\":%u},"
      "\"time\":0,"
      "\"lor\":0,"
      "\"mainseg\":%u,"
      "\"seg\":%s}",
      c->is_on() ? "true" : "false", c->get_brightness(), transition_tenths, transition_tenths, c->get_active_preset(),
      c->get_active_playlist(), playlist.c_str(), presets.c_str(), c->is_nightlight_active() ? "true" : "false",
      c->get_nightlight_duration_min(), c->get_nightlight_mode(), c->get_nightlight_target_brightness(),
      c->get_nightlight_remaining_s(), c->get_udp_send() ? "true" : "false", c->get_udp_receive() ? "true" : "false",
      c->get_udp_sync_groups(), c->get_udp_receive_groups(), c->get_udp_receive_brightness() ? "true" : "false",
      c->get_udp_receive_color() ? "true" : "false", c->get_udp_receive_effects() ? "true" : "false",
      c->get_udp_receive_palette() ? "true" : "false", c->get_udp_receive_segment_options() ? "true" : "false",
      c->get_udp_receive_segments() ? "true" : "false", c->get_udp_notify_direct() ? "true" : "false",
      c->get_udp_notify_button() ? "true" : "false", c->get_udp_notify_alexa() ? "true" : "false",
      c->get_udp_notify_hue() ? "true" : "false", c->get_udp_retries(), c->get_main_segment(), seg_array.c_str());
}

std::string build_info_json(const WLEDBridgeComponent *c) {
  // Build matrix field: present only when 2D is configured
  std::string matrix_field;
  if (c->is_2d()) {
    matrix_field = string_sprintf(",\"matrix\":{\"w\":%u,\"h\":%u,\"s\":%s}", c->get_matrix_width(),
                                  c->get_matrix_height(), c->get_matrix_serpentine() ? "true" : "false");
  }

  return string_sprintf(
      "{"
      "\"ver\":\"0.1.0\","
      "\"vid\":2605010,"
      "\"leds\":{"
      "\"count\":%u,"
      "\"pwr\":%u,"
      "\"fps\":%u,"
      "\"maxpwr\":%u,"
      "\"maxseg\":%u,"
      "\"cco\":0"
      "},"
      "\"str\":false,"
      "\"name\":\"WLED Bridge\","
      "\"udpport\":%u,"
      "\"live\":%s,"
      "\"liveseg\":-1,"
      "\"lm\":\"%s\","
      "\"lip\":\"\","
      "\"ws\":%d,"
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
      "%s"
      "}",
      c->get_led_count(), c->get_current_ma(), WLED_FPS, c->get_max_ma(), WLEDBridgeComponent::get_max_segments(),
      c->get_udp_port(),
      (c->is_ddp_receiving() || c->is_e131_receiving() || c->is_artnet_receiving()) ? "true" : "false",
      c->is_ddp_receiving() ? "DDP" : (c->is_e131_receiving() ? "E1.31" : (c->is_artnet_receiving() ? "Art-Net" : "")),
#ifdef USE_ESP32
      c->get_ws_client_count(),
#else
      0,
#endif
      WLED_EFFECT_COUNT, WLED_PALETTE_COUNT, static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
      static_cast<uint32_t>(millis() / 1000u), matrix_field.c_str());
}

std::string build_effects_json() {
  std::string out = "[";
  for (size_t i = 0; i < WLED_EFFECT_COUNT; i++) {
    if (i > 0)
      out += ",";
    out += "\"";
    out += json_escape(WLED_EFFECTS[i].name);
    out += "\"";
  }
  out += "]";
  return out;
}

std::string build_fxdata_json() {
  std::string out = "[";
  for (size_t i = 0; i < WLED_EFFECT_COUNT; i++) {
    if (i > 0)
      out += ",";
    const char *meta = WLED_EFFECTS[i].meta;
    const char *data = meta == nullptr ? nullptr : strchr(meta, '@');
    out += "\"";
    out += json_escape(data == nullptr ? "" : data + 1);
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
  std::string bus_ins = "[";
  size_t bus_count = c->get_bus_count();
  for (size_t i = 0; i < bus_count; i++) {
    if (i > 0)
      bus_ins += ",";
    bus_ins += string_sprintf("{\"start\":%u,\"len\":%u,\"pin\":[-1],\"order\":0,\"rev\":false,"
                              "\"skip\":0,\"type\":22,\"ref\":false,\"maxpwr\":%u,\"ledma\":%u}",
                              c->get_bus_start(i), c->get_bus_len(i), c->get_bus_max_ma(i), c->get_bus_led_ma(i));
  }
  bus_ins += "]";

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
      "\"rgbwm\":%u,"
      "\"cct\":false,"
      "\"cr\":false,"
      "\"cb\":0,"
      "\"ins\":%s"
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
      "\"sync\":{\"port0\":%u,\"port1\":%u,"
      "\"recv\":{\"en\":%s,\"grp\":%u,\"bri\":%s,\"col\":%s,\"fx\":%s,\"pal\":%s,\"opt\":%s,\"seg\":%s},"
      "\"send\":{\"en\":%s,\"dir\":%s,\"btn\":%s,\"va\":%s,\"hue\":%s,\"grp\":%u,\"ret\":%u}},"
      "\"live\":{\"en\":false,\"mso\":false,\"port\":5568},"
      "\"va\":{\"alexa\":false,\"macros\":[0,0]},"
      "\"mqtt\":{\"en\":false}"
      "},"
      "\"ol\":{\"clock\":0,\"cntdwn\":false,\"min\":0,\"max\":29},"
      "\"timers\":{\"cntdwn\":{\"goal\":[20,1,1,0,0,0],\"macro\":0},\"ins\":[]}"
      "}",
      c->get_led_count(), c->get_max_ma(), bus_count == 0 ? 0 : c->get_bus_led_ma(0), c->get_auto_white_mode(),
      bus_ins.c_str(), c->get_transition_ms() / 100u, c->get_active_preset(), c->is_on() ? "true" : "false",
      c->get_brightness(), c->get_effect_index(), c->get_params().speed, c->get_params().intensity,
      c->get_params().palette_id, c->get_udp_port(), c->get_udp_port2(), c->get_udp_receive() ? "true" : "false",
      c->get_udp_receive_groups(), c->get_udp_receive_brightness() ? "true" : "false",
      c->get_udp_receive_color() ? "true" : "false", c->get_udp_receive_effects() ? "true" : "false",
      c->get_udp_receive_palette() ? "true" : "false", c->get_udp_receive_segment_options() ? "true" : "false",
      c->get_udp_receive_segments() ? "true" : "false", c->get_udp_send() ? "true" : "false",
      c->get_udp_notify_direct() ? "true" : "false", c->get_udp_notify_button() ? "true" : "false",
      c->get_udp_notify_alexa() ? "true" : "false", c->get_udp_notify_hue() ? "true" : "false",
      c->get_udp_sync_groups(), c->get_udp_retries());
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

std::string build_pins_json() {
  // ESPHome owns GPIO allocation. Keep the WLED probe endpoint read-only and
  // empty so settings pages/tools do not infer that WLED may reconfigure pins.
  return "{\"pins\":[]}";
}

static std::string build_preset_segment_json(uint8_t id, uint16_t start, uint16_t stop, uint16_t grouping,
                                             uint16_t spacing, bool on, uint8_t opacity, const uint32_t colors[3],
                                             uint8_t fx, uint8_t sx, uint8_t ix, uint8_t pal, uint8_t c1, uint8_t c2,
                                             uint8_t c3, bool o1, bool o2, bool o3, bool reverse, bool mirror,
                                             bool selected) {
  return string_sprintf("{"
                        "\"id\":%u,"
                        "\"start\":%u,"
                        "\"stop\":%u,"
                        "\"grp\":%u,"
                        "\"spc\":%u,"
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
                        "\"sel\":%s,"
                        "\"rev\":%s,"
                        "\"mi\":%s"
                        "}",
                        id, start, stop, grouping, spacing, on ? "true" : "false", opacity, R(colors[0]), G(colors[0]),
                        B(colors[0]), W(colors[0]), R(colors[1]), G(colors[1]), B(colors[1]), W(colors[1]),
                        R(colors[2]), G(colors[2]), B(colors[2]), W(colors[2]), fx, sx, ix, pal, c1, c2, c3,
                        o1 ? "true" : "false", o2 ? "true" : "false", o3 ? "true" : "false",
                        selected ? "true" : "false", reverse ? "true" : "false", mirror ? "true" : "false");
}

std::string build_presets_json(const WLEDBridgeComponent *c) {
  std::string out = "{";
  bool first = true;
  for (uint8_t id = 1; id <= WLED_PRESET_COUNT; id++) {
    const auto *preset = c->get_preset(id);
    if (preset == nullptr || preset->valid == 0)
      continue;

    std::string name = json_escape(preset->name[0] != '\0' ? preset->name : "Preset");
    std::string seg_json = "[";
    seg_json += build_preset_segment_json(
        0, preset->segment_start, preset->segment_stop, preset->main_grouping, preset->main_spacing, preset->on != 0,
        preset->main_opacity, preset->colors, preset->effect, preset->speed, preset->intensity, preset->palette,
        preset->custom1, preset->custom2, preset->custom3, preset->check1 != 0, preset->check2 != 0,
        preset->check3 != 0, preset->reverse != 0, preset->mirror != 0, preset->main_segment == 0);
    uint8_t extra_count = std::min<uint8_t>(preset->extra_count, WLED_MAX_SEGMENTS - 1);
    for (uint8_t i = 0; i < extra_count; i++) {
      const WLEDExtraSegRecord &seg = preset->extras[i];
      seg_json += ",";
      seg_json += build_preset_segment_json(static_cast<uint8_t>(i + 1), seg.start, seg.stop, seg.grouping, seg.spacing,
                                            seg.on != 0, seg.opacity, seg.colors, seg.mode, seg.speed, seg.intensity,
                                            seg.palette, seg.custom1, seg.custom2, seg.custom3, seg.check1 != 0,
                                            seg.check2 != 0, seg.check3 != 0, seg.reverse != 0, seg.mirror != 0,
                                            preset->main_segment == i + 1);
    }
    seg_json += "]";

    std::string preset_json = string_sprintf("%s\"%u\":{\"n\":\"%s\",\"on\":%s,\"bri\":%u,\"transition\":%u,"
                                             "\"tt\":%u,\"mainseg\":%u,\"seg\":%s}",
                                             first ? "" : ",", id, name.c_str(), preset->on != 0 ? "true" : "false",
                                             preset->brightness, preset->transition_ms / 100u,
                                             preset->transition_ms / 100u, preset->main_segment, seg_json.c_str());
    if (preset->playlist_count > 0) {
      std::string playlist = ",\"playlist\":{\"ps\":[";
      for (uint8_t i = 0; i < preset->playlist_count; i++) {
        if (i > 0)
          playlist += ",";
        playlist += std::to_string(preset->playlist_presets[i]);
      }
      playlist += "],\"dur\":[";
      for (uint8_t i = 0; i < preset->playlist_count; i++) {
        if (i > 0)
          playlist += ",";
        playlist += std::to_string(preset->playlist_durations[i]);
      }
      playlist += "],\"transition\":[";
      for (uint8_t i = 0; i < preset->playlist_count; i++) {
        if (i > 0)
          playlist += ",";
        playlist += std::to_string(preset->playlist_transitions[i]);
      }
      playlist += "],\"repeat\":";
      playlist += std::to_string(preset->playlist_repeat);
      playlist += "}";
      size_t close = preset_json.rfind('}');
      if (close != std::string::npos)
        preset_json.insert(close, playlist);
    }
    out += preset_json;
    first = false;
  }
  out += "}";
  return out;
}

std::string build_live_json(const WLEDBridgeComponent *c) {
  static constexpr uint32_t MAX_LIVE_LEDS = 256;
  uint32_t used = c->get_led_count();
  uint32_t step = used == 0 ? 1 : ((used - 1u) / MAX_LIVE_LEDS) + 1u;
  std::string out = "{\"leds\":[";
  bool first = true;
  for (uint32_t i = 0; i < used; i += step) {
    uint32_t color = c->get_live_pixel_color(i);
    uint8_t r = qadd8(W(color), R(color));
    uint8_t g = qadd8(W(color), G(color));
    uint8_t b = qadd8(W(color), B(color));
    if (!first)
      out += ",";
    out += string_sprintf("\"%02X%02X%02X\"", r, g, b);
    first = false;
  }
  out += "],\"n\":";
  out += std::to_string(step);
  out += "}";
  return out;
}

static void fill_extra_record_from_json(WLEDExtraSegRecord *rec, JsonVariant seg) {
  if (rec == nullptr || seg.isNull())
    return;
  if (!seg["start"].isNull())
    rec->start = seg["start"].as<uint16_t>();
  if (!seg["stop"].isNull())
    rec->stop = seg["stop"].as<uint16_t>();
  if (!seg["grp"].isNull())
    rec->grouping = seg["grp"].as<uint16_t>();
  if (rec->grouping < 1)
    rec->grouping = 1;
  if (!seg["spc"].isNull())
    rec->spacing = seg["spc"].as<uint16_t>();
  if (!seg["on"].isNull())
    rec->on = json_bool(seg["on"]) ? 1 : 0;
  if (!seg["bri"].isNull())
    rec->opacity = json_u8(seg["bri"], rec->opacity);
  if (!seg["fx"].isNull())
    rec->mode = json_u8(seg["fx"], rec->mode);
  if (!seg["sx"].isNull())
    rec->speed = json_u8(seg["sx"], rec->speed);
  if (!seg["ix"].isNull())
    rec->intensity = json_u8(seg["ix"], rec->intensity);
  if (!seg["pal"].isNull())
    rec->palette = json_u8(seg["pal"], rec->palette);
  if (!seg["c1"].isNull())
    rec->custom1 = json_u8(seg["c1"], rec->custom1);
  if (!seg["c2"].isNull())
    rec->custom2 = json_u8(seg["c2"], rec->custom2);
  if (!seg["c3"].isNull())
    rec->custom3 = json_u8(seg["c3"], rec->custom3);
  if (!seg["o1"].isNull())
    rec->check1 = json_bool(seg["o1"]) ? 1 : 0;
  if (!seg["o2"].isNull())
    rec->check2 = json_bool(seg["o2"]) ? 1 : 0;
  if (!seg["o3"].isNull())
    rec->check3 = json_bool(seg["o3"]) ? 1 : 0;
  if (!seg["rev"].isNull())
    rec->reverse = json_bool(seg["rev"]) ? 1 : 0;
  if (!seg["mi"].isNull())
    rec->mirror = json_bool(seg["mi"]) ? 1 : 0;
  if (seg["col"].is<JsonArray>()) {
    JsonArray cols = seg["col"].as<JsonArray>();
    for (size_t i = 0; i < cols.size() && i < 3; i++)
      rec->colors[i] = json_color(cols[i], rec->colors[i]);
  }
}

static WLEDPresetRecord preset_from_json(uint8_t preset_id, JsonVariant value) {
  WLEDPresetRecord preset;
  preset.valid = 1;
  if (!value["n"].isNull())
    snprintf(preset.name, sizeof(preset.name), "%s", value["n"].as<const char *>());
  if (!value["on"].isNull())
    preset.on = json_bool(value["on"]) ? 1 : 0;
  if (!value["bri"].isNull())
    preset.brightness = json_u8(value["bri"], preset.brightness);
  if (!value["transition"].isNull())
    preset.transition_ms = transition_tenths_to_ms(value["transition"].as<uint16_t>());
  if (!value["tt"].isNull())
    preset.transition_ms = transition_tenths_to_ms(value["tt"].as<uint16_t>());
  if (!value["mainseg"].isNull())
    preset.main_segment = json_u8(value["mainseg"]);
  if (value["playlist"].is<JsonObject>()) {
    JsonObject playlist = value["playlist"].as<JsonObject>();
    JsonVariant psv = playlist["ps"];
    if (psv.isNull())
      psv = playlist["presets"];
    JsonArray ps = psv.is<JsonArray>() ? psv.as<JsonArray>() : JsonArray();
    JsonArray dur = playlist["dur"].is<JsonArray>() ? playlist["dur"].as<JsonArray>() : JsonArray();
    if (dur.isNull() && playlist["durations"].is<JsonArray>())
      dur = playlist["durations"].as<JsonArray>();
    JsonArray tr = playlist["transition"].is<JsonArray>() ? playlist["transition"].as<JsonArray>() : JsonArray();
    if (tr.isNull() && playlist["tt"].is<JsonArray>())
      tr = playlist["tt"].as<JsonArray>();
    uint16_t default_duration = playlist["dur"].is<uint16_t>() ? playlist["dur"].as<uint16_t>() : 10u;
    if (!playlist["duration"].isNull())
      default_duration = playlist["duration"].as<uint16_t>();
    uint16_t default_transition = playlist["transition"].is<uint16_t>() ? playlist["transition"].as<uint16_t>() : 7u;
    if (!playlist["tt"].isNull() && playlist["tt"].is<uint16_t>())
      default_transition = playlist["tt"].as<uint16_t>();
    for (size_t i = 0; i < ps.size() && preset.playlist_count < WLED_PLAYLIST_MAX_ENTRIES; i++) {
      uint8_t entry = json_u8(ps[i]);
      if (entry == 0 || entry == preset_id)
        continue;
      uint8_t out = preset.playlist_count++;
      preset.playlist_presets[out] = entry;
      preset.playlist_durations[out] = (!dur.isNull() && i < dur.size()) ? dur[i].as<uint16_t>() : default_duration;
      preset.playlist_transitions[out] = (!tr.isNull() && i < tr.size()) ? tr[i].as<uint16_t>() : default_transition;
    }
    if (!playlist["repeat"].isNull())
      preset.playlist_repeat = json_u8(playlist["repeat"]);
    else if (!playlist["r"].isNull())
      preset.playlist_repeat = json_u8(playlist["r"]);
  }

  auto apply_main = [&preset](JsonVariant seg) {
    WLEDExtraSegRecord tmp;
    tmp.start = preset.segment_start;
    tmp.stop = preset.segment_stop;
    tmp.grouping = preset.main_grouping;
    tmp.spacing = preset.main_spacing;
    tmp.on = preset.on;
    tmp.opacity = preset.main_opacity;
    tmp.mode = preset.effect;
    tmp.speed = preset.speed;
    tmp.intensity = preset.intensity;
    tmp.custom1 = preset.custom1;
    tmp.custom2 = preset.custom2;
    tmp.custom3 = preset.custom3;
    tmp.check1 = preset.check1;
    tmp.check2 = preset.check2;
    tmp.check3 = preset.check3;
    tmp.palette = preset.palette;
    tmp.reverse = preset.reverse;
    tmp.mirror = preset.mirror;
    tmp.colors[0] = preset.colors[0];
    tmp.colors[1] = preset.colors[1];
    tmp.colors[2] = preset.colors[2];
    fill_extra_record_from_json(&tmp, seg);
    preset.segment_start = tmp.start;
    preset.segment_stop = tmp.stop;
    preset.main_grouping = static_cast<uint8_t>(std::min<uint16_t>(tmp.grouping, 255));
    preset.main_spacing = static_cast<uint8_t>(std::min<uint16_t>(tmp.spacing, 255));
    preset.main_opacity = tmp.opacity;
    preset.effect = tmp.mode;
    preset.speed = tmp.speed;
    preset.intensity = tmp.intensity;
    preset.custom1 = tmp.custom1;
    preset.custom2 = tmp.custom2;
    preset.custom3 = tmp.custom3;
    preset.check1 = tmp.check1;
    preset.check2 = tmp.check2;
    preset.check3 = tmp.check3;
    preset.palette = tmp.palette;
    preset.reverse = tmp.reverse;
    preset.mirror = tmp.mirror;
    preset.colors[0] = tmp.colors[0];
    preset.colors[1] = tmp.colors[1];
    preset.colors[2] = tmp.colors[2];
  };

  if (value["seg"].is<JsonArray>()) {
    JsonArray arr = value["seg"].as<JsonArray>();
    uint8_t max_extra = 0;
    for (size_t i = 0; i < arr.size(); i++) {
      JsonVariant seg = arr[i];
      uint8_t id = seg["id"].isNull() ? clamp_segment_id(i) : json_u8(seg["id"]);
      if (id == 0) {
        apply_main(seg);
      } else if (id < WLED_MAX_SEGMENTS) {
        fill_extra_record_from_json(&preset.extras[id - 1], seg);
        max_extra = std::max<uint8_t>(max_extra, id);
      }
      if (!seg["sel"].isNull() && json_bool(seg["sel"]))
        preset.main_segment = id;
    }
    preset.extra_count = max_extra;
  } else if (value["seg"].is<JsonObject>()) {
    JsonVariant seg = value["seg"];
    uint8_t id = seg["id"].isNull() ? 0 : json_u8(seg["id"]);
    if (id == 0) {
      apply_main(seg);
    } else if (id < WLED_MAX_SEGMENTS) {
      fill_extra_record_from_json(&preset.extras[id - 1], seg);
      preset.extra_count = id;
    }
    if (!seg["sel"].isNull() && json_bool(seg["sel"]))
      preset.main_segment = id;
  } else {
    apply_main(value);
  }

  if (preset.name[0] == '\0')
    snprintf(preset.name, sizeof(preset.name), "Preset %u", preset_id);
  if (preset.main_segment > preset.extra_count)
    preset.main_segment = 0;
  return preset;
}

static bool parse_playlist_json(WLEDBridgeComponent *comp, JsonVariant value) {
  if (comp == nullptr || value.isNull() || !value.is<JsonObject>())
    return false;

  if (!value["on"].isNull() && !json_bool(value["on"])) {
    comp->stop_playlist();
    return true;
  }

  JsonVariant psv = value["ps"];
  if (psv.isNull())
    psv = value["presets"];
  if (!psv.is<JsonArray>())
    return false;

  JsonArray ps = psv.as<JsonArray>();
  uint8_t presets[WLED_PLAYLIST_MAX_ENTRIES]{};
  uint16_t durations[WLED_PLAYLIST_MAX_ENTRIES]{};
  uint16_t transitions[WLED_PLAYLIST_MAX_ENTRIES]{};
  uint8_t count = 0;

  JsonArray dur = value["dur"].is<JsonArray>() ? value["dur"].as<JsonArray>() : JsonArray();
  if (dur.isNull() && value["durations"].is<JsonArray>())
    dur = value["durations"].as<JsonArray>();
  JsonArray tr = value["transition"].is<JsonArray>() ? value["transition"].as<JsonArray>() : JsonArray();
  if (tr.isNull() && value["tt"].is<JsonArray>())
    tr = value["tt"].as<JsonArray>();

  uint16_t default_duration = value["dur"].is<uint16_t>() ? value["dur"].as<uint16_t>() : 10u;
  if (!value["duration"].isNull())
    default_duration = value["duration"].as<uint16_t>();
  uint16_t default_transition = value["transition"].is<uint16_t>() ? value["transition"].as<uint16_t>() : 7u;
  if (!value["tt"].isNull() && value["tt"].is<uint16_t>())
    default_transition = value["tt"].as<uint16_t>();

  for (size_t i = 0; i < ps.size() && count < WLED_PLAYLIST_MAX_ENTRIES; i++) {
    uint8_t preset_id = json_u8(ps[i]);
    if (preset_id == 0)
      continue;
    presets[count] = preset_id;
    durations[count] = (!dur.isNull() && i < dur.size()) ? dur[i].as<uint16_t>() : default_duration;
    transitions[count] = (!tr.isNull() && i < tr.size()) ? tr[i].as<uint16_t>() : default_transition;
    count++;
  }

  uint8_t repeat = 0;
  if (!value["repeat"].isNull())
    repeat = json_u8(value["repeat"]);
  else if (!value["r"].isNull())
    repeat = json_u8(value["r"]);
  return comp->start_playlist(presets, durations, transitions, count, repeat);
}

static bool parse_nightlight_json(WLEDBridgeComponent *comp, JsonVariant value) {
  if (comp == nullptr || value.isNull() || !value.is<JsonObject>())
    return false;

  uint16_t duration_min = comp->get_nightlight_duration_min();
  if (!value["dur"].isNull())
    duration_min = value["dur"].as<uint16_t>();
  else if (!value["fade"].isNull())
    duration_min = value["fade"].as<uint16_t>();

  uint8_t target = comp->get_nightlight_target_brightness();
  if (!value["tbri"].isNull())
    target = json_u8(value["tbri"], target);
  else if (!value["bri"].isNull())
    target = json_u8(value["bri"], target);

  uint8_t mode = comp->get_nightlight_mode();
  if (!value["mode"].isNull())
    mode = json_u8(value["mode"], mode);

  uint32_t duration_s = static_cast<uint32_t>(duration_min) * 60u;
  uint16_t clamped_duration_s = static_cast<uint16_t>(std::min<uint32_t>(duration_s, 65535u));
  comp->configure_nightlight(clamped_duration_s, target, mode);

  if (!value["on"].isNull()) {
    if (json_bool(value["on"]))
      comp->start_nightlight(clamped_duration_s, target, mode);
    else
      comp->stop_nightlight();
  } else if (comp->is_nightlight_active()) {
    comp->start_nightlight(clamped_duration_s, target, mode);
  }
  return true;
}

static bool parse_udpn_json(WLEDBridgeComponent *comp, JsonVariant value) {
  if (comp == nullptr || value.isNull() || !value.is<JsonObject>())
    return false;

  if (!value["send"].isNull())
    comp->set_udp_send_enabled(json_bool(value["send"]));
  else if (!value["en"].isNull())
    comp->set_udp_send_enabled(json_bool(value["en"]));

  if (!value["recv"].isNull())
    comp->set_udp_receive_enabled(json_bool(value["recv"]));

  if (!value["sgrp"].isNull())
    comp->set_udp_sync_groups(json_u8(value["sgrp"], comp->get_udp_sync_groups()));
  else if (!value["grp"].isNull())
    comp->set_udp_sync_groups(json_u8(value["grp"], comp->get_udp_sync_groups()));

  if (!value["rgrp"].isNull())
    comp->set_udp_receive_groups(json_u8(value["rgrp"], comp->get_udp_receive_groups()));

  if (!value["rb"].isNull())
    comp->set_udp_receive_brightness(json_bool(value["rb"]));
  else if (!value["bri"].isNull())
    comp->set_udp_receive_brightness(json_bool(value["bri"]));

  if (!value["rc"].isNull())
    comp->set_udp_receive_color(json_bool(value["rc"]));
  else if (!value["col"].isNull())
    comp->set_udp_receive_color(json_bool(value["col"]));

  if (!value["rx"].isNull())
    comp->set_udp_receive_effects(json_bool(value["rx"]));
  else if (!value["fx"].isNull())
    comp->set_udp_receive_effects(json_bool(value["fx"]));

  if (!value["rp"].isNull())
    comp->set_udp_receive_palette(json_bool(value["rp"]));
  else if (!value["pal"].isNull())
    comp->set_udp_receive_palette(json_bool(value["pal"]));

  if (!value["so"].isNull())
    comp->set_udp_receive_segment_options(json_bool(value["so"]));
  else if (!value["opt"].isNull())
    comp->set_udp_receive_segment_options(json_bool(value["opt"]));

  if (!value["sg"].isNull())
    comp->set_udp_receive_segments(json_bool(value["sg"]));
  else if (!value["seg"].isNull())
    comp->set_udp_receive_segments(json_bool(value["seg"]));

  if (!value["dir"].isNull())
    comp->set_udp_notify_direct(json_bool(value["dir"]));
  else if (!value["sd"].isNull())
    comp->set_udp_notify_direct(json_bool(value["sd"]));

  if (!value["btn"].isNull())
    comp->set_udp_notify_button(json_bool(value["btn"]));
  else if (!value["sb"].isNull())
    comp->set_udp_notify_button(json_bool(value["sb"]));

  if (!value["va"].isNull())
    comp->set_udp_notify_alexa(json_bool(value["va"]));
  else if (!value["sa"].isNull())
    comp->set_udp_notify_alexa(json_bool(value["sa"]));

  if (!value["hue"].isNull())
    comp->set_udp_notify_hue(json_bool(value["hue"]));
  else if (!value["sh"].isNull())
    comp->set_udp_notify_hue(json_bool(value["sh"]));

  if (!value["ret"].isNull())
    comp->set_udp_retries(json_u8(value["ret"], comp->get_udp_retries()));
  else if (!value["retries"].isNull())
    comp->set_udp_retries(json_u8(value["retries"], comp->get_udp_retries()));

  return true;
}

static bool apply_segment_pixels_json(WLEDBridgeComponent *comp, uint8_t id, JsonVariant value) {
  if (comp == nullptr || value.isNull() || !value.is<JsonArray>())
    return false;

  JsonArray arr = value.as<JsonArray>();
  if (arr.size() == 0) {
    comp->clear_pixel_overrides();
    return true;
  }

  bool changed = false;
  uint32_t start = 0;
  uint32_t stop = 0;
  uint8_t index_state = 0;  // 0 = need start, 1 = have start, 2 = have range
  for (size_t i = 0; i < arr.size(); i++) {
    JsonVariant item = arr[i];
    if (item.is<int>()) {
      uint32_t index = static_cast<uint32_t>(abs(item.as<int>()));
      if (index_state == 0) {
        start = index;
        stop = index + 1;
        index_state = 1;
      } else {
        stop = index;
        index_state = 2;
      }
      continue;
    }

    uint32_t color = 0;
    if (!json_pixel_color(item, &color))
      continue;
    if (index_state == 0)
      continue;
    if (index_state < 2 || stop <= start)
      stop = start + 1;
    for (uint32_t p = start; p < stop; p++)
      changed |= comp->set_pixel_override(id, p, color);
    index_state = 0;
  }
  return changed;
}

// Apply one WLED seg object to segment `id`. When allow_power is false (flat
// root form), on/bri are skipped because they were already consumed as the
// global power/brightness.
static void apply_segment_json(WLEDBridgeComponent *comp, uint8_t id, JsonVariant seg, bool allow_power) {
  if (seg.isNull())
    return;

  // Bounds first so a newly-created extra segment exists before other props.
  if (!seg["start"].isNull() || !seg["stop"].isNull()) {
    WLEDBridgeComponent::SegmentReadView cur;
    bool have = comp->get_segment_view(id, cur);
    uint32_t start = seg["start"].isNull() ? (have ? cur.start : 0u) : seg["start"].as<uint32_t>();
    uint32_t stop = seg["stop"].isNull() ? (have ? cur.stop : 0u) : seg["stop"].as<uint32_t>();
    comp->segment_set_bounds(id, start, stop);
  }
  if (!seg["grp"].isNull() || !seg["spc"].isNull()) {
    WLEDBridgeComponent::SegmentReadView cur;
    bool have = comp->get_segment_view(id, cur);
    uint16_t grp = seg["grp"].isNull() ? (have ? cur.grouping : 1u) : seg["grp"].as<uint16_t>();
    uint16_t spc = seg["spc"].isNull() ? (have ? cur.spacing : 0u) : seg["spc"].as<uint16_t>();
    comp->segment_set_grouping(id, grp, spc);
  }
  if (!seg["fx"].isNull()) {
    WLEDBridgeComponent::SegmentReadView sv;
    uint8_t cur_fx = comp->get_segment_view(id, sv) ? sv.mode : 0;
    uint8_t new_fx;
    if (json_incremental_u8(seg["fx"], cur_fx, static_cast<uint8_t>(WLED_EFFECT_COUNT), &new_fx))
      comp->segment_set_effect(id, new_fx);
  }
  if (!seg["sx"].isNull())
    comp->segment_set_speed(id, json_u8(seg["sx"]));
  if (!seg["ix"].isNull())
    comp->segment_set_intensity(id, json_u8(seg["ix"]));
  if (!seg["pal"].isNull()) {
    WLEDBridgeComponent::SegmentReadView sv;
    uint8_t cur_pal = comp->get_segment_view(id, sv) ? sv.palette : 0;
    uint8_t new_pal;
    if (json_incremental_u8(seg["pal"], cur_pal, static_cast<uint8_t>(WLED_PALETTE_COUNT), &new_pal))
      comp->segment_set_palette(id, new_pal);
  }
  if (!seg["c1"].isNull())
    comp->segment_set_custom(id, 1, json_u8(seg["c1"]));
  if (!seg["c2"].isNull())
    comp->segment_set_custom(id, 2, json_u8(seg["c2"]));
  if (!seg["c3"].isNull())
    comp->segment_set_custom(id, 3, json_u8(seg["c3"]));
  if (!seg["o1"].isNull())
    comp->segment_set_check(id, 1, json_bool(seg["o1"]));
  if (!seg["o2"].isNull())
    comp->segment_set_check(id, 2, json_bool(seg["o2"]));
  if (!seg["o3"].isNull())
    comp->segment_set_check(id, 3, json_bool(seg["o3"]));
  if (!seg["rev"].isNull())
    comp->segment_set_reverse(id, json_bool(seg["rev"]));
  if (!seg["mi"].isNull())
    comp->segment_set_mirror(id, json_bool(seg["mi"]));
  if (!seg["sel"].isNull()) {
    if (json_bool(seg["sel"])) {
      comp->set_main_segment(id);
    } else if (comp->get_main_segment() == id) {
      comp->set_main_segment(0);
    }
  }

  if (seg["col"].is<JsonArray>()) {
    JsonArray cols = seg["col"].as<JsonArray>();
    for (size_t i = 0; i < cols.size() && i < 3; i++) {
      if (cols[i].is<JsonArray>() && cols[i].size() >= 3) {
        uint8_t r = json_u8(cols[i][0]);
        uint8_t g = json_u8(cols[i][1]);
        uint8_t b = json_u8(cols[i][2]);
        uint8_t w = cols[i].size() >= 4 ? json_u8(cols[i][3]) : 0;
        comp->segment_set_color(id, static_cast<uint8_t>(i), RGBW32(r, g, b, w));
      } else if (cols[i].is<uint32_t>()) {
        comp->segment_set_color(id, static_cast<uint8_t>(i), cols[i].as<uint32_t>());
      }
    }
  }
  if (!seg["i"].isNull())
    apply_segment_pixels_json(comp, id, seg["i"]);

  if (allow_power) {
    if (!seg["on"].isNull())
      comp->segment_set_on(id, json_bool(seg["on"]));
    if (!seg["bri"].isNull())
      comp->segment_set_opacity(id, json_u8(seg["bri"], 255));
  }
}

// ============================================================
// WLEDJsonHandler
// ============================================================
bool WLEDJsonHandler::canHandle(web_server_idf::AsyncWebServerRequest *request) const {
  char url_buf[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
  auto url = request->url_to(url_buf);
  return strncmp(url.c_str(), "/json", 5) == 0 || strcmp(url.c_str(), "/presets.json") == 0 ||
         strcmp(url.c_str(), "/cfg.json") == 0 || strcmp(url.c_str(), "/win") == 0 ||
         strncmp(url.c_str(), "/win&", 5) == 0 || strcmp(url.c_str(), "/version") == 0 ||
         strcmp(url.c_str(), "/freeheap") == 0;
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
    } else if (request->method() == HTTP_POST) {
      handle_post_presets_(request, this->post_body_);
      this->post_body_.clear();
    } else {
      request->send(405, "application/json", "{\"error\":\"method not allowed\"}");
    }
    return;
  }

  if (strcmp(url.c_str(), "/cfg.json") == 0) {
    if (request->method() == HTTP_GET) {
      handle_get_config_(request);
    } else {
      request->send(405, "application/json", "{\"error\":\"cfg restore is managed by ESPHome\"}");
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
                       ",\"effects\":" + this->cached_effects_json_() +
                       ",\"palettes\":" + this->cached_palettes_json_() + "}";
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
  } else if (strcmp(url.c_str(), "/json/live") == 0) {
    auto body = build_live_json(comp_);
    auto *resp = request->beginResponse(200, "application/json", body);
    request->send(resp);
  } else if (strcmp(url.c_str(), "/json/effects") == 0 || strcmp(url.c_str(), "/json/eff") == 0) {
    handle_get_effects_(request);
  } else if (strcmp(url.c_str(), "/json/fxdata") == 0) {
    auto *resp = request->beginResponse(200, "application/json", this->cached_fxdata_json_());
    request->send(resp);
  } else if (strcmp(url.c_str(), "/json/palettes") == 0 || strcmp(url.c_str(), "/json/pal") == 0 ||
             strcmp(url.c_str(), "/json/palx") == 0) {
    handle_get_palettes_(request);
  } else if (strcmp(url.c_str(), "/json/cfg") == 0 || strcmp(url.c_str(), "/json/config") == 0) {
    handle_get_config_(request);
  } else if (strcmp(url.c_str(), "/json/net") == 0 || strcmp(url.c_str(), "/json/nodes") == 0) {
    handle_get_network_(request);
  } else if (strcmp(url.c_str(), "/json/pins") == 0) {
    handle_get_pins_(request);
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

const std::string &WLEDJsonHandler::cached_effects_json_() {
  if (this->effects_cache_.empty())
    this->effects_cache_ = build_effects_json();
  return this->effects_cache_;
}

const std::string &WLEDJsonHandler::cached_palettes_json_() {
  if (this->palettes_cache_.empty())
    this->palettes_cache_ = build_palettes_json();
  return this->palettes_cache_;
}

const std::string &WLEDJsonHandler::cached_fxdata_json_() {
  if (this->fxdata_cache_.empty())
    this->fxdata_cache_ = build_fxdata_json();
  return this->fxdata_cache_;
}

void WLEDJsonHandler::handle_get_effects_(web_server_idf::AsyncWebServerRequest *request) {
  auto *resp = request->beginResponse(200, "application/json", this->cached_effects_json_());
  request->send(resp);
}

void WLEDJsonHandler::handle_get_palettes_(web_server_idf::AsyncWebServerRequest *request) {
  auto *resp = request->beginResponse(200, "application/json", this->cached_palettes_json_());
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

void WLEDJsonHandler::handle_get_pins_(web_server_idf::AsyncWebServerRequest *request) {
  auto body = build_pins_json();
  auto *resp = request->beginResponse(200, "application/json", body);
  request->send(resp);
}

void WLEDJsonHandler::handle_get_presets_(web_server_idf::AsyncWebServerRequest *request) {
  auto body = build_presets_json(comp_);
  auto *resp = request->beginResponse(200, "application/json", body);
  request->send(resp);
}

void WLEDJsonHandler::handle_post_presets_(web_server_idf::AsyncWebServerRequest *request, const std::string &body) {
  if (body.empty()) {
    auto response_body = build_presets_json(comp_);
    auto *resp = request->beginResponse(200, "application/json", response_body);
    request->send(resp);
    return;
  }

  auto doc = json::parse_json(body);
  if (doc.isNull() || !doc.is<JsonObject>()) {
    request->send(400, "application/json", "{\"error\":\"invalid presets json\"}");
    return;
  }

  JsonObject root = doc.as<JsonObject>();
  JsonObject presets = root;
  if (root["presets"].is<JsonObject>())
    presets = root["presets"].as<JsonObject>();

  uint8_t imported = 0;
  uint8_t deleted = 0;
  for (JsonPair kv : presets) {
    const char *key = kv.key().c_str();
    if (key == nullptr || key[0] == '\0')
      continue;
    char *end = nullptr;
    long parsed_id = strtol(key, &end, 10);
    if (end == key || *end != '\0' || parsed_id <= 0 || parsed_id > WLED_PRESET_COUNT)
      continue;

    uint8_t preset_id = static_cast<uint8_t>(parsed_id);
    JsonVariant value = kv.value();
    if (value.isNull()) {
      if (comp_->delete_preset(preset_id))
        deleted++;
      continue;
    }
    if (!value.is<JsonObject>())
      continue;
    if (!value["delete"].isNull() && json_bool(value["delete"])) {
      if (comp_->delete_preset(preset_id))
        deleted++;
      continue;
    }

    WLEDPresetRecord preset = preset_from_json(preset_id, value);
    if (comp_->set_preset(preset_id, preset))
      imported++;
  }

  std::string response = string_sprintf("{\"success\":true,\"imported\":%u,\"deleted\":%u}", imported, deleted);
  auto *resp = request->beginResponse(200, "application/json", response);
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

  // Sync settings form compatibility (settings_sync.htm / set.cpp names).
  // These affect bridge runtime/API state; hardware/network ownership remains ESPHome.
  if (win_arg_u16(request, url, "UP", &value) && value > 0) {
    comp_->set_udp_port(value);
  }
  if (win_arg_u16(request, url, "U2", &value) && value > 0) {
    comp_->set_udp_port2(value);
  }
  if (win_arg_u16(request, url, "GS", &value)) {
    comp_->set_udp_sync_groups(static_cast<uint8_t>(value > 255 ? 255 : value));
  }
  if (win_arg_u16(request, url, "GR", &value)) {
    comp_->set_udp_receive_groups(static_cast<uint8_t>(value > 255 ? 255 : value));
  }
  if (win_arg_u16(request, url, "UR", &value)) {
    comp_->set_udp_retries(static_cast<uint8_t>(value > 29 ? 29 : value));
  }
  bool flag = false;
  if (win_arg_bool(request, url, "RB", &flag))
    comp_->set_udp_receive_brightness(flag);
  if (win_arg_bool(request, url, "RC", &flag))
    comp_->set_udp_receive_color(flag);
  if (win_arg_bool(request, url, "RX", &flag))
    comp_->set_udp_receive_effects(flag);
  if (win_arg_bool(request, url, "RP", &flag))
    comp_->set_udp_receive_palette(flag);
  if (win_arg_bool(request, url, "SO", &flag))
    comp_->set_udp_receive_segment_options(flag);
  if (win_arg_bool(request, url, "SG", &flag))
    comp_->set_udp_receive_segments(flag);
  if (win_arg_bool(request, url, "SS", &flag))
    comp_->set_udp_send_enabled(flag);
  if (win_arg_bool(request, url, "SD", &flag))
    comp_->set_udp_notify_direct(flag);
  if (win_arg_bool(request, url, "SB", &flag))
    comp_->set_udp_notify_button(flag);
  if (win_arg_bool(request, url, "SA", &flag))
    comp_->set_udp_notify_alexa(flag);
  if (win_arg_bool(request, url, "SH", &flag))
    comp_->set_udp_notify_hue(flag);

  bool use_default_nightlight_duration = win_has_arg(request, url, "ND");
  bool nightlight_changed = false;
  uint16_t nightlight_duration_min = comp_->get_nightlight_duration_min();
  uint8_t nightlight_target = comp_->get_nightlight_target_brightness();
  uint8_t nightlight_mode = comp_->get_nightlight_mode();
  if (win_arg_u16(request, url, "NT", &value)) {
    nightlight_target = static_cast<uint8_t>(value > 255 ? 255 : value);
    nightlight_changed = true;
  }
  if (win_arg_u16(request, url, "NF", &value)) {
    nightlight_mode = static_cast<uint8_t>(value > 3 ? 3 : value);
    nightlight_changed = true;
  }
  if (win_arg_u16(request, url, "NL", &value)) {
    if (value == 0) {
      comp_->stop_nightlight();
    } else {
      if (!use_default_nightlight_duration)
        nightlight_duration_min = value;
      uint32_t duration_s = static_cast<uint32_t>(nightlight_duration_min) * 60u;
      comp_->start_nightlight(static_cast<uint16_t>(std::min<uint32_t>(duration_s, 65535u)), nightlight_target,
                              nightlight_mode);
    }
    changed = true;
  } else if (use_default_nightlight_duration) {
    uint32_t duration_s = static_cast<uint32_t>(nightlight_duration_min) * 60u;
    comp_->start_nightlight(static_cast<uint16_t>(std::min<uint32_t>(duration_s, 65535u)), nightlight_target,
                            nightlight_mode);
    changed = true;
  } else if (nightlight_changed && comp_->is_nightlight_active()) {
    uint32_t duration_s = static_cast<uint32_t>(nightlight_duration_min) * 60u;
    comp_->start_nightlight(static_cast<uint16_t>(std::min<uint32_t>(duration_s, 65535u)), nightlight_target,
                            nightlight_mode);
    changed = true;
  } else if (nightlight_changed) {
    uint32_t duration_s = static_cast<uint32_t>(nightlight_duration_min) * 60u;
    comp_->configure_nightlight(static_cast<uint16_t>(std::min<uint32_t>(duration_s, 65535u)), nightlight_target,
                                nightlight_mode);
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

  if (request == nullptr)
    return;
  auto state_body = build_state_json(comp_);
  auto *resp = request->beginResponse(200, "application/json", state_body);
  request->send(resp);
}

void WLEDJsonHandler::apply_body_str(const std::string &body) {
  handle_post_state_(nullptr, body);
}

void WLEDJsonHandler::handle_post_state_(web_server_idf::AsyncWebServerRequest *request, const std::string &body) {
  if (body.empty()) {
    if (request)
      request->send(200, "application/json", "{}");
    return;
  }

  // Parse JSON using ESPHome's ArduinoJson wrapper
  auto doc = json::parse_json(body);
  if (doc.isNull()) {
    if (request)
      request->send(400, "application/json", "{\"error\":\"invalid json\"}");
    return;
  }

  // on / bri
  bool was_on = comp_->is_on();
  if (!doc["on"].isNull()) {
    if (json_on_is_toggle(doc["on"])) {
      uint8_t requested_bri = doc["bri"].isNull() ? 0 : json_u8(doc["bri"]);
      if (!was_on && requested_bri > 0) {
        comp_->set_on(true);
      } else {
        comp_->set_on(!comp_->is_on());
      }
    } else {
      comp_->set_on(json_bool(doc["on"]));
    }
  }
  if (!doc["bri"].isNull())
    comp_->set_brightness(json_u8(doc["bri"], comp_->get_brightness()));
  if (!doc["transition"].isNull())
    comp_->set_transition(transition_tenths_to_ms(doc["transition"].as<uint16_t>()));
  if (!doc["tt"].isNull())
    comp_->set_transition(transition_tenths_to_ms(doc["tt"].as<uint16_t>()));
  if (!doc["ps"].isNull()) {
    uint8_t ps_val;
    if (json_incremental_u8(doc["ps"], comp_->get_active_preset(), WLED_PRESET_COUNT + 1, &ps_val))
      comp_->load_preset(ps_val);
  }
  if (!doc["pdel"].isNull())
    comp_->delete_preset(json_u8(doc["pdel"]));
  if (!doc["np"].isNull() || (!doc["pl"].isNull() && doc["pl"].as<int>() == -2)) {
    // "np" or pl:-2 = advance to next playlist entry (WLED compatibility)
    if (comp_->is_playlist_active()) {
      uint8_t next_idx = (comp_->get_playlist_index() + 1) % comp_->get_playlist_count();
      uint8_t next_preset = comp_->get_playlist_preset(next_idx);
      if (next_preset > 0)
        comp_->load_preset(next_preset);
    }
  }
  if (!doc["pl"].isNull() && doc["pl"].as<int>() < 0 && doc["pl"].as<int>() != -2)
    comp_->stop_playlist();
  if (!doc["playlist"].isNull())
    parse_playlist_json(comp_, doc["playlist"]);
  if (!doc["nl"].isNull())
    parse_nightlight_json(comp_, doc["nl"]);
  if (!doc["udpn"].isNull())
    parse_udpn_json(comp_, doc["udpn"]);
  bool has_mainseg = !doc["mainseg"].isNull();
  uint8_t requested_mainseg = has_mainseg ? json_u8(doc["mainseg"]) : 0;

  // Segments (WLED-compatible). seg may be an array (multi-segment), a single
  // object, or — for legacy flat clients — params at the document root.
  JsonVariant segv = doc["seg"];
  if (segv.is<JsonArray>()) {
    JsonArray arr = segv.as<JsonArray>();
    for (size_t i = 0; i < arr.size(); i++) {
      JsonVariant e = arr[i];
      uint8_t id = e["id"].isNull() ? static_cast<uint8_t>(i) : json_u8(e["id"]);
      apply_segment_json(comp_, id, e, true);
    }
  } else if (segv.is<JsonObject>()) {
    uint8_t id = segv["id"].isNull() ? 0 : json_u8(segv["id"]);
    apply_segment_json(comp_, id, segv, true);
  } else {
    // Flat root form — apply to main segment but leave on/bri to the global
    // handlers above (allow_power = false).
    apply_segment_json(comp_, 0, doc, false);
  }

  if (!doc["psave"].isNull())
    comp_->save_preset(json_u8(doc["psave"]), doc["n"].isNull() ? nullptr : doc["n"].as<const char *>());
  if (has_mainseg)
    comp_->set_main_segment(requested_mainseg);
  if (!doc["win"].isNull() && doc["win"].is<const char *>()) {
    std::string win_url = "/win&";
    win_url += doc["win"].as<const char *>();
    handle_win_(nullptr, win_url);
  }

  comp_->publish_light_state();

  if (request != nullptr) {
    auto state_body = build_state_json(comp_);
    auto *resp = request->beginResponse(200, "application/json", state_body);
    request->send(resp);
  }
}

// ============================================================
// WLEDSseHandler
// ============================================================
bool WLEDSseHandler::canHandle(web_server_idf::AsyncWebServerRequest *request) const {
  char url_buf[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
  auto url = request->url_to(url_buf);
  return strcmp(url.c_str(), "/wled_events") == 0;
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

#ifdef WLED_BRIDGE_WEB_UI
// ============================================================
// WLEDUiHandler
// ============================================================

namespace {
// Map a request URL to a gzip blob and its Content-Type.
// Returns false if the URL is not a static UI asset.
bool ui_asset_for_url(const char *url, const uint8_t **data_out, size_t *size_out, const char **ctype_out) {
  struct Asset {
    const char *path;
    const uint8_t *data;
    const size_t *size;
    const char *ctype;
  };
  static const Asset ASSETS[] = {
      {"/index.js", WLED_INDEX_JS_GZ, &WLED_INDEX_JS_GZ_SIZE, "application/javascript"},
      {"/index.css", WLED_INDEX_CSS_GZ, &WLED_INDEX_CSS_GZ_SIZE, "text/css"},
      {"/rangetouch.js", WLED_RANGETOUCH_GZ, &WLED_RANGETOUCH_GZ_SIZE, "application/javascript"},
      {"/iro.js", WLED_IRO_GZ, &WLED_IRO_GZ_SIZE, "application/javascript"},
  };
  for (auto &a : ASSETS) {
    if (strcmp(url, a.path) == 0) {
      *data_out = a.data;
      *size_out = *a.size;
      *ctype_out = a.ctype;
      return true;
    }
  }
  return false;
}
}  // namespace

bool WLEDUiHandler::canHandle(web_server_idf::AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET)
    return false;
  char url_buf[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
  auto url = request->url_to(url_buf);
  const char *u = url.c_str();
  return strcmp(u, "/") == 0 || strcmp(u, "/index.htm") == 0 || strcmp(u, "/index.html") == 0 ||
         strcmp(u, "/index.js") == 0 || strcmp(u, "/index.css") == 0 || strcmp(u, "/rangetouch.js") == 0 ||
         strcmp(u, "/iro.js") == 0 || strncmp(u, "/settings", 9) == 0;
}

void WLEDUiHandler::handleRequest(web_server_idf::AsyncWebServerRequest *request) {
  char url_buf[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
  auto url = request->url_to(url_buf);
  const char *u = url.c_str();

  // Settings paths → ESPHome redirect stub
  if (strncmp(u, "/settings", 9) == 0) {
    if (WLED_SETTINGS_GZ_SIZE > 0) {
      auto *resp = request->beginResponse(200, "text/html", WLED_SETTINGS_GZ, WLED_SETTINGS_GZ_SIZE);
      resp->addHeader("Content-Encoding", "gzip");
      resp->addHeader("Cache-Control", "no-cache");
      request->send(resp);
    } else {
      request->send(200, "text/html",
                    "<html><body><h1>Settings managed by ESPHome</h1>"
                    "<p><a href='/'>Back</a></p></body></html>");
    }
    return;
  }

  // Static JS/CSS assets with long-lived cache
  const uint8_t *data;
  size_t size;
  const char *ctype;
  if (ui_asset_for_url(u, &data, &size, &ctype)) {
    if (size > 0) {
      auto *resp = request->beginResponse(200, ctype, data, size);
      resp->addHeader("Content-Encoding", "gzip");
      resp->addHeader("Cache-Control", "public, max-age=86400");
      request->send(resp);
    } else {
      request->send(404, "text/plain", "Not found");
    }
    return;
  }

  // Default: serve index.htm (/, /index.htm, /index.html)
  if (WLED_INDEX_GZ_SIZE > 0) {
    auto *resp = request->beginResponse(200, "text/html", WLED_INDEX_GZ, WLED_INDEX_GZ_SIZE);
    resp->addHeader("Content-Encoding", "gzip");
    resp->addHeader("Cache-Control", "no-cache");
    request->send(resp);
  } else {
    request->send(200, "text/html",
                  "<html><head><title>WLED Bridge</title></head><body>"
                  "<h1>WLED Bridge</h1>"
                  "<p>JSON API available at <a href='/json'>/json</a></p>"
                  "</body></html>");
  }
}
#endif  // WLED_BRIDGE_WEB_UI

}  // namespace wled_bridge
}  // namespace esphome

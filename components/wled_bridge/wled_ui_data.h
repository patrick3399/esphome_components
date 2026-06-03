#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef WLED_BRIDGE_WEB_UI

// WLED v16.0.0 UI assets embedded as gzip PROGMEM blobs.
// Injected at build time by components/wled_bridge/__init__.py when web_ui: true.
// Served paths:
//   /  /index.htm /index.html  → WLED_INDEX
//   /index.js                  → WLED_INDEX_JS
//   /index.css                 → WLED_INDEX_CSS
//   /rangetouch.js             → WLED_RANGETOUCH
//   /iro.js                    → WLED_IRO
//   /settings*                 → WLED_SETTINGS (ESPHome redirect stub)

namespace esphome {
namespace wled_bridge {

extern const uint8_t WLED_INDEX_GZ[];
extern const size_t WLED_INDEX_GZ_SIZE;

extern const uint8_t WLED_INDEX_JS_GZ[];
extern const size_t WLED_INDEX_JS_GZ_SIZE;

extern const uint8_t WLED_INDEX_CSS_GZ[];
extern const size_t WLED_INDEX_CSS_GZ_SIZE;

extern const uint8_t WLED_RANGETOUCH_GZ[];
extern const size_t WLED_RANGETOUCH_GZ_SIZE;

extern const uint8_t WLED_IRO_GZ[];
extern const size_t WLED_IRO_GZ_SIZE;

extern const uint8_t WLED_SETTINGS_GZ[];
extern const size_t WLED_SETTINGS_GZ_SIZE;

}  // namespace wled_bridge
}  // namespace esphome

#endif  // WLED_BRIDGE_WEB_UI

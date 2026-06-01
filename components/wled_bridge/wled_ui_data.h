#pragma once
#include <stdint.h>
#include <stddef.h>

namespace esphome {
namespace wled_bridge {

// Gzip'd HTML status page embedded as PROGMEM.
// Size 0 → fallback plain-text response used instead.
extern const uint8_t WLED_INDEX_GZ[];
extern const size_t WLED_INDEX_GZ_SIZE;

}  // namespace wled_bridge
}  // namespace esphome

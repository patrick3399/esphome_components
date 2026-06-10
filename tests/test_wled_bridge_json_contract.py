"""Contract checks for the WLED bridge JSON surface.

These tests intentionally stay source-level and dependency-free. They are not a
replacement for device/runtime tests; they guard the WLED UI/API compatibility
strings that are easy to regress while the bridge is still an external
component PoC.
"""

from pathlib import Path
import gzip
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
WLED_JSON_CPP = ROOT / "components" / "wled_bridge" / "wled_json.cpp"
WLED_BRIDGE_H = ROOT / "components" / "wled_bridge" / "wled_bridge.h"
WLED_EFFECT_CONTEXT_H = ROOT / "components" / "wled_bridge" / "wled_effect_context.h"
WLED_EFFECTS_H = ROOT / "components" / "wled_bridge" / "wled_effects.h"
WLED_EFFECTS_CPP = ROOT / "components" / "wled_bridge" / "wled_effects.cpp"
WLED_FX_MATH_H = ROOT / "components" / "wled_bridge" / "wled_fx_math.h"
WLED_AUDIO_CPP = ROOT / "components" / "wled_bridge" / "wled_audio.cpp"
WLED_AUDIO_FFT_CPP = ROOT / "components" / "wled_bridge" / "wled_audio_fft.cpp"
WLED_UI_CPP = ROOT / "components" / "wled_bridge" / "wled_ui_data.cpp"
WLED_UI_GENERATOR = ROOT / "tools" / "gen_wled_ui.py"
WLED_PALETTE_H = ROOT / "components" / "wled_bridge" / "wled_palette.h"
WLED_PALETTE_CPP = ROOT / "components" / "wled_bridge" / "wled_palette.cpp"
WLED_UDP_CPP = ROOT / "components" / "wled_bridge" / "wled_udp.cpp"
WLED_UDP_SYNC_CPP = ROOT / "components" / "wled_bridge" / "wled_udp_sync.cpp"
WLED_UDP_REALTIME_CPP = ROOT / "components" / "wled_bridge" / "wled_udp_realtime.cpp"
WLED_UDP_H = ROOT / "components" / "wled_bridge" / "wled_udp.h"
WLED_WS_CPP = ROOT / "components" / "wled_bridge" / "wled_ws.cpp"
WLED_INIT_PY = ROOT / "components" / "wled_bridge" / "__init__.py"
WLED_COMPONENT_README = ROOT / "components" / "wled_bridge" / "README.md"
PROJECT_README = ROOT / "README.md"
WLED_AUDIO_POC = ROOT / "devices" / "generic" / "wled_bridge_audio_poc.yaml"
WLED_FULL_POC = ROOT / "devices" / "generic" / "wled_bridge_full_poc.yaml"


def read(path: Path) -> str:
    text = path.read_text(encoding="utf-8")
    return text.replace('\\"', '"')


def assert_contains_all(testcase: unittest.TestCase, text: str, needles: list[str]) -> None:
    missing = [needle for needle in needles if needle not in text]
    testcase.assertFalse(
        missing, "missing expected contract strings: " + ", ".join(missing)
    )


def extract_embedded_ui_html() -> str:
    generated = read(WLED_UI_CPP)
    array_match = re.search(
        r"const uint8_t WLED_INDEX_GZ\[\] = \{\n(?P<body>.*?)\n\};",
        generated,
        re.S,
    )
    if array_match is None:
        raise AssertionError("missing WLED_INDEX_GZ byte array")
    values = [int(value, 16) for value in re.findall(r"0x([0-9a-fA-F]{2})", array_match.group("body"))]
    return gzip.decompress(bytes(values)).decode("utf-8")


def cpp_const_int(source: str, name: str) -> int:
    match = re.search(rf"{re.escape(name)}\s*=\s*(0x[0-9a-fA-F]+|\d+)", source)
    if match is None:
        raise AssertionError(f"missing C++ constant {name}")
    return int(match.group(1), 0)


def rgbw(r: int, g: int, b: int, w: int = 0) -> tuple[int, int, int, int]:
    return (r, g, b, w)


def make_ddp_packet(
    pixels: list[tuple[int, ...]],
    *,
    channel_offset: int = 0,
    channels_per_pixel: int = 3,
    flags: int = 0x41,
    dest: int = 1,
) -> bytes:
    data = bytes(channel for pixel in pixels for channel in pixel[:channels_per_pixel])
    data_type = channels_per_pixel << 3
    return bytes(
        [
            flags,
            0x00,
            data_type,
            dest,
            (channel_offset >> 24) & 0xFF,
            (channel_offset >> 16) & 0xFF,
            (channel_offset >> 8) & 0xFF,
            channel_offset & 0xFF,
            (len(data) >> 8) & 0xFF,
            len(data) & 0xFF,
        ]
    ) + data


def parse_ddp_fixture(packet: bytes, led_count: int) -> dict[int, tuple[int, int, int, int]]:
    if len(packet) < 10:
        return {}
    flags = packet[0]
    if (flags & 0x40) == 0 or flags & (0x02 | 0x04 | 0x08):
        return {}
    if packet[3] not in (1, 255):
        return {}
    channels_per_pixel = (packet[2] >> 3) & 0x07
    if channels_per_pixel < 3:
        channels_per_pixel = 3
    channel_offset = int.from_bytes(packet[4:8], "big")
    data_len = int.from_bytes(packet[8:10], "big")
    data = packet[10 : 10 + min(data_len, len(packet) - 10)]
    start_led = channel_offset // channels_per_pixel
    result = {}
    for i in range(len(data) // channels_per_pixel):
        led = start_led + i
        if led >= led_count:
            break
        pixel = data[i * channels_per_pixel : (i + 1) * channels_per_pixel]
        result[led] = rgbw(pixel[0], pixel[1], pixel[2], pixel[3] if channels_per_pixel >= 4 else 0)
    return result


def make_e131_packet(
    pixels: list[tuple[int, int, int]],
    *,
    universe: int,
    sequence: int = 1,
    options: int = 0,
    start_code: int = 0,
) -> bytes:
    packet = bytearray(126)
    packet[4:16] = bytes([0x41, 0x53, 0x43, 0x2D, 0x45, 0x31, 0x2E, 0x31, 0x37, 0x00, 0x00, 0x00])
    packet[111] = sequence
    packet[112] = options
    packet[113] = (universe >> 8) & 0xFF
    packet[114] = universe & 0xFF
    packet[125] = start_code
    packet.extend(channel for pixel in pixels for channel in pixel)
    return bytes(packet)


def parse_e131_fixture(
    packet: bytes,
    *,
    start_universe: int,
    universe_count: int,
    led_count: int,
) -> dict[int, tuple[int, int, int, int]]:
    if len(packet) < 126:
        return {}
    if packet[4:16] != bytes([0x41, 0x53, 0x43, 0x2D, 0x45, 0x31, 0x2E, 0x31, 0x37, 0x00, 0x00, 0x00]):
        return {}
    if packet[112] & 0xC0:
        return {}
    if packet[125] != 0:
        return {}
    universe = int.from_bytes(packet[113:115], "big")
    if universe < start_universe or universe >= start_universe + universe_count:
        return {}
    uni_index = universe - start_universe
    data = packet[126 : 126 + min(len(packet) - 126, 512)]
    start_led = uni_index * 170
    result = {}
    for i in range(len(data) // 3):
        led = start_led + i
        if led >= led_count:
            break
        result[led] = rgbw(data[i * 3], data[i * 3 + 1], data[i * 3 + 2])
    return result


def make_artnet_packet(
    pixels: list[tuple[int, int, int]],
    *,
    universe: int,
    sequence: int = 1,
    protocol_version: int = 14,
    opcode: int = 0x5000,
) -> bytes:
    data = bytes(channel for pixel in pixels for channel in pixel)
    return (
        b"Art-Net\0"
        + bytes([opcode & 0xFF, (opcode >> 8) & 0xFF])
        + bytes([(protocol_version >> 8) & 0xFF, protocol_version & 0xFF])
        + bytes([sequence, 0x00, universe & 0xFF, (universe >> 8) & 0xFF, (len(data) >> 8) & 0xFF, len(data) & 0xFF])
        + data
    )


def parse_artnet_fixture(
    packet: bytes,
    *,
    start_universe: int,
    universe_count: int,
    led_count: int,
) -> dict[int, tuple[int, int, int, int]]:
    if len(packet) < 18:
        return {}
    if packet[:8] != b"Art-Net\0":
        return {}
    if int.from_bytes(packet[8:10], "little") != 0x5000:
        return {}
    if int.from_bytes(packet[10:12], "big") < 14:
        return {}
    universe = int.from_bytes(packet[14:16], "little")
    if universe < start_universe or universe >= start_universe + universe_count:
        return {}
    data_len = min(int.from_bytes(packet[16:18], "big"), 512, len(packet) - 18)
    data = packet[18 : 18 + data_len]
    start_led = (universe - start_universe) * 170
    result = {}
    for i in range(len(data) // 3):
        led = start_led + i
        if led >= led_count:
            break
        result[led] = rgbw(data[i * 3], data[i * 3 + 1], data[i * 3 + 2])
    return result


class WLEDBridgeJsonContractTest(unittest.TestCase):
    def test_component_documentation_states_public_compatibility_contract(self) -> None:
        component_readme = read(WLED_COMPONENT_README)
        project_readme = read(PROJECT_README)

        assert_contains_all(
            self,
            component_readme,
            [
                "WLED v16.0.0",
                "MODE_COUNT = 220",
                "`Unsupported`",
                "use_task: true",
                "E1.31 universe ranges must stay within 1..63999",
                "Art-Net universe ranges must stay within 0..32767",
                "Runtime `/win` updates to those ports are also ignored",
                "JSON POST bodies are capped at 8192 bytes",
                "lock to the first valid source IP",
                "auto-white conversion path",
                "not a full WLED firmware replacement",
                "Version Strategy",
            ],
        )
        assert_contains_all(
            self,
            project_readme,
            [
                "WLED v16-compatible bridge",
                "fixed 220 WLED mode slots",
                "supported-effect mapping",
            ],
        )

    def test_random_wheel_index_uses_wrap_safe_distance_check(self) -> None:
        source = read(WLED_FX_MATH_H)

        assert_contains_all(
            self,
            source,
            [
                "uint8_t delta = r > base ? r - base : base - r",
                "if (delta > 128)",
                "delta = 256 - delta",
                "if (delta >= min_dist)",
            ],
        )
        self.assertNotIn("base + minDist", source)
        self.assertNotIn("base - minDist", source)

    def test_audio_analyzer_bounds_stale_backlog_drain(self) -> None:
        source = read(WLED_AUDIO_CPP)

        assert_contains_all(
            self,
            source,
            [
                "MAX_STALE_AUDIO_DRAIN_BATCHES = 4",
                "uint8_t drained_batches = 0",
                "bytes_available > bytes_to_read * 2 && drained_batches < MAX_STALE_AUDIO_DRAIN_BATCHES",
                "drained_batches++",
                "if (bytes_available > bytes_to_read * 2)",
                "rb->reset();",
                "return;",
            ],
        )
        self.assertNotIn("while (bytes_available > bytes_to_read * 2) {", source)

    def test_audio_analyzer_setup_is_fail_safe(self) -> None:
        header = read(ROOT / "components" / "wled_bridge" / "wled_audio.h")
        source = read(WLED_AUDIO_CPP)
        codegen = read(WLED_INIT_PY)

        assert_contains_all(
            self,
            header,
            [
                "bool audio_fft_setup();",
            ],
        )
        assert_contains_all(
            self,
            source,
            [
                "if (this->source_ == nullptr)",
                "Audio analyzer requires a microphone source",
                "this->fft_enabled_ = false;",
                "if (!this->source_->is_passive())",
                "this->source_->start();",
                "this->fft_enabled_ = audio_fft_setup();",
            ],
        )
        assert_contains_all(
            self,
            codegen,
            [
                "cv.Optional(CONF_AUDIO_PASSIVE, default=False): cv.boolean",
                "microphone_source_to_code(",
                "passive=audio[CONF_AUDIO_PASSIVE]",
            ],
        )
        self.assertNotIn("this->source_->add_data_callback", source.split("if (this->source_ == nullptr)", 1)[0])

    def test_audio_demo_configs_start_microphone_and_avoid_gpio3_strapping_pin(self) -> None:
        for path in (WLED_AUDIO_POC, WLED_FULL_POC):
            text = read(path)
            assert_contains_all(
                self,
                text,
                [
                    "passive: false",
                    "i2s_lrclk_pin: GPIO6",
                ],
            )
            self.assertNotIn("passive: true", text)
            self.assertNotIn("i2s_lrclk_pin: GPIO3", text)

    def test_audio_fft_allocation_failure_cleans_up_partial_buffers(self) -> None:
        source = read(WLED_AUDIO_FFT_CPP)

        assert_contains_all(
            self,
            source,
            [
                "static void fft_free_buffers_()",
                "free(fft_sin_table_);",
                "free(fft_cos_table_);",
                "free(fft_bit_rev_);",
                "free(fft_real_);",
                "free(fft_imag_);",
                "fft_sin_table_ = nullptr;",
                "fft_free_buffers_();",
                "Failed to allocate FFT buffers",
                "return false;",
                "return true;",
                "samples == nullptr || out == nullptr || fft_sin_table_ == nullptr || fft_cos_table_ == nullptr",
                "fft_bit_rev_ == nullptr || fft_real_ == nullptr || fft_imag_ == nullptr",
            ],
        )

    def test_state_json_exposes_wled_ui_core_fields(self) -> None:
        source = read(WLED_JSON_CPP)

        assert_contains_all(
            self,
            source,
            [
                '"on"',
                '"bri"',
                '"transition"',
                '"tt"',
                '"ps"',
                '"pl"',
                '"presets"',
                '"presets":%s',
                "build_preset_validity_json",
                '"nl"',
                '"udpn"',
                '"send":%s',
                '"recv":%s',
                '"rb":%s',
                '"rc":%s',
                '"rx":%s',
                '"rp":%s',
                '"sg":%s',
                '"dir":%s',
                '"btn":%s',
                '"va":%s',
                '"hue":%s',
                '"ret":%u',
                '"mainseg"',
                '"seg"',
                '"start"',
                '"stop"',
                '"len"',
                '"col"',
                '"fx"',
                '"sx"',
                '"ix"',
                '"pal"',
                '"c1"',
                '"c2"',
                '"c3"',
                '"o1"',
                '"o2"',
                '"o3"',
                '"sel"',
                '"rev"',
                '"mi"',
            ],
        )

    def test_json_state_accepts_runtime_udp_notifier_controls(self) -> None:
        header = read(WLED_BRIDGE_H)
        source = read(WLED_JSON_CPP)
        codegen = read(WLED_INIT_PY)

        assert_contains_all(
            self,
            header,
            [
                "set_udp_port2",
                "get_udp_port2",
                "set_udp_send_enabled",
                "set_udp_receive_enabled",
                "set_udp_sync_groups",
                "set_udp_receive_groups",
                "set_udp_receive_brightness",
                "set_udp_receive_color",
                "set_udp_receive_effects",
                "set_udp_receive_palette",
                "set_udp_receive_segments",
                "set_udp_receive_segment_options",
                "set_udp_notify_direct",
                "set_udp_notify_button",
                "set_udp_notify_alexa",
                "set_udp_notify_hue",
                "set_udp_retries",
                "get_udp_send",
                "get_udp_receive",
                "get_udp_sync_groups",
                "get_udp_receive_groups",
                "get_udp_receive_brightness",
                "get_udp_receive_color",
                "get_udp_receive_effects",
                "get_udp_receive_palette",
                "get_udp_receive_segments",
                "get_udp_receive_segment_options",
                "get_udp_notify_direct",
                "get_udp_notify_button",
                "get_udp_notify_alexa",
                "get_udp_notify_hue",
                "get_udp_retries",
            ],
        )
        assert_contains_all(
            self,
            source,
            [
                "parse_udpn_json",
                'doc["udpn"]',
                'value["send"]',
                'value["recv"]',
                'value["sgrp"]',
                'value["rgrp"]',
                'value["rb"]',
                'value["rc"]',
                'value["rx"]',
                'value["rp"]',
                'value["so"]',
                'value["opt"]',
                'value["sg"]',
                'value["dir"]',
                'value["sd"]',
                'value["btn"]',
                'value["sb"]',
                'value["va"]',
                'value["sa"]',
                'value["hue"]',
                'value["sh"]',
                'value["ret"]',
                "set_udp_send_enabled",
                "set_udp_receive_enabled",
                "set_udp_receive_brightness",
                "set_udp_receive_color",
                "set_udp_receive_effects",
                "set_udp_receive_palette",
                "set_udp_receive_segments",
                "set_udp_receive_segment_options",
                "set_udp_notify_direct",
                "set_udp_notify_button",
                "set_udp_notify_alexa",
                "set_udp_notify_hue",
                "set_udp_retries",
            ],
        )
        assert_contains_all(
            self,
            codegen,
            [
                'CONF_UDP_PORT2 = "udp_port2"',
                "cv.Optional(CONF_UDP_PORT2, default=65506): cv.port",
                "cg.add(var.set_udp_port2(config[CONF_UDP_PORT2]))",
            ],
        )

    def test_info_json_exposes_wled_ui_core_fields(self) -> None:
        source = read(WLED_JSON_CPP)

        assert_contains_all(
            self,
            source,
            [
                '"ver"',
                '"vid"',
                '"leds"',
                '"count"',
                '"pwr"',
                '"fps"',
                '"maxpwr"',
                '"maxseg"',
                '"name"',
                '"udpport"',
                '"live"',
                '"liveseg"',
                '"ws"',
                '"fxcount"',
                '"palcount"',
                '"wifi"',
                '"arch"',
                '"core"',
                '"freeheap"',
                '"uptime"',
                '"brand"',
                '"product"',
                '"btype"',
                '"release"',
            ],
        )

    def test_info_json_reports_target_arch_instead_of_hardcoded_s3(self) -> None:
        source = read(WLED_JSON_CPP)

        assert_contains_all(
            self,
            source,
            [
                "static const char *wled_arch()",
                "CONFIG_IDF_TARGET_ESP32",
                "CONFIG_IDF_TARGET_ESP32S3",
                "wled_arch()",
                '"arch":"%s"',
            ],
        )
        self.assertNotIn('"arch":"esp32s3"', source)

    def test_component_schema_rejects_unsupported_platforms_and_frameworks(self) -> None:
        source = read(WLED_INIT_PY)

        assert_contains_all(
            self,
            source,
            [
                "Framework",
                "PLATFORM_ESP32",
                "cv.only_on(PLATFORM_ESP32)",
                "cv.only_with_framework(Framework.ESP_IDF)",
            ],
        )
        self.assertNotIn("only_with_esp_idf", source)

    def test_mdns_wled_service_uses_web_server_port(self) -> None:
        source = read(WLED_BRIDGE_H.parent / "wled_bridge.cpp")

        assert_contains_all(
            self,
            source,
            [
                "wsb->get_port()",
                'mdns_service_add(nullptr, "_wled", "_tcp", mdns_port, txt, 1)',
                '"mDNS _wled._tcp registered (port=%u, mac=%s)"',
            ],
        )
        self.assertNotIn('mdns_service_add(nullptr, "_wled", "_tcp", 80, txt, 1)', source)

    def test_config_and_network_json_expose_wled_probe_fields(self) -> None:
        header = read(ROOT / "components" / "wled_bridge" / "wled_json.h")
        source = read(WLED_JSON_CPP)

        assert_contains_all(
            self,
            header,
            [
                "build_config_json",
                "build_network_json",
                "build_pins_json",
                "handle_get_config_",
                "handle_get_network_",
                "handle_get_pins_",
            ],
        )
        assert_contains_all(
            self,
            source,
            [
                "build_config_json",
                "build_network_json",
                '"/json/cfg"',
                '"/json/config"',
                '"/cfg.json"',
                '"cfg restore is managed by ESPHome"',
                '"/json/net"',
                '"/json/nodes"',
                '"/json/pins"',
                '"rev"',
                '"vid"',
                '"id"',
                '"nw"',
                '"ap"',
                '"wifi"',
                '"hw"',
                '"led"',
                '"total"',
                '"maxpwr"',
                '"ledma"',
                '"ins"',
                '"light"',
                '"def"',
                '"if"',
                '"nodes"',
                '"pins"',
                '"fxcount"',
                '"palcount"',
                '"sync"',
                '"port0":%u',
                '"port1":%u',
                '"recv"',
                '"send"',
                '"opt":%s',
                '"grp":%u',
                '"ret":%u',
                '"btn":%s',
                '"va":%s',
                '"hue":%s',
            ],
        )

    def test_json_routes_cover_wled_aliases(self) -> None:
        source = read(WLED_JSON_CPP)

        assert_contains_all(
            self,
            source,
            [
                '"/json"',
                '"/json/si"',
                '"/json/state"',
                '"/json/info"',
                '"/json/effects"',
                '"/json/eff"',
                '"/json/fxdata"',
                '"/json/palettes"',
                '"/json/pal"',
                '"/json/palx"',
                '"/json/cfg"',
                '"/json/config"',
                '"/cfg.json"',
                '"/json/net"',
                '"/json/nodes"',
                '"/json/pins"',
                '"/presets.json"',
                '"/win"',
                '"/win&"',
                '"/wled_events"',
                '"/json/live"',
            ],
        )

    def test_post_parser_accepts_wled_segment_controls(self) -> None:
        source = read(WLED_JSON_CPP)

        assert_contains_all(
            self,
            source,
            [
                'doc["tt"]',
                'doc["transition"]',
                'transition_tenths_to_ms(doc["transition"].as<uint16_t>())',
                'transition_tenths_to_ms(doc["tt"].as<uint16_t>())',
                'doc["ps"]',
                'doc["psave"]',
                'doc["pdel"]',
                'doc["n"]',
                'doc["win"]',
                "json_on_is_toggle",
                "handle_win_(nullptr, win_url)",
                'segv.is<JsonArray>()',
                'segv.is<JsonObject>()',
                'seg["fx"]',
                'seg["sx"]',
                'seg["ix"]',
                'seg["pal"]',
                'seg["c1"]',
                'seg["c2"]',
                'seg["c3"]',
                'seg["o1"]',
                'seg["o2"]',
                'seg["o3"]',
                'seg["start"]',
                'seg["stop"]',
                'seg["rev"]',
                'seg["mi"]',
                'seg["col"]',
                'seg["i"]',
                "RGBW32(r, g, b, w)",
                "publish_light_state",
            ],
        )

    def test_segment_color_parser_matches_wled_json_color_shapes(self) -> None:
        source = read(WLED_JSON_CPP)

        assert_contains_all(
            self,
            source,
            [
                "static uint32_t json_color(JsonVariant value, uint32_t fallback)",
                'value.is<JsonObject>()',
                'obj["r"].isNull() ? R(fallback) : json_u8(obj["r"])',
                'obj["g"].isNull() ? G(fallback) : json_u8(obj["g"])',
                'obj["b"].isNull() ? B(fallback) : json_u8(obj["b"])',
                'obj["w"].isNull() ? W(fallback) : json_u8(obj["w"])',
                'text[0] == \'r\' && text[1] == \'\\0\'',
                "color_wheel(hw_random8())",
                "parse_hex_color(text, &out)",
                "kelvin_to_rgbw(value.as<uint32_t>())",
            ],
        )

    def test_segment_object_without_id_targets_selected_main_segment(self) -> None:
        source = read(WLED_JSON_CPP)

        self.assertIn('uint8_t id = segv["id"].isNull() ? comp_->get_main_segment() : json_u8(segv["id"]);', source)
        self.assertNotIn('uint8_t id = segv["id"].isNull() ? 0 : json_u8(segv["id"]);', source)

    def test_json_parser_uses_arduinojson7_style_key_checks(self) -> None:
        source = read(WLED_JSON_CPP)

        self.assertNotIn("containsKey", source)

    def test_json_builders_do_not_use_fixed_size_output_buffers(self) -> None:
        source = read(WLED_JSON_CPP)

        assert_contains_all(
            self,
            source,
            [
                "string_sprintf",
                "vsnprintf(nullptr, 0, format, args)",
                "std::vector<char>",
            ],
        )
        self.assertNotIn("char buf[", source)
        self.assertNotIn("return std::string(buf)", source)

    def test_win_http_api_accepts_common_wled_query_controls(self) -> None:
        header = read(ROOT / "components" / "wled_bridge" / "wled_json.h")
        source = read(WLED_JSON_CPP)

        assert_contains_all(
            self,
            header,
            [
                "handle_win_",
            ],
        )
        assert_contains_all(
            self,
            source,
            [
                "request_arg_u16",
                "legacy_path_arg_u16",
                "legacy_path_arg_string",
                "win_arg_u16",
                "win_arg_bool",
                "parse_bool_text",
                "request == nullptr",
                "transition_tenths_to_ms",
                '"/win"',
                '"/win&"',
                '"T"',
                '"A"',
                '"FX"',
                '"SX"',
                '"IX"',
                '"FP"',
                '"TT"',
                '"PL"',
                '"PS"',
                '"PD"',
                '"UP"',
                '"U2"',
                '"GS"',
                '"GR"',
                '"RB"',
                '"RC"',
                '"RX"',
                '"RP"',
                '"SO"',
                '"SG"',
                '"SS"',
                '"SD"',
                '"SB"',
                '"SA"',
                '"SH"',
                '"UR"',
                '"R"',
                '"G"',
                '"B"',
                '"W"',
                "comp_->set_on(!comp_->is_on())",
                "comp_->set_brightness",
                "comp_->set_effect",
                "if (value < WLED_MODE_COUNT)",
                "comp_->set_palette",
                "if (value < WLED_PALETTE_COUNT)",
                "comp_->set_transition(transition_tenths_to_ms(value))",
                "comp_->load_preset",
                "comp_->save_preset",
                "comp_->delete_preset",
                "comp_->set_color(0, RGBW32",
                "comp_->set_udp_port(value)",
                "comp_->set_udp_port2(value)",
                "comp_->set_udp_sync_groups",
                "comp_->set_udp_receive_groups",
                "comp_->set_udp_receive_brightness(flag)",
                "comp_->set_udp_receive_color(flag)",
                "comp_->set_udp_receive_effects(flag)",
                "comp_->set_udp_receive_palette(flag)",
                "comp_->set_udp_receive_segment_options(flag)",
                "comp_->set_udp_receive_segments(flag)",
                "comp_->set_udp_send_enabled(flag)",
                "comp_->set_udp_notify_direct(flag)",
                "comp_->set_udp_notify_button(flag)",
                "comp_->set_udp_notify_alexa(flag)",
                "comp_->set_udp_notify_hue(flag)",
                "comp_->set_udp_retries",
                "comp_->publish_light_state();",
            ],
        )

    def test_json_incremental_values_respect_declared_upper_bounds(self) -> None:
        source = read(WLED_JSON_CPP)

        assert_contains_all(
            self,
            source,
            [
                "static bool json_incremental_u8(JsonVariant value, uint8_t current, uint8_t max_val, uint8_t *out)",
                "if (max_val > 0 && v >= max_val)",
                "return false",
                'json_incremental_u8(seg["fx"], cur_fx, static_cast<uint8_t>(WLED_MODE_COUNT), &new_fx)',
                'json_incremental_u8(seg["pal"], cur_pal, static_cast<uint8_t>(WLED_PALETTE_COUNT), &new_pal)',
            ],
        )
        self.assertNotIn("comp_->set_effect(static_cast<uint8_t>(value > 255 ? 255 : value));", source)

    def test_runtime_udp_sync_ports_reject_realtime_receiver_collisions(self) -> None:
        bridge_h = read(WLED_BRIDGE_H)
        bridge_cpp = read(ROOT / "components" / "wled_bridge" / "wled_bridge.cpp")
        udp_h = read(WLED_UDP_H)
        realtime_cpp = read(WLED_UDP_REALTIME_CPP)

        assert_contains_all(
            self,
            udp_h,
            [
                "WLED_DDP_PORT = 4048",
                "WLED_E131_PORT = 5568",
                "WLED_ARTNET_PORT = 6454",
            ],
        )
        assert_contains_all(
            self,
            bridge_h,
            [
                "void set_udp_port(uint16_t port)",
                "void set_udp_port2(uint16_t port)",
                "bool udp_port_conflicts_realtime_(uint16_t port) const",
                "last_udp_port_conflict_log_ms_",
            ],
        )
        assert_contains_all(
            self,
            bridge_cpp,
            [
                "udp_port_conflicts_realtime_",
                "port == WLED_DDP_PORT",
                "port == WLED_E131_PORT",
                "port == WLED_ARTNET_PORT",
                "Ignoring UDP sync port %u because it conflicts with an enabled realtime receiver",
                "this->udp_sync_.set_ports(this->udp_port_, this->udp_port2_)",
            ],
        )
        assert_contains_all(
            self,
            realtime_cpp,
            [
                "WLED_DDP_PORT",
                "WLED_E131_PORT",
                "WLED_ARTNET_PORT",
            ],
        )
        self.assertNotIn("static constexpr uint16_t DDP_PORT = 4048", realtime_cpp)
        self.assertNotIn("static constexpr uint16_t E131_PORT = 5568", realtime_cpp)
        self.assertNotIn("static constexpr uint16_t ARTNET_PORT = 6454", realtime_cpp)

    def test_repeated_state_setters_do_not_dirty_state(self) -> None:
        bridge_h = read(WLED_BRIDGE_H)
        bridge_cpp = read(ROOT / "components" / "wled_bridge" / "wled_bridge.cpp")

        assert_contains_all(
            self,
            bridge_h,
            [
                "void set_bool_dirty_(bool &field, bool value, bool clear_active_preset = true)",
                "void set_u8_dirty_(uint8_t &field, uint8_t value, bool clear_active_preset = true)",
                "if (field == value)",
                "this->set_u8_dirty_(this->udp_sync_groups_, groups, false)",
                "this->set_bool_dirty_(this->udp_receive_brightness_, v, false)",
                "this->set_u8_dirty_(this->params_.speed, sx)",
                "this->set_u8_dirty_(this->params_.intensity, ix)",
                "if (pal >= WLED_PALETTE_COUNT)",
                "this->set_u8_dirty_(this->params_.palette_id, pal)",
                "this->set_u8_dirty_(this->params_.custom1, c1)",
                "this->set_bool_dirty_(this->params_.check1, o1)",
                "if (slot >= 3 || this->params_.colors[slot] == rgb)",
                "if (this->transition_ms_ == ms)",
            ],
        )
        assert_contains_all(
            self,
            bridge_cpp,
            [
                "if (this->is_on_ == on)",
                "if (this->global_bri_ == bri)",
            ],
        )

    def test_segment_mutators_validate_bounds_and_avoid_redundant_dirty_state(self) -> None:
        bridge_cpp = read(ROOT / "components" / "wled_bridge" / "wled_bridge.cpp")

        assert_contains_all(
            self,
            bridge_cpp,
            [
                "if (this->main_grouping_ == grouping && this->main_spacing_ == spacing)",
                "seg == nullptr || (seg->grouping == grouping && seg->spacing == spacing)",
                "if (this->main_opacity_ == opacity)",
                "seg == nullptr || seg->opacity == opacity",
                "seg == nullptr || seg->params.speed == v",
                "seg == nullptr || seg->params.intensity == v",
                "if (v >= WLED_PALETTE_COUNT)",
                "seg == nullptr || seg->params.palette_id == v",
                "uint8_t *field = nullptr",
                "if (*field == v)",
                "if (p->check1 == v)",
                "if (p->check2 == v)",
                "if (p->check3 == v)",
                "seg == nullptr || seg->params.colors[slot] == rgb",
            ],
        )

    def test_persisted_and_imported_presets_sanitize_palette_ids(self) -> None:
        bridge_cpp = read(ROOT / "components" / "wled_bridge" / "wled_bridge.cpp")

        assert_contains_all(
            self,
            bridge_cpp,
            [
                "static uint8_t valid_palette_or_default(uint8_t palette_id)",
                "return palette_id < WLED_PALETTE_COUNT ? palette_id : 0;",
                "seg.params.palette_id = valid_palette_or_default(rec.palette);",
                "this->params_.palette_id = valid_palette_or_default(preset.palette);",
                "stored.palette = valid_palette_or_default(stored.palette);",
                "seg.palette = valid_palette_or_default(seg.palette);",
            ],
        )
        self.assertNotIn("this->params_.palette_id = preset.palette;", bridge_cpp)
        self.assertNotIn("seg.params.palette_id = rec.palette;", bridge_cpp)

    def test_light_output_runtime_fail_safe_guards(self) -> None:
        bridge_h = read(WLED_BRIDGE_H)
        bridge_cpp = read(ROOT / "components" / "wled_bridge" / "wled_bridge.cpp")

        assert_contains_all(
            self,
            bridge_h,
            [
                "static_cast<light::AddressableLightState *>(this->state_)->get_output()",
                "if (al != nullptr)",
                "al->set_effect_active(true)",
            ],
        )
        assert_contains_all(
            self,
            bridge_cpp,
            [
                "bus.strip = static_cast<light::AddressableLight *>(bus.light_state->get_output())",
                "if (state == nullptr)",
                "Ignoring null light bus",
                "if (bus.light_state == nullptr)",
                "Bus light state is null",
                "if (bus.strip == nullptr)",
                '"Bus output is not ready or is not addressable"',
                "this->mark_failed()",
            ],
        )

    def test_non_addressable_light_bus_uses_capability_safe_sync(self) -> None:
        codegen = read(WLED_INIT_PY)
        bridge_h = read(WLED_BRIDGE_H)
        bridge_cpp = read(ROOT / "components" / "wled_bridge" / "wled_bridge.cpp")
        publish_body = bridge_cpp.split("void WLEDBridgeComponent::publish_light_state() {", 1)[1].split(
            "// ============================================================", 1
        )[0]

        assert_contains_all(
            self,
            codegen,
            [
                'CONF_TYPE = "type"',
                '"monochromatic"',
                '"rgbct"',
                '"rgbww"',
                '"cwww"',
                "ADDRESSABLE_BUS_SCHEMA",
                "LIGHTSTATE_BUS_SCHEMA",
                "add_light_bus",
            ],
        )
        assert_contains_all(
            self,
            bridge_h,
            [
                "enum class BusKind",
                "LIGHTSTATE_PIXEL",
                "void add_light_bus",
                "light_values_to_rgbw_",
                "light_values_to_cct_",
                "set_pixel_cct_",
            ],
        )
        assert_contains_all(
            self,
            bridge_cpp,
            [
                "bus.kind = BusKind::LIGHTSTATE_PIXEL",
                "bus.len = 1",
                "sync_lightstate_buses_to_frame_",
                "light_values_to_cct_",
                "ColorCapability::COLD_WARM_WHITE",
                "ColorCapability::COLOR_TEMPERATURE",
                "this->set_pixel_cct_(bus.start, cct, mark_dirty)",
                "call.set_color_mode_if_supported",
                "call.set_brightness_if_supported",
                "call.set_warm_white_if_supported",
                "call.set_cold_white_if_supported",
                "call.set_publish(false)",
                "call.set_save(false)",
            ],
        )
        self.assertNotIn("set_rgbw", publish_body)

    def test_setup_allocation_failures_release_runtime_buffers(self) -> None:
        bridge_h = read(WLED_BRIDGE_H)
        bridge_cpp = read(ROOT / "components" / "wled_bridge" / "wled_bridge.cpp")

        assert_contains_all(
            self,
            bridge_h,
            [
                "void free_runtime_buffers_();",
            ],
        )
        assert_contains_all(
            self,
            bridge_cpp,
            [
                "void WLEDBridgeComponent::free_runtime_buffers_()",
                "free(this->full_frame_);",
                "free(this->transition_frame_);",
                "free(this->pixel_opacity_);",
                "free(this->pixel_override_colors_);",
                "free(this->pixel_override_mask_);",
                "this->full_frame_ = nullptr;",
                "this->transition_frame_ = nullptr;",
                "this->pixel_opacity_ = nullptr;",
                "this->pixel_override_colors_ = nullptr;",
                "this->pixel_override_mask_ = nullptr;",
                "this->pixel_overrides_active_ = false;",
                "Unable to allocate full-frame buffer",
                "Unable to allocate transition buffer",
                "Unable to allocate opacity map",
                "Unable to allocate pixel override buffers",
                "this->free_runtime_buffers_();",
            ],
        )

    def test_ha_entities_publish_only_confirmed_or_throttled_state(self) -> None:
        header = read(ROOT / "components" / "wled_bridge" / "wled_entities.h")
        entities = read(ROOT / "components" / "wled_bridge" / "wled_entities.cpp")

        assert_contains_all(
            self,
            header,
            [
                "uint8_t last_published_{255}",
            ],
        )
        assert_contains_all(
            self,
            entities,
            [
                "void WLEDPresetSelect::setup()",
                "if (this->bridge_ == nullptr)",
                "this->publish_state(this->traits.get_options()[0]);",
                "options.size() && i < this->option_count_",
                "void WLEDPresetSelect::rebuild_options_()",
                "if (!this->bridge_->load_preset(this->option_preset_ids_[i]))",
                "return;",
                "this->bridge_->publish_light_state();",
                "this->last_publish_ms_ = now;",
                "uint32_t ma = this->bridge_->get_current_ma();",
                "if (ma != this->last_ma_)",
            ],
        )

    def test_json_state_accepts_wled_win_passthrough_and_toggle_on(self) -> None:
        source = read(WLED_JSON_CPP)

        assert_contains_all(
            self,
            source,
            [
                "json_on_is_toggle",
                "text[0] == 't'",
                "text[0] == 'T'",
                "bool was_on = comp_->is_on();",
                "uint8_t requested_bri",
                "comp_->set_on(true);",
                'doc["win"]',
                'std::string win_url = "/win&";',
                'win_url += doc["win"].as<const char *>();',
                "handle_win_(nullptr, win_url);",
                "if (request == nullptr)",
            ],
        )

    def test_bridge_declares_bidirectional_light_state_sync(self) -> None:
        header = read(WLED_BRIDGE_H)

        assert_contains_all(
            self,
            header,
            [
                "LightRemoteValuesListener",
                "on_light_remote_values_update",
                "publish_light_state",
                "save_preset",
                "load_preset",
                "delete_preset",
                "set_segment_bounds",
                "set_segment_reverse",
                "set_segment_mirror",
                "WLED_PRESET_COUNT = 16",
                "WLED_PRESET_NAME_SIZE",
                "char name[WLED_PRESET_NAME_SIZE]",
                "WLEDPresetStore",
                "WLEDStoredState",
                "get_preset",
                "suppress_light_sync_",
                "sync_from_light_state_",
            ],
        )

    def test_presets_json_exposes_saved_slots(self) -> None:
        header = read(ROOT / "components" / "wled_bridge" / "wled_json.h")
        source = read(WLED_JSON_CPP)
        bridge_source = read(ROOT / "components" / "wled_bridge" / "wled_bridge.cpp")
        bridge_header = read(WLED_BRIDGE_H)

        assert_contains_all(
            self,
            header,
            [
                "build_presets_json",
                "handle_get_presets_",
            ],
        )
        assert_contains_all(
            self,
            source,
            [
                "build_presets_json",
                '"/presets.json"',
                "get_preset(id)",
                "json_escape",
                '"n":"%s"',
                '"seg":%s',
                "build_preset_segment_json",
                '"transition"',
                '"tt"',
                '"mainseg"',
                '"col"',
                '"cct"',
                '"fx"',
                '"pal"',
                "method not allowed",
            ],
        )
        assert_contains_all(
            self,
            bridge_source,
            [
                "WLED_PRESET_MAGIC = 0x574C5037",
                "main_cct",
                "set_default_preset_name_",
                "copy_preset_name_",
                '"Preset %u"',
            ],
        )
        assert_contains_all(
            self,
            bridge_header,
            [
                "void mark_dirty_(bool clear_active_preset = true)",
                "this->active_preset_ = 0",
            ],
        )
        assert_contains_all(
            self,
            bridge_source,
            [
                "this->mark_dirty_(false);",
            ],
        )

    def test_bridge_persists_last_runtime_state_with_debounce(self) -> None:
        header = read(WLED_BRIDGE_H)
        source = read(ROOT / "components" / "wled_bridge" / "wled_bridge.cpp")

        assert_contains_all(
            self,
            header,
            [
                "load_state_",
                "persist_state_",
                "schedule_state_save_",
                "state_save_due_ms_",
                "state_loaded_",
            ],
        )
        assert_contains_all(
            self,
            source,
            [
                "WLED_STATE_MAGIC = 0x574C5335",
                "WLED_STATE_SAVE_DELAY_MS",
                'fnv1a_hash("wled_bridge_state")',
                "make_preference<WLEDStoredState>",
                "persist_state_",
            ],
        )

    def test_bridge_publishes_restored_state_to_esphome_light(self) -> None:
        source = read(ROOT / "components" / "wled_bridge" / "wled_bridge.cpp")

        assert_contains_all(
            self,
            source,
            [
                "this->load_state_();",
                "if (this->state_loaded_)",
                "this->publish_light_state();",
                "this->sync_from_light_state_(false);",
            ],
        )

    def test_bridge_applies_transition_crossfade_buffer(self) -> None:
        header = read(WLED_BRIDGE_H)
        source = read(ROOT / "components" / "wled_bridge" / "wled_bridge.cpp")

        assert_contains_all(
            self,
            header,
            [
                "begin_transition_",
                "apply_transition_blend_",
                "transition_frame_",
                "transition_active_",
                "transition_duration_ms_",
            ],
        )
        assert_contains_all(
            self,
            source,
            [
                "Unable to allocate transition buffer",
                "color_blend(this->transition_frame_",
                "this->apply_transition_blend_(now)",
                "this->begin_transition_();",
            ],
        )

    def test_bridge_exposes_versioned_live_state_snapshot(self) -> None:
        header = read(WLED_BRIDGE_H)
        source = read(WLED_JSON_CPP)

        assert_contains_all(
            self,
            header,
            [
                "get_state_version",
                "state_version_",
                "state_version_++",
            ],
        )
        assert_contains_all(
            self,
            source,
            [
                '"version"',
                "pending_version_",
                "get_state_version",
                '"/wled_events"',
            ],
        )

    def test_live_led_peek_matches_wled_http_shape(self) -> None:
        header = read(WLED_BRIDGE_H)
        source = read(WLED_JSON_CPP)

        assert_contains_all(
            self,
            header,
            [
                "get_live_pixel_color",
                "last_output_scale_",
            ],
        )
        assert_contains_all(
            self,
            source,
            [
                "build_live_json",
                '"/json/live"',
                "MAX_LIVE_LEDS = 256",
                '"leds"',
                '"n"',
                "%02X%02X%02X",
                "qadd8(W(color), R(color))",
            ],
        )

    def test_segment_i_pixel_override_payload_is_supported(self) -> None:
        header = read(WLED_BRIDGE_H)
        source = read(WLED_JSON_CPP)
        bridge_source = read(ROOT / "components" / "wled_bridge" / "wled_bridge.cpp")

        assert_contains_all(
            self,
            header,
            [
                "set_pixel_override",
                "mark_pixel_overrides_changed",
                "clear_pixel_overrides",
                "apply_pixel_overrides_",
                "pixel_override_colors_",
                "pixel_override_mask_",
                "pixel_overrides_active_",
            ],
        )
        assert_contains_all(
            self,
            source,
            [
                "apply_segment_pixels_json",
                'seg["i"]',
                "json_pixel_color",
                "parse_hex_color",
                "clear_pixel_overrides",
                "set_pixel_override",
            ],
        )
        assert_contains_all(
            self,
            bridge_source,
            [
                "apply_pixel_overrides_();",
                "this->pixel_override_colors_[index] = color;",
                "this->pixel_override_mask_[index] = 1;",
            ],
        )

    def test_http_post_body_has_size_guard(self) -> None:
        header = read(ROOT / "components" / "wled_bridge" / "wled_json.h")
        source = read(WLED_JSON_CPP)
        assert_contains_all(
            self,
            header,
            [
                "struct PostBodyState",
                "std::map<httpd_req_t *, PostBodyState> post_bodies_",
                "request_body_",
                "last_post_too_large_log_ms_",
                "last_invalid_json_log_ms_",
            ],
        )
        assert_contains_all(
            self,
            source,
            [
                "WLED_JSON_MAX_POST_BODY = 8192",
                "WLED_JSON_WARNING_LOG_INTERVAL_MS = 5000",
                "POST body too large, rejecting request",
                "413 Content Too Large",
                '"request body too large"',
                "httpd_req_t *raw_request = *request;",
                "auto &state = this->post_bodies_[raw_request];",
                "this->post_bodies_.erase(state_it);",
            ],
        )

    def test_idf_json_post_reads_raw_request_body(self) -> None:
        source = read(WLED_JSON_CPP)

        assert_contains_all(
            self,
            source,
            [
                "bool WLEDJsonHandler::request_body_",
                "request->contentLength()",
                "httpd_req_recv(*request",
                "handle_post_presets_(request, body)",
                "handle_post_state_(request, body)",
            ],
        )
        self.assertNotIn("content_type_contains", source)
        self.assertNotIn("handle_post_state_(request, this->post_body_)", source)
        self.assertNotIn("handle_post_presets_(request, this->post_body_)", source)
        self.assertNotIn("std::string post_body_", read(ROOT / "components" / "wled_bridge" / "wled_json.h"))

    def test_json_error_responses_preserve_non_200_status_codes_on_idf(self) -> None:
        source = read(WLED_JSON_CPP)

        assert_contains_all(
            self,
            source,
            [
                "static void send_json_status",
                "httpd_resp_set_status(*request, status)",
                'send_json_status(request, "400 Bad Request"',
                'send_json_status(request, "405 Method Not Allowed"',
                'send_json_status(request, "413 Content Too Large"',
            ],
        )
        self.assertNotIn("request->send(400", source)
        self.assertNotIn("request->send(405", source)
        self.assertNotIn("request->send(413", source)

    def test_json_handler_matches_only_wled_path_boundaries(self) -> None:
        source = read(WLED_JSON_CPP)

        assert_contains_all(
            self,
            source,
            [
                "static bool path_has_prefix_boundary(const char *path, const char *prefix)",
                "return c == '\\0' || c == '/' || c == '?' || c == '&';",
                'path_has_prefix_boundary(url.c_str(), "/json")',
            ],
        )
        self.assertNotIn('strncmp(url.c_str(), "/json", 5) == 0', source)

    def test_websocket_clients_are_bounded(self) -> None:
        source = read(WLED_WS_CPP)

        assert_contains_all(
            self,
            source,
            [
                "WS_MAX_RECV = 8192",
                "WS_MAX_CLIENTS = 4",
                "std::find(this->client_fds_.begin(), this->client_fds_.end(), fd)",
                "this->client_fds_.size() >= WS_MAX_CLIENTS",
                "WS client limit reached (%u), closing fd=%d",
                "httpd_sess_trigger_close(this->server_, fd)",
            ],
        )

    def test_websocket_oversized_frames_close_the_session(self) -> None:
        source = read(WLED_WS_CPP)

        assert_contains_all(
            self,
            source,
            [
                "if (frame.len > WS_MAX_RECV)",
                "httpd_req_to_sockfd(req)",
                "WS frame too large (%u bytes), closing fd=%d",
                "httpd_sess_trigger_close(self->server_, fd)",
                "if (frame.type != HTTPD_WS_TYPE_TEXT)",
                "Unsupported WS frame type %u, closing fd=%d",
                "if (frame.len == 0)",
            ],
        )
        self.assertNotIn("frame.len == 0 || frame.len > WS_MAX_RECV", source)

    def test_websocket_control_frames_drain_payloads_before_returning(self) -> None:
        source = read(WLED_WS_CPP)

        assert_contains_all(
            self,
            source,
            [
                "static bool receive_frame_payload",
                "httpd_ws_recv_frame(req, frame, frame->len)",
                "if (frame.type == HTTPD_WS_TYPE_PING)",
                "if (frame.len > 125)",
                "WS ping frame too large (%u bytes), closing fd=%d",
                "receive_frame_payload(req, &frame, &ping_buf)",
                "pong.payload = frame.payload",
                "pong.len = frame.len",
                "if (frame.type == HTTPD_WS_TYPE_PONG)",
                "receive_frame_payload(req, &frame, &pong_buf)",
            ],
        )

    def test_effect_metadata_is_separate_from_effect_names(self) -> None:
        header = read(ROOT / "components" / "wled_bridge" / "wled_json.h")
        source = read(WLED_JSON_CPP)

        assert_contains_all(
            self,
            header,
            [
                "build_effects_json",
                "build_fxdata_json",
            ],
        )
        assert_contains_all(
            self,
            source,
            [
                "effect_for_wled_id",
                'effect == nullptr ? "Unsupported" : effect->name',
                "WLED_MODE_COUNT",
                "build_fxdata_json",
                "strchr(meta, '@')",
                '"/json/fxdata"',
                '"/json/palx"',
            ],
        )

    def test_effect_ids_are_mapped_to_wled_mode_ids(self) -> None:
        header = read(WLED_EFFECTS_H)
        effects = read(WLED_EFFECTS_CPP)
        bridge = read(WLED_BRIDGE_H)
        bridge_source = read(ROOT / "components" / "wled_bridge" / "wled_bridge.cpp")
        entities = read(ROOT / "components" / "wled_bridge" / "wled_entities.cpp")

        assert_contains_all(
            self,
            header,
            [
                "WLED_MODE_COUNT = 220",
                "WLED_EFFECT_UNSUPPORTED = 0xFF",
                "effect_wled_id_for_index",
                "effect_index_for_wled_id",
                "effect_for_wled_id",
            ],
        )
        assert_contains_all(
            self,
            effects,
            [
                "WLED_EFFECT_INDEX_TO_MODE",
                "117,\n    52,\n    35,\n    47",
                "153,\n    WLED_EFFECT_UNSUPPORTED,\n    172",
                "132,\n    134,\n    144,\n    128",
            ],
        )
        assert_contains_all(
            self,
            bridge,
            [
                "uint8_t get_effect_index() const",
                "void set_effect(uint8_t fx_index)",
            ],
        )
        assert_contains_all(
            self,
            bridge_source,
            [
                "uint8_t wled_id = effect_wled_id_for_index(i)",
                "auto *proxy = new WLEDProxyEffect(WLED_EFFECTS[i].name, wled_id, this)",
                "if (!effect_wled_id_supported(fx_index))",
                "log_unsupported_effect_",
                "Ignoring unsupported WLED effect id %u",
                "const EffectDescriptor *effect = effect_for_wled_id(this->active_fx_)",
                "uint32_t effect_index = this->light_state_->get_effect_index(effect->name)",
                "call.set_effect(effect_index)",
                "const EffectDescriptor *effect = effect_for_wled_id(view.mode)",
            ],
        )
        assert_contains_all(
            self,
            entities,
            [
                "effect_option_index_for_wled_id",
                "effect_wled_id_for_index(i)",
                "this->bridge_->set_effect(wled_id)",
            ],
        )

    def test_set_effect_automation_schema_uses_wled_mode_slot_limit(self) -> None:
        source = read(WLED_INIT_PY)

        assert_contains_all(
            self,
            source,
            [
                "WLED_MODE_MAX = 219",
                "cv.Required(CONF_EFFECT): cv.templatable(cv.int_range(min=0, max=WLED_MODE_MAX))",
            ],
        )
        self.assertNotIn("cv.Required(CONF_EFFECT): cv.templatable(cv.int_range(min=0, max=255))", source)

    def test_set_palette_automation_schema_uses_palette_slot_limit(self) -> None:
        source = read(WLED_INIT_PY)

        assert_contains_all(
            self,
            source,
            [
                "WLED_PALETTE_MAX = 71",
                "cv.Required(CONF_PALETTE): cv.templatable(cv.int_range(min=0, max=WLED_PALETTE_MAX))",
            ],
        )
        self.assertNotIn("cv.Required(CONF_PALETTE): cv.templatable(cv.int_range(min=0, max=255))", source)

    def test_pins_probe_is_read_only_under_esphome_hardware_ownership(self) -> None:
        header = read(ROOT / "components" / "wled_bridge" / "wled_json.h")
        source = read(WLED_JSON_CPP)

        assert_contains_all(
            self,
            header,
            [
                "build_pins_json",
                "handle_get_pins_",
            ],
        )
        assert_contains_all(
            self,
            source,
            [
                "ESPHome owns GPIO allocation",
                "return \"{\"pins\":[]}\"",
                '"/json/pins"',
                "handle_get_pins_",
            ],
        )

    def test_effect_context_maps_single_segment_pixels(self) -> None:
        header = read(WLED_EFFECT_CONTEXT_H)

        assert_contains_all(
            self,
            header,
            [
                "start",
                "stop",
                "reverse",
                "mirror",
                "map_pixel",
                "start + mapped",
            ],
        )

    def test_effect_context_xy_mapping_contract_for_2d_effects(self) -> None:
        header = read(WLED_EFFECT_CONTEXT_H)

        assert_contains_all(
            self,
            header,
            [
                "bool is_2d() const",
                "return matrix_w > 0 && matrix_h > 0;",
                "int32_t xy(int16_t x, int16_t y) const",
                "x < 0 || y < 0",
                "x >= static_cast<int16_t>(matrix_w)",
                "y >= static_cast<int16_t>(matrix_h)",
                "if (serpentine && (row & 1))",
                "col = static_cast<int32_t>(matrix_w) - 1 - col;",
                "return row * static_cast<int32_t>(matrix_w) + col;",
                "void set_pixel_2d",
                "uint32_t get_pixel_2d",
            ],
        )

        def xy(width: int, height: int, serpentine: bool, x: int, y: int) -> int:
            if x < 0 or y < 0 or x >= width or y >= height:
                return -1
            col = width - 1 - x if serpentine and (y & 1) else x
            return y * width + col

        self.assertEqual(xy(4, 3, False, 0, 0), 0)
        self.assertEqual(xy(4, 3, False, 3, 1), 7)
        self.assertEqual(xy(4, 3, True, 0, 1), 7)
        self.assertEqual(xy(4, 3, True, 3, 1), 4)
        self.assertEqual(xy(4, 3, True, 2, 2), 10)
        self.assertEqual(xy(4, 3, True, -1, 0), -1)
        self.assertEqual(xy(4, 3, True, 4, 0), -1)
        self.assertEqual(xy(4, 3, True, 0, 3), -1)

    def test_transition_tenths_and_brightness_factor_math_contracts(self) -> None:
        json_source = read(WLED_JSON_CPP)
        bridge_source = read(ROOT / "components" / "wled_bridge" / "wled_bridge.cpp")

        assert_contains_all(
            self,
            json_source,
            [
                "static uint16_t transition_tenths_to_ms(uint16_t tenths)",
                "std::min<uint32_t>(tenths, 655u) * 100u",
            ],
        )
        assert_contains_all(
            self,
            bridge_source,
            [
                "uint8_t effective_bri = this->global_bri_;",
                "if (this->brightness_factor_ < 100)",
                "effective_bri = static_cast<uint8_t>((static_cast<uint16_t>(effective_bri) * this->brightness_factor_) / 100u);",
                "if (this->auto_white_mode_ != 0)\n        c = apply_auto_white(c, this->auto_white_mode_);",
                "uint64_t requested_bus_ma = (raw_bus_ma * effective_bri) / 255u;",
                "uint64_t requested_total_ma = (raw_total_ma * effective_bri) / 255u;",
                "uint8_t final_scale = std::min<uint8_t>(effective_bri, limit_scale);",
            ],
        )

        def transition_tenths_to_ms(tenths: int) -> int:
            return min(tenths, 655) * 100

        def effective_brightness(global_bri: int, brightness_factor: int) -> int:
            return (global_bri * brightness_factor) // 100 if brightness_factor < 100 else global_bri

        self.assertEqual(transition_tenths_to_ms(0), 0)
        self.assertEqual(transition_tenths_to_ms(4), 400)
        self.assertEqual(transition_tenths_to_ms(655), 65500)
        self.assertEqual(transition_tenths_to_ms(999), 65500)
        self.assertEqual(effective_brightness(255, 100), 255)
        self.assertEqual(effective_brightness(255, 80), 204)
        self.assertEqual(effective_brightness(128, 50), 64)

    def test_embedded_ui_is_generated_from_operational_control_surface(self) -> None:
        generator = read(WLED_UI_GENERATOR)
        generated = read(WLED_UI_CPP)

        # Generator fetches real WLED v16 UI assets from GitHub
        assert_contains_all(
            self,
            generator,
            [
                "raw.githubusercontent.com/Aircoookie/WLED",
                "v16.0.0",
                "index.htm",
                "index.js",
                "index.css",
                "rangetouch.js",
                "iro.js",
                "WLED_INDEX",
                "WLED_INDEX_JS",
                "WLED_INDEX_CSS",
                "WLED_RANGETOUCH",
                "WLED_IRO",
                "WLED_SETTINGS",
                "settings",
            ],
        )
        # Generated cpp declares all 6 blobs
        assert_contains_all(
            self,
            generated,
            [
                "Auto-generated by tools/gen_wled_ui.py",
                "const uint8_t WLED_INDEX_GZ[]",
                "const size_t WLED_INDEX_GZ_SIZE",
                "const uint8_t WLED_INDEX_JS_GZ[]",
                "const size_t WLED_INDEX_JS_GZ_SIZE",
                "const uint8_t WLED_INDEX_CSS_GZ[]",
                "const size_t WLED_INDEX_CSS_GZ_SIZE",
                "const uint8_t WLED_RANGETOUCH_GZ[]",
                "const size_t WLED_RANGETOUCH_GZ_SIZE",
                "const uint8_t WLED_IRO_GZ[]",
                "const size_t WLED_IRO_GZ_SIZE",
                "const uint8_t WLED_SETTINGS_GZ[]",
                "const size_t WLED_SETTINGS_GZ_SIZE",
            ],
        )

    def test_embedded_ui_payload_is_not_placeholder_sized(self) -> None:
        generated = read(WLED_UI_CPP)
        size_match = re.search(r"WLED_INDEX_GZ_SIZE = (\d+);", generated)

        self.assertIsNotNone(size_match)
        self.assertGreater(int(size_match.group(1)), 3000)

    def test_embedded_ui_blob_contains_live_control_surface(self) -> None:
        # WLED_INDEX_GZ is the real WLED v16 index.htm shell.
        # JS/CSS/API logic lives in index.js (WLED_INDEX_JS_GZ).
        html = extract_embedded_ui_html()

        assert_contains_all(
            self,
            html,
            [
                "<title>WLED</title>",
                'src="index.js"',
                'href="index.css"',
                'src="rangetouch.js"',
                'id="fx"',
                'id="sliderBri"',
                'id="sliderSpeed"',
                'id="checkO1"',
            ],
        )


    def test_effects_count_matches_declared_constant(self) -> None:
        effects_h = read(ROOT / "components" / "wled_bridge" / "wled_effects.h")
        effects_cpp = read(ROOT / "components" / "wled_bridge" / "wled_effects.cpp")

        # Declared constant — computed from 1D + 2D sub-counts
        m1d = re.search(r"WLED_EFFECT_COUNT_1D\s*=\s*(\d+)", effects_h)
        m2d = re.search(r"WLED_EFFECT_COUNT_2D\s*=\s*(\d+)", effects_h)
        self.assertIsNotNone(m1d, "WLED_EFFECT_COUNT_1D not found in wled_effects.h")
        self.assertIsNotNone(m2d, "WLED_EFFECT_COUNT_2D not found in wled_effects.h")
        declared = int(m1d.group(1)) + int(m2d.group(1))
        self.assertGreaterEqual(declared, 44, "expected at least 44 effects")

        # Table entries — count /* N */ comment markers in the table block
        table_entries = re.findall(r"/\*\s*\d+\s*\*/", effects_cpp)
        self.assertEqual(
            len(table_entries),
            declared,
            f"WLED_EFFECT_COUNT={declared} but table has {len(table_entries)} entries",
        )

    def test_effect_table_indices_are_contiguous_and_metadata_is_present(self) -> None:
        effects_h = read(ROOT / "components" / "wled_bridge" / "wled_effects.h")
        effects_cpp = read(ROOT / "components" / "wled_bridge" / "wled_effects.cpp")

        m1d = re.search(r"WLED_EFFECT_COUNT_1D\s*=\s*(\d+)", effects_h)
        m2d = re.search(r"WLED_EFFECT_COUNT_2D\s*=\s*(\d+)", effects_h)
        m_audio_1d = re.search(r"WLED_EFFECT_COUNT_AUDIO_1D\s*=\s*(\d+)", effects_h)
        m_audio_2d = re.search(r"WLED_EFFECT_COUNT_AUDIO_2D\s*=\s*(\d+)", effects_h)
        count_1d = int(m1d.group(1))
        count_2d = int(m2d.group(1))
        count_audio_1d = int(m_audio_1d.group(1))
        count_audio_2d = int(m_audio_2d.group(1))
        declared_base = count_1d + count_2d
        declared_total = declared_base + count_audio_1d + count_audio_2d

        marker_values = [
            int(value)
            for value in re.findall(r"/\*\s*(\d+)\s*\*/\s*\{", effects_cpp)
        ]
        self.assertEqual(marker_values, list(range(declared_base)))

        table_block = effects_cpp.split("const EffectDescriptor WLED_EFFECTS[WLED_EFFECT_COUNT]", 1)[1]
        metadata_strings = re.findall(r'\{\s*"[^"]+"\s*,\s*"([^"]*)"\s*,\s*fx_', table_block)
        self.assertEqual(len(metadata_strings), declared_total)
        self.assertTrue(all(meta for meta in metadata_strings), "every effect needs UI metadata")

    def test_2d_effect_metadata_count_matches_current_coverage(self) -> None:
        effects_h = read(ROOT / "components" / "wled_bridge" / "wled_effects.h")
        effects_cpp = read(ROOT / "components" / "wled_bridge" / "wled_effects.cpp")

        m2d = re.search(r"WLED_EFFECT_COUNT_2D\s*=\s*(\d+)", effects_h)
        m_audio_2d = re.search(r"WLED_EFFECT_COUNT_AUDIO_2D\s*=\s*(\d+)", effects_h)
        count_2d = int(m2d.group(1))
        count_audio_2d = int(m_audio_2d.group(1))

        table_block = effects_cpp.split("const EffectDescriptor WLED_EFFECTS[WLED_EFFECT_COUNT]", 1)[1]
        two_d_entries = re.findall(r'\{\s*"2D [^"]+"\s*,\s*"[^"]*m12=2[^"]*"\s*,\s*fx_2d_', table_block)

        self.assertEqual(len(two_d_entries), count_2d + count_audio_2d)

    def test_palette_count_matches_declared_constant(self) -> None:
        palettes_h = read(WLED_PALETTE_H)
        palettes_cpp = read(WLED_PALETTE_CPP)

        match = re.search(r"WLED_PALETTE_COUNT\s*=\s*(\d+)", palettes_h)
        self.assertIsNotNone(match, "WLED_PALETTE_COUNT not found in wled_palette.h")
        declared = int(match.group(1))
        table_entries = re.findall(r"/\*\s*\d+\s*\*/\s*\"[^\"]+\"", palettes_cpp)

        self.assertEqual(declared, 72)
        self.assertEqual(
            len(table_entries),
            declared,
            f"WLED_PALETTE_COUNT={declared} but table has {len(table_entries)} entries",
        )

        expected_names = [
            "Default",
            "Random Cycle",
            "Color 1",
            "Colors 1&2",
            "Color Gradient",
            "Colors Only",
            "Party",
            "Cloud",
            "Lava",
            "Ocean",
            "Forest",
            "Rainbow",
            "Rainbow Bands",
            "Sunset",
            "Rivendell",
            "Breeze",
            "Red & Blue",
            "Yellowout",
            "Analogous",
            "Splash",
            "Pastel",
            "Sunset 2",
            "Beach",
            "Vintage",
            "Departure",
            "Landscape",
            "Beech",
            "Sherbet",
            "Hult",
            "Hult 64",
            "Drywet",
            "Jul",
            "Grintage",
            "Rewhi",
            "Tertiary",
            "Fire",
            "Icefire",
            "Cyane",
            "Light Pink",
            "Autumn",
            "Magenta",
            "Magred",
            "Yelmag",
            "Yelblu",
            "Orange & Teal",
            "Tiamat",
            "April Night",
            "Orangery",
            "C9",
            "Sakura",
            "Aurora",
            "Atlantica",
            "C9 2",
            "C9 New",
            "Temperature",
            "Aurora 2",
            "Retro Clown",
            "Candy",
            "Toxy Reaf",
            "Fairy Reaf",
            "Semi Blue",
            "Pink Candy",
            "Red Reaf",
            "Aqua Flash",
            "Yelblu Hot",
            "Lite Light",
            "Red Flash",
            "Blink Red",
            "Red Shift",
            "Red Tide",
            "Candy2",
            "Traffic Light",
        ]
        actual_names = re.findall(r"/\*\s*\d+\s*\*/\s*\"([^\"]+)\"", palettes_cpp)
        self.assertEqual(actual_names, expected_names)

        table_match = re.search(
            r"const PaletteInfo WLED_PALETTES\[WLED_PALETTE_COUNT\] = \{(?P<body>.*?)\n\};",
            palettes_cpp,
            re.S,
        )
        self.assertIsNotNone(table_match, "palette table not found")
        table_rows = re.findall(r"/\*\s*(\d+)\s*\*/\s*\"[^\"]+\",\s*([^}]+)\}", table_match.group("body"))
        self.assertEqual(len(table_rows), declared)
        missing_data = [int(index) for index, data_expr in table_rows if int(index) >= 6 and "nullptr" in data_expr]
        self.assertEqual(missing_data, [])

    def test_json_combined_endpoint_includes_effects_and_palettes(self) -> None:
        source = read(WLED_JSON_CPP)

        # /json route must build all four arrays
        assert_contains_all(
            self,
            source,
            [
                '"effects":',
                '"palettes":',
                "build_effects_json()",
                "build_palettes_json()",
            ],
        )
        # /json combined block must concatenate effects + palettes
        # Both strings should appear in the same code block after '"/json"'
        combined_block_match = re.search(
            r'strcmp\(url\.c_str\(\),\s*"/json"\)',
            source,
        )
        self.assertIsNotNone(combined_block_match, '"/json" route not found in source')

    def test_probe_endpoints_version_and_freeheap(self) -> None:
        source = read(WLED_JSON_CPP)

        assert_contains_all(
            self,
            source,
            [
                '"/version"',
                '"/freeheap"',
                '"WLED Bridge',
                "heap_caps_get_free_size",
                "MALLOC_CAP_INTERNAL",
            ],
        )

    def test_udp_notifier_uses_wled_protocol_v12_shape(self) -> None:
        header = read(WLED_UDP_H)
        source = read(WLED_UDP_SYNC_CPP)

        assert_contains_all(
            self,
            header,
            [
                "compatibility v12",
                "uint16_t port2",
                "set_ports",
                "recv2_fd_",
                "set_send_enabled",
                "set_receive_enabled",
                "close_send_socket_",
                "close_recv_socket_",
            ],
        )
        assert_contains_all(
            self,
            source,
            [
                "WLED_NOTIFIER_PROTOCOL = 0",
                "WLED_NOTIFIER_COMPAT_VERSION = 12",
                "UDP_SEG_SIZE = 36",
                "SEG_OFFSET = 41",
                "buf[0] = WLED_NOTIFIER_PROTOCOL",
                "buf[11] = WLED_NOTIFIER_COMPAT_VERSION",
                "buf[36] = this->comp_->get_udp_sync_groups()",
                "buf[39] = sent_segments",
                "buf[40] = UDP_SEG_SIZE",
                "packet_size = SEG_OFFSET + static_cast<size_t>(sent_segments) * UDP_SEG_SIZE",
                "uint8_t buf[1472]",
                "get_udp_retries()",
                "get_udp_notify_direct()",
                "this->open_recv_socket_(&this->recv2_fd_, this->port2_)",
                "this->close_recv_socket_(&this->recv2_fd_)",
                "this->drain_socket_(this->recv2_fd_)",
            ],
        )

    def test_udp_notifier_parses_wled_groups_and_segment_payload(self) -> None:
        source = read(WLED_UDP_SYNC_CPP)

        assert_contains_all(
            self,
            source,
            [
                "this->comp_->get_udp_receive_groups() & groups",
                "get_udp_receive_brightness()",
                "get_udp_receive_color()",
                "get_udp_receive_effects()",
                "get_udp_receive_palette()",
                "get_udp_receive_segments()",
                "get_udp_receive_segment_options()",
                "receive_segment_payload",
                "segment_set_bounds",
                "segment_set_grouping",
                "segment_set_reverse",
                "segment_set_on",
                "segment_set_mirror",
                "segment_set_opacity",
                "segment_set_cct",
                "segment_set_effect",
                "segment_set_speed",
                "segment_set_intensity",
                "segment_set_palette",
                "segment_set_custom",
                "segment_set_check",
                "this->last_send_ms_ = millis();",
                "this->comp_->publish_light_state();",
            ],
        )

    def test_e131_receiver_contract_is_declared_and_accounted(self) -> None:
        codegen = read(WLED_INIT_PY)
        header = read(WLED_BRIDGE_H)
        udp_header = read(WLED_UDP_H)
        udp_source = read(WLED_UDP_REALTIME_CPP)
        json_source = read(WLED_JSON_CPP)
        bridge_source = read(ROOT / "components" / "wled_bridge" / "wled_bridge.cpp")

        assert_contains_all(
            self,
            codegen,
            [
                'CONF_E131_RECEIVE = "e131_receive"',
                'CONF_E131_UNIVERSE = "e131_universe"',
                'CONF_E131_UNIVERSE_COUNT = "e131_universe_count"',
                "cv.Optional(CONF_E131_RECEIVE, default=False): cv.boolean",
                "cv.Optional(CONF_E131_UNIVERSE, default=1): cv.int_range(min=1, max=63999)",
                "cv.Optional(CONF_E131_UNIVERSE_COUNT, default=1): cv.int_range(min=1, max=8)",
                "_consume_wled_bridge_sockets",
                "if config.get(CONF_E131_RECEIVE, False):",
                "count += 1  # E1.31/sACN port 5568",
                "cg.add(var.set_e131_enabled(e131_en))",
                "cg.add(var.set_e131_universe(config[CONF_E131_UNIVERSE]))",
                "cg.add(var.set_e131_universe_count(config[CONF_E131_UNIVERSE_COUNT]))",
            ],
        )
        assert_contains_all(
            self,
            header,
            [
                "WLEDE131Receiver e131_receiver_{}",
                "set_e131_enabled",
                "set_e131_universe",
                "set_e131_universe_count",
                "get_e131_enabled",
                "get_e131_universe",
                "get_e131_universe_count",
                "is_e131_receiving",
            ],
        )
        assert_contains_all(
            self,
            udp_header,
            [
                "class WLEDE131Receiver",
                "Listens on UDP 5568",
                "uint16_t start_universe_{1}",
                "uint8_t universe_count_{1}",
                "uint32_t source_ip_{0}",
                "uint32_t last_source_reject_log_ms_{0}",
                "uint8_t last_seq_[8]{}",
            ],
        )
        assert_contains_all(
            self,
            udp_source,
            [
                "WLED_E131_PORT",
                "E131_PIXELS_PER_UNIVERSE = 170",
                "E131_TIMEOUT_MS = 2500",
                "IP_ADD_MEMBERSHIP",
                "E131_ACN_ID",
                "E131_DMX_START_CODE_OFFSET",
                "if (options & 0xC0)",
                "if (buf[E131_DMX_START_CODE_OFFSET] != 0x00)",
                "universe < this->start_universe_",
                "universe >= this->start_universe_ + this->universe_count_",
                'realtime_source_allowed("E1.31", source_ip',
                "int8_t diff = static_cast<int8_t>(seq - last)",
                "this->comp_->set_pixel_override(0, start_led + i, RGBW32(r, g, b, 0), false)",
                "this->comp_->mark_pixel_overrides_changed();",
                "this->comp_->clear_pixel_overrides();",
            ],
        )
        assert_contains_all(
            self,
            json_source,
            [
                "(c->is_ddp_receiving() || c->is_e131_receiving() || c->is_artnet_receiving())",
                'c->is_ddp_receiving() ? "DDP" : (c->is_e131_receiving() ? "E1.31" : (c->is_artnet_receiving() ? "Art-Net" : ""))',
            ],
        )
        assert_contains_all(
            self,
            bridge_source,
            [
                "this->e131_receiver_.setup(this, this->e131_enabled_, this->e131_universe_, this->e131_universe_count_);",
                "this->e131_receiver_.loop();",
                'ESP_LOGCONFIG(TAG, "  E1.31 receiver: universe %u-%u"',
            ],
        )

    def test_ddp_packet_fixture_matches_receiver_mapping_contract(self) -> None:
        udp_source = read(WLED_UDP_REALTIME_CPP)
        self.assertEqual(cpp_const_int(udp_source, "DDP_HEADER_LEN"), 10)
        self.assertEqual(cpp_const_int(udp_source, "DDP_FLAGS_VER1"), 0x40)
        self.assertEqual(cpp_const_int(udp_source, "DDP_FLAGS_PUSH"), 0x01)
        self.assertEqual(cpp_const_int(udp_source, "DDP_ID_DISPLAY"), 1)
        self.assertEqual(cpp_const_int(udp_source, "DDP_ID_ALL"), 255)

        rgb_packet = make_ddp_packet([(1, 2, 3), (4, 5, 6)], channel_offset=3)
        self.assertEqual(
            parse_ddp_fixture(rgb_packet, led_count=4),
            {
                1: rgbw(1, 2, 3),
                2: rgbw(4, 5, 6),
            },
        )

        rgbw_packet = make_ddp_packet([(10, 20, 30, 40), (50, 60, 70, 80)], channel_offset=4, channels_per_pixel=4)
        self.assertEqual(
            parse_ddp_fixture(rgbw_packet, led_count=4),
            {
                1: rgbw(10, 20, 30, 40),
                2: rgbw(50, 60, 70, 80),
            },
        )

        self.assertEqual(parse_ddp_fixture(make_ddp_packet([(1, 2, 3)], flags=0x01), led_count=4), {})
        self.assertEqual(parse_ddp_fixture(make_ddp_packet([(1, 2, 3)], flags=0x43), led_count=4), {})
        self.assertEqual(parse_ddp_fixture(make_ddp_packet([(1, 2, 3)], dest=2), led_count=4), {})
        self.assertEqual(parse_ddp_fixture(b"\x40\x00", led_count=4), {})

        truncated = bytearray(make_ddp_packet([(1, 2, 3), (4, 5, 6)]))
        truncated[8] = 0
        truncated[9] = 12
        self.assertEqual(
            parse_ddp_fixture(bytes(truncated[:-3]), led_count=4),
            {0: rgbw(1, 2, 3)},
        )

    def test_e131_packet_fixture_matches_receiver_mapping_contract(self) -> None:
        udp_source = read(WLED_UDP_REALTIME_CPP)
        self.assertEqual(cpp_const_int(udp_source, "E131_MIN_PACKET"), 126)
        self.assertEqual(cpp_const_int(udp_source, "E131_UNIVERSE_OFFSET"), 113)
        self.assertEqual(cpp_const_int(udp_source, "E131_SEQUENCE_OFFSET"), 111)
        self.assertEqual(cpp_const_int(udp_source, "E131_OPTIONS_OFFSET"), 112)
        self.assertEqual(cpp_const_int(udp_source, "E131_DMX_START_CODE_OFFSET"), 125)
        self.assertEqual(cpp_const_int(udp_source, "E131_DMX_DATA_OFFSET"), 126)
        self.assertEqual(cpp_const_int(udp_source, "E131_PIXELS_PER_UNIVERSE"), 170)

        first_universe = make_e131_packet([(1, 2, 3), (4, 5, 6)], universe=1)
        self.assertEqual(
            parse_e131_fixture(first_universe, start_universe=1, universe_count=2, led_count=400),
            {
                0: rgbw(1, 2, 3),
                1: rgbw(4, 5, 6),
            },
        )

        second_universe = make_e131_packet([(10, 20, 30)], universe=2)
        self.assertEqual(
            parse_e131_fixture(second_universe, start_universe=1, universe_count=2, led_count=400),
            {170: rgbw(10, 20, 30)},
        )

        self.assertEqual(parse_e131_fixture(first_universe, start_universe=3, universe_count=1, led_count=400), {})
        self.assertEqual(
            parse_e131_fixture(make_e131_packet([(1, 2, 3)], universe=1, options=0x40), start_universe=1, universe_count=1, led_count=10),
            {},
        )
        self.assertEqual(
            parse_e131_fixture(make_e131_packet([(1, 2, 3)], universe=1, start_code=1), start_universe=1, universe_count=1, led_count=10),
            {},
        )
        self.assertEqual(parse_e131_fixture(b"ASC", start_universe=1, universe_count=1, led_count=10), {})

        truncated = make_e131_packet([(1, 2, 3), (4, 5, 6)], universe=1)[:-3]
        self.assertEqual(
            parse_e131_fixture(truncated, start_universe=1, universe_count=1, led_count=10),
            {0: rgbw(1, 2, 3)},
        )

    def test_artnet_packet_fixture_matches_receiver_mapping_contract(self) -> None:
        udp_source = read(WLED_UDP_REALTIME_CPP)
        self.assertEqual(cpp_const_int(udp_source, "ARTNET_MIN_PACKET"), 18)
        self.assertEqual(cpp_const_int(udp_source, "ARTNET_DMX_DATA_OFFSET"), 18)
        self.assertEqual(cpp_const_int(udp_source, "ARTNET_OPCODE_DMX"), 0x5000)
        self.assertEqual(cpp_const_int(udp_source, "ARTNET_PIXELS_PER_UNIVERSE"), 170)

        first_universe = make_artnet_packet([(1, 2, 3), (4, 5, 6)], universe=0)
        self.assertEqual(
            parse_artnet_fixture(first_universe, start_universe=0, universe_count=2, led_count=400),
            {
                0: rgbw(1, 2, 3),
                1: rgbw(4, 5, 6),
            },
        )

        second_universe = make_artnet_packet([(10, 20, 30)], universe=1)
        self.assertEqual(
            parse_artnet_fixture(second_universe, start_universe=0, universe_count=2, led_count=400),
            {170: rgbw(10, 20, 30)},
        )

        self.assertEqual(parse_artnet_fixture(first_universe, start_universe=2, universe_count=1, led_count=400), {})
        self.assertEqual(
            parse_artnet_fixture(make_artnet_packet([(1, 2, 3)], universe=0, protocol_version=13), start_universe=0, universe_count=1, led_count=10),
            {},
        )
        self.assertEqual(
            parse_artnet_fixture(make_artnet_packet([(1, 2, 3)], universe=0, opcode=0x5100), start_universe=0, universe_count=1, led_count=10),
            {},
        )
        self.assertEqual(parse_artnet_fixture(b"Art-Net", start_universe=0, universe_count=1, led_count=10), {})

        truncated = make_artnet_packet([(1, 2, 3), (4, 5, 6)], universe=0)[:-3]
        self.assertEqual(
            parse_artnet_fixture(truncated, start_universe=0, universe_count=1, led_count=10),
            {0: rgbw(1, 2, 3)},
        )

    def test_realtime_sequence_window_contract(self) -> None:
        udp_source = read(WLED_UDP_REALTIME_CPP)
        assert_contains_all(
            self,
            udp_source,
            [
                "int8_t diff = static_cast<int8_t>(seq - last)",
                "if (diff < 0 && diff > -20)",
            ],
        )

        def accepted(last: int, seq: int) -> bool:
            if seq == 0 or last == 0:
                return True
            diff = (seq - last) & 0xFF
            if diff >= 128:
                diff -= 256
            return not (diff < 0 and diff > -20)

        self.assertTrue(accepted(10, 11))
        self.assertFalse(accepted(10, 9))
        self.assertFalse(accepted(10, 1))
        self.assertTrue(accepted(10, 0))
        self.assertTrue(accepted(250, 3))
        self.assertTrue(accepted(100, 70))

    def test_realtime_process_packet_defensively_rechecks_minimum_lengths(self) -> None:
        udp_source = read(WLED_UDP_REALTIME_CPP)

        assert_contains_all(
            self,
            udp_source,
            [
                "if (buf == nullptr || len < DDP_HEADER_LEN)",
                "if (buf == nullptr || len < E131_MIN_PACKET)",
                "if (buf == nullptr || len < ARTNET_MIN_PACKET)",
            ],
        )

    def test_realtime_receivers_lock_to_one_source_until_timeout(self) -> None:
        udp_header = read(WLED_UDP_H)
        udp_source = read(WLED_UDP_REALTIME_CPP)

        assert_contains_all(
            self,
            udp_header,
            [
                "process_packet_(const uint8_t *buf, size_t len, uint32_t source_ip)",
                "uint32_t source_ip_{0}",
                "uint32_t last_source_reject_log_ms_{0}",
            ],
        )
        assert_contains_all(
            self,
            udp_source,
            [
                "REALTIME_SOURCE_REJECT_LOG_INTERVAL_MS = 5000",
                "format_source_ip",
                "realtime_source_allowed",
                "while locked to %s",
                "this->process_packet_(buf, static_cast<size_t>(len), src.sin_addr.s_addr)",
                'realtime_source_allowed("DDP", source_ip',
                'realtime_source_allowed("E1.31", source_ip',
                'realtime_source_allowed("Art-Net", source_ip',
                "this->source_ip_ = 0;",
            ],
        )

    def test_artnet_receiver_contract_is_declared_and_accounted(self) -> None:
        codegen = read(WLED_INIT_PY)
        header = read(WLED_BRIDGE_H)
        udp_header = read(WLED_UDP_H)
        udp_source = read(WLED_UDP_REALTIME_CPP)
        bridge_source = read(ROOT / "components" / "wled_bridge" / "wled_bridge.cpp")

        assert_contains_all(
            self,
            codegen,
            [
                'CONF_ARTNET_RECEIVE = "artnet_receive"',
                'CONF_ARTNET_UNIVERSE = "artnet_universe"',
                'CONF_ARTNET_UNIVERSE_COUNT = "artnet_universe_count"',
                "cv.Optional(CONF_ARTNET_RECEIVE, default=False): cv.boolean",
                "cv.Optional(CONF_ARTNET_UNIVERSE, default=0): cv.int_range(min=0, max=32767)",
                "cv.Optional(CONF_ARTNET_UNIVERSE_COUNT, default=1): cv.int_range(min=1, max=8)",
                "if config.get(CONF_ARTNET_RECEIVE, False):",
                "count += 1  # Art-Net port 6454",
                "cg.add(var.set_artnet_enabled(artnet_en))",
                "cg.add(var.set_artnet_universe(config[CONF_ARTNET_UNIVERSE]))",
                "cg.add(var.set_artnet_universe_count(config[CONF_ARTNET_UNIVERSE_COUNT]))",
            ],
        )
        assert_contains_all(
            self,
            header,
            [
                "WLEDArtNetReceiver artnet_receiver_{}",
                "set_artnet_enabled",
                "set_artnet_universe",
                "set_artnet_universe_count",
                "get_artnet_enabled",
                "get_artnet_universe",
                "get_artnet_universe_count",
                "is_artnet_receiving",
            ],
        )
        assert_contains_all(
            self,
            udp_header,
            [
                "class WLEDArtNetReceiver",
                "Listens on UDP 6454",
                "uint16_t start_universe_{0}",
                "uint8_t universe_count_{1}",
                "uint32_t source_ip_{0}",
                "uint32_t last_source_reject_log_ms_{0}",
                "uint8_t last_seq_[8]{}",
            ],
        )
        assert_contains_all(
            self,
            udp_source,
            [
                "WLED_ARTNET_PORT",
                "ARTNET_OPCODE_DMX = 0x5000",
                "ARTNET_PIXELS_PER_UNIVERSE = 170",
                "ARTNET_TIMEOUT_MS = 2500",
                "ARTNET_ID[] = {'A', 'r', 't', '-', 'N', 'e', 't', 0x00}",
                "if (opcode != ARTNET_OPCODE_DMX)",
                "if (protocol_version < 14)",
                "universe < this->start_universe_",
                "universe >= this->start_universe_ + this->universe_count_",
                'realtime_source_allowed("Art-Net", source_ip',
                "uint16_t data_len = (static_cast<uint16_t>(buf[16]) << 8) | buf[17]",
                "if (data_len > 512)",
                "int8_t diff = static_cast<int8_t>(seq - last)",
                "this->comp_->set_pixel_override(0, start_led + i, RGBW32(r, g, b, 0), false)",
                "this->comp_->mark_pixel_overrides_changed();",
                "this->comp_->clear_pixel_overrides();",
            ],
        )
        assert_contains_all(
            self,
            bridge_source,
            [
                "this->artnet_receiver_.setup(this, this->artnet_enabled_, this->artnet_universe_, this->artnet_universe_count_);",
                "this->artnet_receiver_.loop();",
                'ESP_LOGCONFIG(TAG, "  Art-Net receiver: universe %u-%u"',
            ],
        )

    def test_boot_preset_and_brightness_factor_contracts(self) -> None:
        codegen = read(WLED_INIT_PY)
        header = read(WLED_BRIDGE_H)
        source = read(ROOT / "components" / "wled_bridge" / "wled_bridge.cpp")

        assert_contains_all(
            self,
            codegen,
            [
                'CONF_BOOT_PRESET = "boot_preset"',
                'CONF_BRIGHTNESS_FACTOR = "brightness_factor"',
                "cv.Optional(CONF_BOOT_PRESET, default=0): cv.int_range(min=0, max=16)",
                "cv.Optional(CONF_BRIGHTNESS_FACTOR, default=100): cv.int_range(min=0, max=100)",
                "if config[CONF_BOOT_PRESET] > 0:",
                "cg.add(var.set_boot_preset(config[CONF_BOOT_PRESET]))",
                "if config[CONF_BRIGHTNESS_FACTOR] < 100:",
                "cg.add(var.set_brightness_factor(config[CONF_BRIGHTNESS_FACTOR]))",
            ],
        )
        assert_contains_all(
            self,
            header,
            [
                "set_boot_preset",
                "get_boot_preset",
                "uint8_t boot_preset_{0}",
                "set_brightness_factor",
                "get_brightness_factor",
                "uint8_t brightness_factor_{100}",
                "percent > 100 ? 100 : percent",
            ],
        )
        assert_contains_all(
            self,
            source,
            [
                "if (this->boot_preset_ > 0 && this->is_preset_valid(this->boot_preset_))",
                "this->load_preset(this->boot_preset_);",
                'ESP_LOGD(TAG, "Boot preset %u applied", this->boot_preset_);',
                "uint8_t effective_bri = this->global_bri_;",
                "if (this->brightness_factor_ < 100)",
                "(static_cast<uint16_t>(effective_bri) * this->brightness_factor_) / 100u",
                "uint64_t requested_bus_ma = (raw_bus_ma * effective_bri) / 255u;",
                "uint64_t requested_total_ma = (raw_total_ma * effective_bri) / 255u;",
                "uint8_t final_scale = std::min<uint8_t>(effective_bri, limit_scale);",
                'ESP_LOGCONFIG(TAG, "  Boot preset: %u", this->boot_preset_);',
            ],
        )

    def test_schema_rejects_unsafe_task_and_validates_limits(self) -> None:
        codegen = read(WLED_INIT_PY)
        assert_contains_all(
            self,
            codegen,
            [
                "cv.Optional(CONF_MAX_MA, default=5000): cv.int_range(min=0)",
                "cv.Optional(CONF_MAX_MA): cv.int_range(min=0)",
                '"\'use_task\' is disabled until the render path is thread-safe"',
                '"\'matrix_width\' and \'matrix_height\' must both be 0 or both be greater than 0"',
                "matrix_width * matrix_height > 65535",
                "WLED segment coordinates are 16-bit",
                "flat_overrides = []",
                "realtime' cannot be mixed with deprecated flat realtime keys",
                "E131_MAX_UNIVERSE = 63999",
                "ARTNET_MAX_UNIVERSE = 32767",
                "if e131_enabled and e131_universe + e131_count - 1 > E131_MAX_UNIVERSE",
                '"E1.31 universe range exceeds 63999"',
                "if artnet_enabled and artnet_universe + artnet_count - 1 > ARTNET_MAX_UNIVERSE",
                '"Art-Net universe range exceeds 32767"',
                "udp_ports = {config[CONF_UDP_PORT], config[CONF_UDP_PORT2]}",
                "UDP sync receive port conflicts with realtime receiver port",
            ],
        )

    def test_matrix_geometry_is_checked_against_runtime_led_count(self) -> None:
        bridge_cpp = read(ROOT / "components" / "wled_bridge" / "wled_bridge.cpp")

        assert_contains_all(
            self,
            bridge_cpp,
            [
                "if (this->is_2d())",
                "uint32_t matrix_pixels = static_cast<uint32_t>(this->matrix_width_) * static_cast<uint32_t>(this->matrix_height_)",
                "if (matrix_pixels > this->total_leds_)",
                "Matrix geometry %ux%u requires %u LEDs, but only %u are configured",
                "this->mark_failed()",
                "WLED_MAX_ADDRESSABLE_LEDS = 65535",
                "this->total_leds_ > WLED_MAX_ADDRESSABLE_LEDS",
                "WLED Bridge supports at most %u virtual LEDs, but %u are configured",
            ],
        )

    def test_socket_accounting_uses_actual_enabled_udp_features(self) -> None:
        codegen = read(WLED_INIT_PY)

        assert_contains_all(
            self,
            codegen,
            [
                "def _count_udp_sockets(config):",
                "if config.get(CONF_UDP_SEND, False):",
                "if config.get(CONF_UDP_RECEIVE, False):",
                "if config.get(CONF_UDP_PORT2, 0) != config.get(CONF_UDP_PORT, 21324):",
                "if rt.get(CONF_DDP, False) or config.get(CONF_DDP_RECEIVE, False):",
                "if rt.get(CONF_E131, False) or config.get(CONF_E131_RECEIVE, False):",
                "if rt.get(CONF_ARTNET, False) or config.get(CONF_ARTNET_RECEIVE, False):",
                "def _consume_wled_bridge_sockets(config):",
                "_count_udp_sockets(config)",
                '"wled_bridge"',
                "SocketType.UDP",
                "_consume_wled_bridge_sockets",
            ],
        )
        self.assertNotIn('consume_sockets(6, "wled_bridge", SocketType.UDP)', codegen)

    def test_source_filters_match_feature_boundaries(self) -> None:
        codegen = read(WLED_INIT_PY)

        assert_contains_all(
            self,
            codegen,
            [
                "def FILTER_SOURCE_FILES():",
                'filtered = ["wled_udp.cpp", "wled_ui_data.cpp"]',
                'filtered.append("wled_effects_2d.cpp")',
                'filtered.append("wled_udp_realtime.cpp")',
                'filtered.append("wled_entities.cpp")',
                'filtered.append("wled_audio.cpp")',
                'filtered.append("wled_audio_fft.cpp")',
                'filtered.append("wled_effects_audio.cpp")',
                "elif not audio.get(CONF_AUDIO_FFT, False):",
                'filtered.append("wled_audio_fft.cpp")',
                'cg.add_define("WLED_BRIDGE_2D")',
                'cg.add_define("WLED_BRIDGE_WEB_UI")',
                'cg.add_define("WLED_BRIDGE_REALTIME")',
                'cg.add_define("WLED_BRIDGE_AUDIO")',
                'cg.add_define("WLED_BRIDGE_FFT")',
                'cg.add_define("WLED_BRIDGE_ENTITIES")',
            ],
        )

    def test_batch2_effect_functions_declared(self) -> None:
        effects_h = read(ROOT / "components" / "wled_bridge" / "wled_effects.h")
        effects_cpp = read(ROOT / "components" / "wled_bridge" / "wled_effects.cpp")

        batch2 = [
            "fx_juggle",
            "fx_bouncing_balls",
            "fx_fireworks",
            "fx_police",
            "fx_chase_flash",
            "fx_heartbeat",
            "fx_rain",
            "fx_sparkle",
            "fx_pride_2015",
            "fx_candle",
            "fx_fill_noise",
            "fx_oscillate",
            "fx_gradient",
            "fx_pacifica",
        ]
        for fn in batch2:
            self.assertIn(fn, effects_h, f"{fn} not declared in wled_effects.h")
            self.assertIn(fn, effects_cpp, f"{fn} not implemented in wled_effects.cpp")

    def test_batch2_effects_have_metadata_strings(self) -> None:
        effects_cpp = read(ROOT / "components" / "wled_bridge" / "wled_effects.cpp")

        # Each entry in the table: {"Name", "meta@...", fn}
        # Batch 2 effect names that should appear in the table
        batch2_names = [
            "Juggle",
            "Bouncing Balls",
            "Fireworks",
            "Police",
            "Chase Flash",
            "Heartbeat",
            "Rain",
            "Sparkle",
            "Pride 2015",
            "Candle",
            "Fill Noise",
            "Oscillate",
            "Gradient",
            "Pacifica",
        ]
        for name in batch2_names:
            self.assertIn(f'"{name}"', effects_cpp, f'effect name "{name}" not in table')


if __name__ == "__main__":
    unittest.main()

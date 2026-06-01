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
WLED_UI_CPP = ROOT / "components" / "wled_bridge" / "wled_ui_data.cpp"
WLED_UI_GENERATOR = ROOT / "tools" / "gen_wled_ui.py"


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


class WLEDBridgeJsonContractTest(unittest.TestCase):
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

    def test_config_and_network_json_expose_wled_probe_fields(self) -> None:
        header = read(ROOT / "components" / "wled_bridge" / "wled_json.h")
        source = read(WLED_JSON_CPP)

        assert_contains_all(
            self,
            header,
            [
                "build_config_json",
                "build_network_json",
                "handle_get_config_",
                "handle_get_network_",
            ],
        )
        assert_contains_all(
            self,
            source,
            [
                "build_config_json",
                "build_network_json",
                '"/json/cfg"',
                '"/json/net"',
                '"/json/nodes"',
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
                '"fxcount"',
                '"palcount"',
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
                '"/json/palettes"',
                '"/json/pal"',
                '"/json/cfg"',
                '"/json/net"',
                '"/json/nodes"',
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
                'doc["seg"].is<JsonArray>()',
                'doc["seg"].is<JsonObject>()',
                'seg0["bri"]',
                'seg0["fx"]',
                'seg0["sx"]',
                'seg0["ix"]',
                'seg0["pal"]',
                'seg0["c1"]',
                'seg0["c2"]',
                'seg0["c3"]',
                'seg0["o1"]',
                'seg0["o2"]',
                'seg0["o3"]',
                'seg0["start"]',
                'seg0["stop"]',
                'seg0["rev"]',
                'seg0["mi"]',
                'seg0["col"]',
                "RGBW32(r, g, b, w)",
                "publish_light_state",
            ],
        )

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
                "win_arg_u16",
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
                '"R"',
                '"G"',
                '"B"',
                '"W"',
                "comp_->set_on(!comp_->is_on())",
                "comp_->set_brightness",
                "comp_->set_effect",
                "comp_->set_palette",
                "comp_->set_transition(transition_tenths_to_ms(value))",
                "comp_->load_preset",
                "comp_->save_preset",
                "comp_->delete_preset",
                "comp_->set_color(0, RGBW32",
                "comp_->publish_light_state();",
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
                '"seg":[{',
                '"transition"',
                '"tt"',
                '"col"',
                '"fx"',
                '"pal"',
                "method not allowed",
            ],
        )
        assert_contains_all(
            self,
            bridge_source,
            [
                "WLED_PRESET_MAGIC = 0x574C5033",
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
                "WLED_STATE_MAGIC = 0x574C5332",
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

    def test_embedded_ui_is_generated_from_operational_control_surface(self) -> None:
        generator = read(WLED_UI_GENERATOR)
        generated = read(WLED_UI_CPP)

        assert_contains_all(
            self,
            generator,
            [
                "/json/si",
                "/json/eff",
                "/json/pal",
                "/presets.json",
                "/json/state",
                "/wled_events",
                "liveVersion",
                "refreshPresets",
                "await refreshPresets()",
                "pollLive",
                "renderPresets",
                "Array.from({ length: 16 }, () => false)",
                "presetName",
                "payload.n",
                "attr(presetName)",
                "postPreset",
                "psave",
                "pdel",
                "segStart",
                "segStop",
                "segRev",
                "segMirror",
                "visibleSliders",
                "numberValue",
                "collectPayload",
                "color-row",
                "Brightness",
                "Transition",
                "transitionV",
                "Effect",
                "Palette",
            ],
        )
        assert_contains_all(
            self,
            generated,
            [
                "Auto-generated by tools/gen_wled_ui.py",
                "const uint8_t WLED_INDEX_GZ[]",
                "const size_t WLED_INDEX_GZ_SIZE",
            ],
        )

    def test_embedded_ui_payload_is_not_placeholder_sized(self) -> None:
        generated = read(WLED_UI_CPP)
        size_match = re.search(r"WLED_INDEX_GZ_SIZE = (\d+);", generated)

        self.assertIsNotNone(size_match)
        self.assertGreater(int(size_match.group(1)), 3000)

    def test_embedded_ui_blob_contains_live_control_surface(self) -> None:
        html = extract_embedded_ui_html()

        assert_contains_all(
            self,
            html,
            [
                '<input id="bri"',
                '<input id="transition"',
                '<select id="fx">',
                '<select id="pal">',
                '<input id="segStart"',
                '<input id="segStop"',
                '<input id="segRev" type="checkbox">',
                '<input id="segMirror" type="checkbox">',
                'api("/json/si")',
                'api("/json/eff")',
                'api("/json/pal")',
                'api("/presets.json")',
                "refreshPresets",
                "await refreshPresets()",
                'api("/wled_events")',
                'method: "POST"',
                "presetName",
                "payload.n",
                'tt: Math.round(Number($("transition").value) / 100)',
                'psave',
                'pdel',
                "Array.from({ length: 16 }, () => false)",
            ],
        )


if __name__ == "__main__":
    unittest.main()

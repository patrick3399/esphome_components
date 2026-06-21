import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


class ComponentReliabilityContractsTest(unittest.TestCase):
    def test_qmi8658_ctrl9_poll_is_non_blocking(self) -> None:
        source = read("components/qmi8658/qmi8658.cpp")
        self.assertIn("void QMI8658Component::poll_ctrl9_cmd_()", source)
        self.assertIn(
            "this->set_timeout(1, [this]() { this->poll_ctrl9_cmd_(); });",
            source,
        )
        self.assertNotIn("for (int i = 0; i < 50; i++)", source)
        self.assertNotIn("delay(1);", source)

    def test_jiecang_bounds_uart_work_and_commands(self) -> None:
        source = read("components/jiecang_desk_controller/jiecang_desk_controller.cpp")
        self.assertIn("RX_BYTES_PER_LOOP = 256", source)
        self.assertIn("while (budget-- > 0 && this->available())", source)
        self.assertIn(
            "height_cm = std::max(this->limit_min_, std::min(height_cm, this->limit_max_));",
            source,
        )
        self.assertIn(
            "pct = std::max(0.0f, std::min(pct, 100.0f));",
            source,
        )
        self.assertIn("const size_t expected_len = 3u + param_cnt;", source)
        self.assertIn("expected_len > this->rx_buf_.size()", source)
        self.assertNotIn("static_cast<uint8_t>(3 + param_cnt)", source)

    def test_camera_frames_handle_allocation_failure(self) -> None:
        for component in ("amg8833", "mcu90640"):
            source = read(f"components/{component}/{component}.cpp")
            header = read(f"components/{component}/{component}.h")
            with self.subTest(component=component):
                self.assertIn("RAMAllocator<uint8_t> allocator;", source)
                self.assertIn("if (!image->valid())", source)
                self.assertIn("Failed to allocate %u-byte camera frame", source)
                self.assertIn("bool valid() const", header)

    def test_nrf52_baseline_does_not_claim_wifi(self) -> None:
        config = read("devices/generic/nrf52_generic.yaml")
        self.assertNotIn("\nwifi:", config)
        self.assertNotIn("\napi:", config)
        self.assertNotIn("\nota:", config)

    def test_i2s_override_documentation_matches_schema(self) -> None:
        readme = read("components/i2s_audio/README.md")
        schema = read("components/i2s_audio/speaker/__init__.py")
        self.assertIn("volume_multiplier", readme)
        self.assertNotIn("output_gain", readme)
        self.assertIn('CONF_VOLUME_MULTIPLIER: Final = "volume_multiplier"', schema)
        self.assertIn(
            "max_sample_rate=config.get(CONF_SAMPLE_RATE),\n        )(config)",
            schema,
        )

    def test_camera_jpeg_quality_matches_encoder_contract(self) -> None:
        for component in ("amg8833", "mcu90640"):
            schema = read(f"components/{component}/__init__.py")
            readme = read(f"components/{component}/README.md")
            with self.subTest(component=component):
                self.assertIn("min=1, max=100", schema)
                self.assertIn("JPEG quality accepts 1–100", readme)
                self.assertIn("PROFILE_DEFAULTS", schema)

    def test_camera_requesters_use_enum_bit_positions(self) -> None:
        for component in ("amg8833", "mcu90640"):
            source = read(f"components/{component}/{component}.cpp")
            with self.subTest(component=component):
                self.assertGreaterEqual(source.count("(1U << requester)"), 3)
                self.assertNotIn(
                    "& static_cast<uint8_t>(requester)",
                    source,
                )
                self.assertNotIn(
                    "|= static_cast<uint8_t>(requester)",
                    source,
                )

    def test_camera_frame_memory_is_bounded(self) -> None:
        for component in ("amg8833", "mcu90640"):
            source = read(f"components/{component}/{component}.cpp")
            header = read(f"components/{component}/{component}.h")
            with self.subTest(component=component):
                self.assertIn("this->current_image_.use_count() > 1", source)
                self.assertIn("resize_jpeg_buffer_", source)
                self.assertIn("jpeg_buf_max_size_", header)
                self.assertIn("release_buffers_();", source)

    def test_camera_profiles_require_explicit_psram_guarantee(self) -> None:
        for component in ("amg8833", "mcu90640"):
            schema = read(f"components/{component}/__init__.py")
            readme = read(f"components/{component}/README.md")
            with self.subTest(component=component):
                self.assertIn("psram.is_guaranteed()", schema)
                self.assertIn("PROFILE_LOW_MEMORY", schema)
                self.assertIn("PROFILE_BALANCED", schema)
                self.assertIn("PROFILE_HIGH_QUALITY", schema)
                self.assertIn(
                    "psram_guaranteed and profile != PROFILE_LOW_MEMORY", schema
                )
                self.assertIn("ignore_not_found: false", readme)

    def test_aw9523_keeps_shadows_in_sync_with_successful_writes(self) -> None:
        source = read("components/aw9523/aw9523.cpp")
        for failure in (
            "Failed to reset AW9523",
            "Failed to disable AW9523 interrupts",
            "Failed to initialize AW9523 GPIO mode",
            "Failed to configure AW9523 global control",
            "Failed to read AW9523 output latches",
            "Failed to initialize AW9523 pin directions",
        ):
            self.assertIn(failure, source)
        self.assertIn("uint8_t new_mask = mask;", source)
        self.assertIn("mask = new_mask;", source)

    def test_readmes_cover_non_inherited_component_controls(self) -> None:
        expected = {
            "esp_sr_wake_word": (
                "microphone",
                "stop_after_detection",
                "models",
                "wake_word",
                "internal",
                "on_wake_word_detected",
                "esp_sr_wake_word.start",
                "esp_sr_wake_word.stop",
                "esp_sr_wake_word.enable_model",
                "esp_sr_wake_word.disable_model",
                "esp_sr_wake_word.is_running",
                "esp_sr_wake_word.model_is_enabled",
            ),
            "i2s_audio": (
                "i2s_lrclk_pin",
                "i2s_bclk_pin",
                "i2s_mclk_pin",
                "adc_type",
                "i2s_din_pin",
                "pdm",
                "channel",
                "sample_rate",
                "bits_per_sample",
                "i2s_mode",
                "use_apll",
                "mclk_multiple",
                "correct_dc_offset",
                "dac_type",
                "i2s_dout_pin",
                "i2s_comm_fmt",
                "spdif_mode",
                "buffer_duration",
                "timeout",
                "volume_multiplier",
            ),
            "wled_bridge": (
                "light_id",
                "buses",
                "type",
                "max_ma",
                "led_ma",
                "use_task",
                "auto_white",
                "matrix_width",
                "matrix_height",
                "matrix_serpentine",
                "udp_port",
                "udp_port2",
                "udp_send",
                "udp_receive",
                "boot_preset",
                "brightness_factor",
                "web_ui",
                "realtime",
                "ddp",
                "e131",
                "e131_universe",
                "e131_universes",
                "artnet",
                "artnet_universe",
                "artnet_universes",
                "audio",
                "microphone",
                "passive",
                "fft",
                "agc",
                "palette_select",
                "effect_select",
                "speed_number",
                "intensity_number",
                "preset_select",
                "nightlight_switch",
                "sync_send_switch",
                "sync_receive_switch",
                "estimated_current",
            ),
            "ys_irtm_uart": (
                "on_ir_receive",
                "ys_irtm_uart_id",
                "ys_irtm_uart.send_raw",
                "ys_irtm_uart.send_nec",
                "ys_irtm_uart.send_proxy_packet",
                "ys_irtm_uart.set_address",
                "ys_irtm_uart.set_baudrate",
            ),
        }
        for component, controls in expected.items():
            readme = read(f"components/{component}/README.md")
            for control in controls:
                with self.subTest(component=component, control=control):
                    self.assertIn(control, readme)

    def test_ys_irtm_validates_supported_uart_settings(self):
        source = read("components/ys_irtm_uart/__init__.py")
        cpp = read("components/ys_irtm_uart/ys_irtm_uart.cpp")
        self.assertIn("require_tx=True", source)
        self.assertIn("require_rx=True", source)
        self.assertIn("*BAUD_RATE_OPTIONS", source)
        self.assertIn("check_uart_settings(this->parent_->get_baud_rate())", cpp)
        self.assertNotIn("check_uart_settings(9600)", cpp)


if __name__ == "__main__":
    unittest.main()

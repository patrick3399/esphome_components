import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


class ComponentReliabilityContractsTest(unittest.TestCase):
    def test_bmi270_checks_critical_i2c_writes(self) -> None:
        source = read("components/bmi270/bmi270.cpp")
        for operation in (
            "soft reset",
            "power control",
            "accelerometer configuration",
            "accelerometer range",
            "gyroscope configuration",
            "gyroscope range",
            "disable advanced power save",
            "halt config load",
            "complete config load",
            "power save mode",
        ):
            self.assertIn(f'"{operation}"', source)
        self.assertIn("bool BMI270Component::write_checked_", source)

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
        source = read(
            "components/jiecang_desk_controller/jiecang_desk_controller.cpp"
        )
        self.assertIn("RX_BYTES_PER_LOOP = 256", source)
        self.assertIn("while (budget-- > 0 && this->available())", source)
        self.assertIn(
            "height_cm = std::max(this->limit_min_, "
            "std::min(height_cm, this->limit_max_));",
            source,
        )
        self.assertIn(
            "pct = std::max(0.0f, std::min(pct, 100.0f));",
            source,
        )

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
        self.assertIn('CONF_VOLUME_MULTIPLIER = "volume_multiplier"', schema)


if __name__ == "__main__":
    unittest.main()

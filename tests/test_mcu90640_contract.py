import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / "components" / "mcu90640" / "mcu90640.cpp").read_text(
    encoding="utf-8"
)


class MCU90640RotationContractTest(unittest.TestCase):
    def test_quarter_turn_swaps_output_dimensions(self) -> None:
        self.assertRegex(
            CPP,
            re.compile(
                r"rendered_width_\(\) const \{.*?"
                r"rotation_ == 90 \|\| this->rotation_ == 270"
                r" \? this->output_height_ : this->output_width_;",
                re.DOTALL,
            ),
        )
        self.assertRegex(
            CPP,
            re.compile(
                r"rendered_height_\(\) const \{.*?"
                r"rotation_ == 90 \|\| this->rotation_ == 270"
                r" \? this->output_width_ : this->output_height_;",
                re.DOTALL,
            ),
        )

    def test_jpeg_uses_rotated_dimensions(self) -> None:
        self.assertIn(
            "this->rendered_width_(), this->rendered_height_(),",
            CPP,
        )

    def test_rotation_corner_mapping_stays_inside_source_frame(self) -> None:
        cols = 32
        rows = 24
        expected = {
            0: [(0, 0), (31, 0), (0, 23), (31, 23)],
            90: [(0, 23), (0, 0), (31, 23), (31, 0)],
            180: [(31, 23), (0, 23), (31, 0), (0, 0)],
            270: [(31, 0), (31, 23), (0, 0), (0, 23)],
        }

        for rotation, corners in expected.items():
            with self.subTest(rotation=rotation):
                for src_x, src_y in corners:
                    self.assertGreaterEqual(src_x, 0)
                    self.assertLess(src_x, cols)
                    self.assertGreaterEqual(src_y, 0)
                    self.assertLess(src_y, rows)

    def test_recovery_also_covers_missing_first_frame(self) -> None:
        self.assertIn("this->stream_started_", CPP)
        self.assertIn(
            "this->frame_count_ > 0 ? this->last_frame_time_ : this->last_stream_command_time_",
            CPP,
        )
        self.assertNotIn("if (this->frame_count_ > 0) {", CPP)

    def test_stream_and_image_warnings_are_independent(self) -> None:
        self.assertIn("this->stream_warning_ || this->image_warning_", CPP)
        self.assertIn("this->stream_warning_ = false;", CPP)
        self.assertIn("this->image_warning_ = true;", CPP)
        self.assertNotIn(
            "this->last_frame_time_ = millis();\n  this->status_clear_warning();",
            CPP,
        )

    def test_checksum_failure_rescans_for_sync(self) -> None:
        self.assertIn("void MCU90640Component::resync_after_invalid_frame_()", CPP)
        self.assertIn("memmove(this->rx_buf_", CPP)
        self.assertIn("this->resync_after_invalid_frame_();", CPP)


if __name__ == "__main__":
    unittest.main()

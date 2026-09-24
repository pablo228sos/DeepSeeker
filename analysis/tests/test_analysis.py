import csv
import sys
import tempfile
import unittest
from pathlib import Path


ANALYSIS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ANALYSIS_DIR))

from deepseeker_analysis import haversine_m, mission_summary, read_csv  # noqa: E402


class MissionAnalysisTests(unittest.TestCase):
    def test_haversine_zero(self):
        self.assertEqual(haversine_m(42.0, 77.0, 42.0, 77.0), 0.0)

    def test_summary(self):
        rows = [
            {
                "image": "IMG_000001.JPG",
                "utc": "2026-07-17T03:41:25Z",
                "latitude": "42.0000000",
                "longitude": "77.0000000",
                "satellites": "5",
                "hdop": "2.0",
                "speed_kmph": "0.0",
                "stable": "1",
                "capture_ms": "400",
                "jpeg_bytes": "100000",
            },
            {
                "image": "IMG_000002.JPG",
                "utc": "2026-07-17T03:41:27Z",
                "latitude": "42.0000100",
                "longitude": "77.0000000",
                "satellites": "7",
                "hdop": "1.0",
                "speed_kmph": "3.6",
                "stable": "0",
                "capture_ms": "500",
                "jpeg_bytes": "120000",
            },
        ]
        summary = mission_summary(rows)
        self.assertEqual(summary["frames"], 2)
        self.assertEqual(summary["duration_s"], 2.0)
        self.assertEqual(summary["median_speed_kmph"], 1.8)
        self.assertEqual(summary["median_capture_ms"], 450.0)
        self.assertEqual(summary["median_satellites"], 6.0)
        self.assertEqual(summary["stable_fraction"], 0.5)
        self.assertGreater(summary["north_span_m"], 1.0)

    def test_missing_columns_are_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.csv"
            with path.open("w", encoding="utf-8", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(["image", "utc"])
                writer.writerow(["IMG_1.JPG", "2026-07-17T00:00:00Z"])
            with self.assertRaisesRegex(ValueError, "missing required columns"):
                read_csv(path)


if __name__ == "__main__":
    unittest.main()

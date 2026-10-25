import csv
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import numpy as np
from scipy.io import loadmat
from csv_mat_core import analyze, active_channels, capture_payload, load_capture, save_unique_mat


class CaptureTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def write(self, header, rows, name="采集.csv"):
        path = self.root / name
        with path.open("w", encoding="utf-8-sig", newline="") as fp:
            writer = csv.writer(fp)
            writer.writerow(header)
            writer.writerows(rows)
        return path

    def emg(self, n=2000):
        t = np.arange(n) / 2000
        a = np.sin(2*np.pi*100*t) + .1*np.sin(2*np.pi*31*t)
        return load_capture(self.write(["timestamp_us", "sample_seq", "ch01", "ch02", "ch03", "ch04"],
                                      zip(np.arange(n)*500, np.arange(n), a, 2*a+7, -a, np.zeros(n))))

    def test_mat_roundtrip_exact_original_and_unique_names(self):
        path = self.write(["timestamp_us", "ch01", "ch02", "备注", "a b", "a-b"],
                          [[2**53+1, 4, 8, "左侧", 1, 2], [2**53+501, 5, 10, "右侧", 3, 4]])
        capture = load_capture(path)
        before = path.read_bytes()
        first = save_unique_mat(self.root, "测试", capture_payload(capture))
        second = save_unique_mat(self.root, "测试", capture_payload(capture))
        self.assertNotEqual(first, second)
        mat = loadmat(first, simplify_cells=True)
        np.testing.assert_array_equal(mat["columns"]["timestamp_us"], [2**53+1, 2**53+501])
        self.assertEqual(mat["columns"]["timestamp_us"].dtype, np.dtype("int64"))
        np.testing.assert_array_equal(mat["columns"]["col___"], ["左侧", "右侧"])
        self.assertEqual(len(set(mat["column_keys"])), 6)
        np.testing.assert_allclose(mat["time_s"], [0, .0005])
        self.assertEqual(path.read_bytes(), before)

    def test_multistream_imu_preserves_order(self):
        cap = load_capture(self.write(["source", "source_id", "timestamp_us", "x", "y", "z"],
            [["IMU", 1, 1000, 1, 2, 3], ["IMU", 2, 8000, 4, 5, 6],
             ["IMU", 1, 2000, 7, 8, 9], ["IMU", 2, 10000, 10, 11, 12]]))
        self.assertEqual(cap.kind, "IMU")
        np.testing.assert_array_equal(cap.streams[0]["row_indices"], [1, 3])
        self.assertAlmostEqual(cap.streams[0]["fs_hz"], 1000)
        self.assertAlmostEqual(cap.streams[1]["fs_hz"], 500)
        mat = loadmat(save_unique_mat(self.root, "imu", capture_payload(cap)), simplify_cells=True)
        np.testing.assert_array_equal(mat["imu_xyz"][:, 0], [1, 4, 7, 10])
        np.testing.assert_allclose(mat["time_s"], [0, 0, .001, .002])

    def test_wrap(self):
        cap = load_capture(self.write(["timestamp_us", "ch01", "ch02"],
                                     [[2**32-500, 1, 2], [0, 2, 4], [500, 3, 6]]))
        np.testing.assert_allclose(cap.time_s, [0, .0005, .001])
        self.assertAlmostEqual(cap.fs, 2000)

    def test_known_correlations_and_constant(self):
        cap = self.emg()
        self.assertEqual(active_channels(cap), ["ch01", "ch02", "ch03"])
        result = analyze(cap, cap.channels, start=.1, end=.3)
        np.testing.assert_allclose(result["correlation"], [[1,1,-1],[1,1,-1],[-1,-1,1]], atol=1e-12)
        self.assertEqual(result["sample_count"], 400)
        self.assertAlmostEqual(result["mean_r"], -1/3)
        self.assertAlmostEqual(result["mean_abs_r"], 1)
        self.assertEqual(list(result["excluded_constant_channels"]), ["ch04"])
        mat = loadmat(save_unique_mat(self.root, "analysis", result), simplify_cells=True)
        self.assertEqual(mat["processed_emg"].shape, (400, 3))

    def test_filters_and_invalid_nyquist(self):
        cap = self.emg()
        result = analyze(cap, cap.channels[:2], bandpass=True, notch="harmonics", start=.1, end=.8)
        self.assertAlmostEqual(result["mean_r"], 1., places=10)
        self.assertEqual(len(result["filters"]), 9)
        with self.assertRaisesRegex(ValueError, "带通范围"):
            analyze(cap, cap.channels[:2], bandpass=True, high=1000)

    def test_gap_raw_warns_filter_refuses(self):
        cap = self.emg()
        cap.time_s[1000:] += .001
        result = analyze(cap, cap.channels[:2])
        self.assertTrue(len(result["warnings"]))
        with self.assertRaisesRegex(ValueError, "间断"):
            analyze(cap, cap.channels[:2], bandpass=True)

    def test_timestamp_jitter_is_reported_but_not_a_gap(self):
        cap = self.emg()
        cap.time_s[1000] += .000035
        result = analyze(cap, cap.channels[:2], bandpass=True)
        self.assertAlmostEqual(result["mean_r"], 1., places=10)
        self.assertTrue(any("近似等间隔" in str(w) for w in result["warnings"]))

    def test_sequence_gap_filter_refuses(self):
        cap = self.emg()
        cap.columns["sample_seq"][500:] += 1
        with self.assertRaisesRegex(ValueError, "sample_seq 不连续"):
            analyze(cap, cap.channels[:2], bandpass=True)

    def test_nan_and_constant_rejected(self):
        cap = self.emg()
        cap.columns["ch01"][10] = np.nan
        with self.assertRaisesRegex(ValueError, "NaN"):
            analyze(cap, cap.channels[:2])
        with self.assertRaisesRegex(ValueError, "不足两个"):
            analyze(cap, ["ch03", "ch04"])

    def test_missing_time_requires_manual_rate(self):
        cap = load_capture(self.write(["ch01", "ch02"], [[1,2],[2,4],[3,6],[4,8]]))
        with self.assertRaisesRegex(ValueError, "采样率"):
            analyze(cap, cap.channels)
        result = analyze(cap, cap.channels, fs_override=1000)
        self.assertAlmostEqual(result["mean_r"], 1)
        self.assertEqual(result["fs_hz"], 1000)

    def test_reject_malformed_header_only_duplicates(self):
        for header, rows in ((["x","y"], [[1]]), (["x","x"], [[1,2]]), (["x"], [])):
            with self.assertRaises(ValueError):
                load_capture(self.write(header, rows))

    def test_invalid_time_range_and_nonmonotonic(self):
        cap = self.emg()
        with self.assertRaisesRegex(ValueError, "时间范围"):
            analyze(cap, cap.channels[:2], start=1, end=.5)
        cap.time_s[10] = cap.time_s[9]
        with self.assertRaisesRegex(ValueError, "重复"):
            analyze(cap, cap.channels[:2])


if __name__ == "__main__":
    unittest.main()

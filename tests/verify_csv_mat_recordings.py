"""Non-destructive validation with actual captures; writes only to --output."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import numpy as np
from scipy.io import loadmat
from csv_mat_core import active_channels, analyze, capture_payload, load_capture, save_unique_mat


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--emg", type=Path, required=True)
    parser.add_argument("--imu", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    records = []
    for path in (args.emg, args.imu):
        before = sha256(path)
        capture = load_capture(path)
        dest = save_unique_mat(args.output, path.stem, capture_payload(capture))
        loaded = loadmat(dest, simplify_cells=True)
        payload = capture_payload(capture)
        for name, key in zip(capture.columns, payload["column_keys"]):
            original = capture.columns[name]
            restored = np.asarray(loaded["columns"][key]).reshape(-1)
            if original.dtype.kind == "O":
                np.testing.assert_array_equal(original, restored)
            else:
                np.testing.assert_allclose(original, restored, rtol=0, atol=0, equal_nan=True)
                assert original.dtype == restored.dtype
        assert sha256(path) == before
        record = dict(source=str(path.resolve()), sha256=before, rows=capture.size,
                      kind=capture.kind, all_columns_exact=True, source_unchanged=True,
                      mat=str(dest.resolve()), warnings=capture.warnings)
        if capture.kind == "EMG":
            result = analyze(capture, active_channels(capture), start=2, end=8,
                             bandpass=True, notch="harmonics")
            analysis_path = save_unique_mat(args.output, "verified_analysis", result)
            matrix = loadmat(analysis_path)["correlation"]
            np.testing.assert_allclose(matrix, result["correlation"], rtol=0, atol=0)
            record.update(fs_hz=result["fs_hz"], analyzed_channels=list(result["channel_names"]),
                          start_s=2, end_s=8, sample_count=result["sample_count"],
                          mean_r=result["mean_r"], mean_abs_r=result["mean_abs_r"],
                          analysis_mat=str(analysis_path.resolve()))
        records.append(record)
    (args.output / "verification.json").write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(records, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

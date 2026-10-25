"""Exercise actual file drops, background loading, analysis, conversion and export."""
import csv
import os
from pathlib import Path
import sys
import tempfile
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets
from scipy.io import loadmat
from csv_mat_gui import MainWindow


def wait(app, window):
    deadline = time.monotonic() + 30
    while window.busy:
        app.processEvents()
        time.sleep(.01)
        assert time.monotonic() < deadline, "GUI worker timed out"
    app.processEvents()


def main():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        path = root / "拖放肌电.csv"
        n = 2000
        x = np.sin(np.arange(n)*.2)
        with path.open("w", encoding="utf-8-sig", newline="") as fp:
            writer = csv.writer(fp)
            writer.writerow(["timestamp_us", "ch01", "ch02", "ch03"])
            writer.writerows(zip(np.arange(n)*500, x, -x, np.zeros(n)))
        window = MainWindow()
        window.show()
        errors = []
        window.show_error = lambda message: (errors.append(message), window.finish_work())
        mime = QtCore.QMimeData()
        mime.setUrls([QtCore.QUrl.fromLocalFile(str(path))])
        enter = QtGui.QDragEnterEvent(QtCore.QPoint(10,10), QtCore.Qt.CopyAction, mime,
                                      QtCore.Qt.LeftButton, QtCore.Qt.NoModifier)
        window.files.dragEnterEvent(enter)
        assert enter.isAccepted()
        drop = QtGui.QDropEvent(QtCore.QPointF(10,10), QtCore.Qt.CopyAction, mime,
                                QtCore.Qt.LeftButton, QtCore.Qt.NoModifier)
        window.files.dropEvent(drop)
        wait(app, window)
        assert not errors, errors
        assert window.capture.size == n
        assert window.files.count() == 1
        window.add_files([str(path)])
        assert window.files.count() == 1, "duplicate file was added"
        window.output_directory = root / "导出"
        window.convert_current()
        wait(app, window)
        mat = loadmat(window.output_directory / (path.stem + ".mat"))
        assert mat["emg"].shape == (n, 3)
        window.run_analysis()
        wait(app, window)
        assert not errors, errors
        assert abs(window.result["mean_r"] + 1) < 1e-12
        assert window.export_button.isEnabled()
        window.export_analysis()
        wait(app, window)
        assert not errors, errors
        folder = window.output_directory / (path.stem + "_correlation")
        assert all((folder / name).exists() for name in ("analysis.mat", "correlation.csv", "correlation.png"))
        np.testing.assert_allclose(loadmat(folder / "analysis.mat")["correlation"], [[1,-1],[-1,1]])
        screenshot = os.environ.get("CSV_MAT_SCREENSHOT")
        if screenshot:
            window.grab().save(screenshot)
        window.start.setValue(.1)
        assert window.result is None and not window.export_button.isEnabled()
        window.convert_all()
        wait(app, window)
        assert (window.output_directory / (path.stem + "_1.mat")).exists()
        window.clear_files()
        assert window.capture is None and not window.convert_button.isEnabled()
        assert window.close()
    print("GUI PASS: CSV drop, duplicate handling, preview, background MAT conversion, correlation, export, invalidation, batch, clear")


if __name__ == "__main__":
    main()

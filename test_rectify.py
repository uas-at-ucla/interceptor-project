from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

import rectify
from rectify import CalibrationError, ResolutionMismatchError, StereoRectifier

W, H = 320, 240
FX = 300.0


def synthetic_calib(**overrides):
    calib = {
        "image_width": W,
        "image_height": H,
        "distortion_model": "plumb_bob",
        "left": {"K": [FX, 0, W / 2, 0, FX, H / 2, 0, 0, 1], "D": [0, 0, 0, 0, 0]},
        "right": {"K": [FX, 0, W / 2, 0, FX, H / 2, 0, 0, 1], "D": [0, 0, 0, 0, 0]},
        "R": [1, 0, 0, 0, 1, 0, 0, 0, 1],
        "T": [-0.06, 0, 0],
        "T_units": "m",
    }
    calib.update(overrides)
    return calib


def write(tmp_path, calib):
    path = tmp_path / "calib.yaml"
    path.write_text(yaml.safe_dump(calib))
    return path


def smooth_image(seed=0):
    rng = np.random.default_rng(seed)
    img = rng.integers(0, 256, (H, W), dtype=np.uint8)
    return cv2.GaussianBlur(img, (0, 0), 4)


def test_identity_calibration_leaves_image_unchanged(tmp_path):
    rect = StereoRectifier(write(tmp_path, synthetic_calib()))
    left, right = smooth_image(0), smooth_image(1)
    left_r, right_r = rect.rectify(left, right)
    # Ignore a thin border where remap samples outside the image.
    crop = np.s_[4:-4, 4:-4]
    for raw, out in ((left, left_r), (right, right_r)):
        assert out.shape == raw.shape and out.dtype == raw.dtype
        assert np.abs(out[crop].astype(int) - raw[crop].astype(int)).mean() < 1.0


def test_exposes_intrinsics_and_baseline(tmp_path):
    rect = StereoRectifier(write(tmp_path, synthetic_calib()))
    assert rect.fx == pytest.approx(FX, rel=1e-3)
    assert rect.cx == pytest.approx(W / 2, abs=0.5)
    assert rect.cy == pytest.approx(H / 2, abs=0.5)
    assert rect.baseline_m == pytest.approx(0.06, rel=1e-6)
    assert rect.P1.shape == (3, 4) and rect.P2.shape == (3, 4) and rect.Q.shape == (4, 4)


def test_t_in_mm_is_converted_to_m(tmp_path):
    rect = StereoRectifier(write(tmp_path, synthetic_calib(T=[-60.0, 0, 0], T_units="mm")))
    assert rect.baseline_m == pytest.approx(0.06, rel=1e-6)


def test_maps_built_once(tmp_path, monkeypatch):
    calls = []
    real = cv2.initUndistortRectifyMap
    monkeypatch.setattr(rectify.cv2, "initUndistortRectifyMap", lambda *a: calls.append(1) or real(*a))
    rect = StereoRectifier(write(tmp_path, synthetic_calib()))
    img = smooth_image()
    for _ in range(3):
        rect.rectify(img, img)
    assert len(calls) == 2
    assert rect.last_rectify_s is not None and rect.last_rectify_s >= 0


def test_resolution_mismatch_raises(tmp_path):
    rect = StereoRectifier(write(tmp_path, synthetic_calib()))
    small = np.zeros((H // 2, W // 2), np.uint8)
    with pytest.raises(ResolutionMismatchError, match=f"calibration expects {W}x{H}"):
        rect.rectify(small, small)
    with pytest.raises(ResolutionMismatchError, match="right image"):
        rect.rectify(smooth_image(), small)


def test_fisheye_model_builds_maps(tmp_path):
    calib = synthetic_calib(distortion_model="fisheye")
    calib["left"]["D"] = calib["right"]["D"] = [0.01, 0, 0, 0]
    rect = StereoRectifier(write(tmp_path, calib))
    out_l, out_r = rect.rectify(smooth_image(), smooth_image())
    assert out_l.shape == (H, W)
    assert rect.baseline_m == pytest.approx(0.06, rel=1e-3)


def test_precomputed_rectification_is_used(tmp_path):
    P1 = [250.0, 0, 150, 0, 0, 250, 110, 0, 0, 0, 1, 0]
    P2 = [250.0, 0, 150, -250 * 0.06, 0, 250, 110, 0, 0, 0, 1, 0]
    eye = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    Q = [1, 0, 0, -150, 0, 1, 0, -110, 0, 0, 0, 250, 0, 0, 1 / 0.06, 0]
    rect = StereoRectifier(write(tmp_path, synthetic_calib(R1=eye, R2=eye, P1=P1, P2=P2, Q=Q)))
    assert (rect.fx, rect.cx, rect.cy) == (250.0, 150.0, 110.0)
    assert rect.baseline_m == pytest.approx(0.06)


@pytest.mark.parametrize(
    "overrides, message",
    [
        ({"distortion_model": "polynomial9000"}, "unknown distortion_model"),
        ({"R": [1, 0, 0, 0, 1, 0]}, "'R' has 6 values, expected 9"),
        ({"T": [0.06, 0]}, "'T' has 2 values, expected 3"),
        ({"T_units": "inches"}, "T_units must be"),
        ({"T": [0, 0, 0]}, "baseline must be non-zero"),
        ({"image_width": 0}, "image size must be positive"),
        ({"R": [1, 0, 0, 0, 1, 0, 0, 0, 2]}, "not a rotation matrix"),
        ({"R1": [1, 0, 0, 0, 1, 0, 0, 0, 1]}, "partial rectification"),
        ({"baseline_m": 60.0}, "disagrees with"),
    ],
)
def test_bad_calibration_raises(tmp_path, overrides, message):
    with pytest.raises(CalibrationError, match=message):
        StereoRectifier(write(tmp_path, synthetic_calib(**overrides)))


def test_wrong_d_length_raises(tmp_path):
    calib = synthetic_calib()
    calib["left"]["D"] = [0, 0, 0]
    with pytest.raises(CalibrationError, match=r"'left.D' has 3 values, expected 4 or 5"):
        StereoRectifier(write(tmp_path, calib))


def test_fisheye_requires_four_coefficients(tmp_path):
    with pytest.raises(CalibrationError, match="expected 4$"):
        StereoRectifier(write(tmp_path, synthetic_calib(distortion_model="fisheye")))


@pytest.mark.parametrize("side", ["left", "right"])
def test_zero_focal_length_raises(tmp_path, side):
    calib = synthetic_calib()
    calib[side]["K"] = [0, 0, W / 2, 0, FX, H / 2, 0, 0, 1]
    with pytest.raises(CalibrationError, match=f"{side}.K: focal lengths must be > 0"):
        StereoRectifier(write(tmp_path, calib))


def test_missing_file_raises(tmp_path):
    with pytest.raises(CalibrationError, match="not found"):
        StereoRectifier(tmp_path / "nope.yaml")


def test_opencv_filestorage_format(tmp_path):
    """Files written by cv2.FileStorage (K1/D1/... as !!opencv-matrix) load too."""
    path = str(tmp_path / "fs.yaml")
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    fs.write("model", "rational")
    fs.write("image_width", W)
    fs.write("image_height", H)
    K = np.array([[FX, 0, W / 2], [0, FX, H / 2], [0, 0, 1]])
    for name, val in (("K1", K), ("D1", np.zeros((1, 8))), ("K2", K), ("D2", np.zeros((1, 8))),
                      ("R", np.eye(3)), ("T", np.array([[-0.06], [0], [0]]))):
        fs.write(name, val)
    fs.release()
    rect = StereoRectifier(path)
    assert rect.model == "rational"
    assert rect.baseline_m == pytest.approx(0.06, rel=1e-6)


REPO_CALIB = Path(__file__).with_name("stereo_calibration.yaml")


@pytest.mark.skipif(not REPO_CALIB.exists(), reason="no repo calibration file")
def test_repo_calibration_loads():
    rect = StereoRectifier(REPO_CALIB)
    w, h = rect.image_size
    img = np.zeros((h, w, 3), np.uint8)
    left_r, right_r = rect.rectify(img, img)
    assert left_r.shape == img.shape
    assert 0.01 < rect.baseline_m < 1.0

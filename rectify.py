"""Stereo undistortion + rectification.

Remap tables are built once in ``StereoRectifier.__init__``; each frame then
costs two ``cv2.remap`` calls.

Two calibration file layouts are accepted:

* The plain layout (``left: {K, D}``, ``right: {K, D}``, ``R``, ``T``,
  ``T_units``, ``distortion_model``) with flat row-major lists.
* OpenCV ``FileStorage`` output (``K1``, ``D1``, ``K2``, ``D2``, ``R``, ``T``
  as ``!!opencv-matrix`` nodes, ``model`` instead of ``distortion_model``).

Either may carry precomputed ``R1``, ``R2``, ``P1``, ``P2``, ``Q``; if all five
are present they are used as-is, otherwise they are computed here.
"""

import time
from pathlib import Path

import cv2
import numpy as np
import yaml

# Accepted distortion coefficient counts per model.
_D_LENGTHS = {
    "plumb_bob": (4, 5),
    "rational": (8, 12, 14),
    "fisheye": (4,),
}
_MODEL_ALIASES = {"rational_polynomial": "rational"}
_RECT_SHAPES = {"R1": (3, 3), "R2": (3, 3), "P1": (3, 4), "P2": (3, 4), "Q": (4, 4)}
_T_SCALE = {"m": 1.0, "mm": 1e-3}


class CalibrationError(ValueError):
    """The calibration file is missing, malformed, or inconsistent."""


class ResolutionMismatchError(ValueError):
    """An input frame does not match the calibration resolution."""


class _CalibLoader(yaml.SafeLoader):
    pass


def _opencv_matrix(loader, node):
    m = loader.construct_mapping(node, deep=True)
    return np.asarray(m["data"], dtype=np.float64).reshape(int(m["rows"]), int(m["cols"]))


_CalibLoader.add_constructor("tag:yaml.org,2002:opencv-matrix", _opencv_matrix)


def _read_yaml(path):
    text = Path(path).read_text()
    # OpenCV writes "%YAML:1.0" (invalid for PyYAML) or "%YAML 1.2"; drop the directive.
    if text.startswith("%YAML"):
        text = text.split("\n", 1)[1] if "\n" in text else ""
    try:
        data = yaml.load(text, Loader=_CalibLoader)
    except yaml.YAMLError as e:
        raise CalibrationError(f"{path}: not valid YAML: {e}") from e
    if not isinstance(data, dict):
        raise CalibrationError(f"{path}: expected a mapping at top level")
    return data


def _array(value, name, sizes, shape=None):
    """Convert a list / matrix to float64, checking its element count."""
    if value is None:
        raise CalibrationError(f"missing required field '{name}'")
    try:
        a = np.asarray(value, dtype=np.float64).ravel()
    except (TypeError, ValueError) as e:
        raise CalibrationError(f"'{name}' must contain only numbers: {e}") from e
    if a.size not in sizes:
        want = " or ".join(str(s) for s in sizes)
        raise CalibrationError(f"'{name}' has {a.size} values, expected {want}")
    if not np.all(np.isfinite(a)):
        raise CalibrationError(f"'{name}' contains non-finite values")
    return a.reshape(shape) if shape else a


def load_calibration(path):
    """Read and validate a calibration file. Returns a dict of numpy arrays.

    T is always returned in meters.
    """
    path = Path(path)
    if not path.is_file():
        raise CalibrationError(f"calibration file not found: {path}")
    raw = _read_yaml(path)

    try:
        width = int(raw.get("image_width", 0))
        height = int(raw.get("image_height", 0))
    except (TypeError, ValueError):
        raise CalibrationError("image_width / image_height must be integers")
    if width <= 0 or height <= 0:
        raise CalibrationError(f"image size must be positive, got {width}x{height}")

    model = raw.get("distortion_model", raw.get("model"))
    if model is None:
        raise CalibrationError("missing 'distortion_model'")
    model = _MODEL_ALIASES.get(str(model), str(model))
    if model not in _D_LENGTHS:
        raise CalibrationError(
            f"unknown distortion_model '{model}' (expected one of {sorted(_D_LENGTHS)})"
        )

    if "left" in raw or "right" in raw:
        left, right = raw.get("left") or {}, raw.get("right") or {}
        if not isinstance(left, dict) or not isinstance(right, dict):
            raise CalibrationError("'left' and 'right' must be mappings with K and D")
        k1, d1, k2, d2 = left.get("K"), left.get("D"), right.get("K"), right.get("D")
    else:
        k1, d1, k2, d2 = raw.get("K1"), raw.get("D1"), raw.get("K2"), raw.get("D2")

    calib = {
        "width": width,
        "height": height,
        "model": model,
        "K1": _array(k1, "left.K", (9,), (3, 3)),
        "D1": _array(d1, "left.D", _D_LENGTHS[model]),
        "K2": _array(k2, "right.K", (9,), (3, 3)),
        "D2": _array(d2, "right.D", _D_LENGTHS[model]),
        "R": _array(raw.get("R"), "R", (9,), (3, 3)),
    }
    for side in ("K1", "K2"):
        K = calib[side]
        if K[0, 0] <= 0 or K[1, 1] <= 0:
            name = "left.K" if side == "K1" else "right.K"
            raise CalibrationError(f"{name}: focal lengths must be > 0, got fx={K[0, 0]}, fy={K[1, 1]}")

    units = str(raw.get("T_units", "m"))
    if units not in _T_SCALE:
        raise CalibrationError(f"T_units must be 'm' or 'mm', got '{units}'")
    calib["T"] = _array(raw.get("T"), "T", (3,), (3, 1)) * _T_SCALE[units]
    if np.linalg.norm(calib["T"]) == 0:
        raise CalibrationError("T is zero: baseline must be non-zero")

    R = calib["R"]
    if not np.allclose(R @ R.T, np.eye(3), atol=1e-3) or np.linalg.det(R) < 0:
        raise CalibrationError("R is not a rotation matrix")

    present = [k for k in _RECT_SHAPES if raw.get(k) is not None]
    if present and len(present) != len(_RECT_SHAPES):
        missing = sorted(set(_RECT_SHAPES) - set(present))
        raise CalibrationError(f"partial rectification given: missing {missing}; provide all of R1/R2/P1/P2/Q or none")
    for k in present:
        shape = _RECT_SHAPES[k]
        calib[k] = _array(raw[k], k, (shape[0] * shape[1],), shape)

    # Cross-check a stored baseline against T, which catches mm values labelled as m.
    if raw.get("baseline_m") is not None:
        stored, from_t = float(raw["baseline_m"]), float(np.linalg.norm(calib["T"]))
        if abs(stored - from_t) > 0.05 * from_t:
            raise CalibrationError(
                f"baseline_m={stored} disagrees with |T|={from_t:.6f} m (check T_units)"
            )
    return calib


class StereoRectifier:
    """Undistorts and rectifies stereo pairs with precomputed remap tables.

    Attributes for downstream vision (all in the rectified image frame):
        P1, P2, Q   rectified projection / reprojection matrices
        fx, cx, cy  rectified intrinsics (pixels)
        baseline_m  stereo baseline (meters); depth Z = fx * baseline_m / disparity
        image_size  (width, height)
        last_rectify_s  wall time of the most recent rectify() call (seconds)
    """

    def __init__(self, calib_path):
        c = load_calibration(calib_path)
        self.model = c["model"]
        self.image_size = (c["width"], c["height"])
        fisheye = self.model == "fisheye"

        if "P1" in c:
            R1, R2, P1, P2, Q = c["R1"], c["R2"], c["P1"], c["P2"], c["Q"]
        elif fisheye:
            R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
                c["K1"], c["D1"].reshape(4, 1), c["K2"], c["D2"].reshape(4, 1),
                self.image_size, c["R"], c["T"], cv2.CALIB_ZERO_DISPARITY,
                balance=0.0,
            )
        else:
            R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
                c["K1"], c["D1"], c["K2"], c["D2"], self.image_size, c["R"], c["T"],
                flags=cv2.CALIB_ZERO_DISPARITY, alpha=0,
            )
        self.R1, self.R2, self.P1, self.P2, self.Q = R1, R2, P1, P2, Q

        # Horizontal rectification puts the baseline in P2[0, 3] = -fx * B.
        if abs(P2[1, 3]) > abs(P2[0, 3]):
            raise CalibrationError("calibration rectifies vertically; a horizontal stereo pair is required")
        self.fx = float(P1[0, 0])
        self.cx = float(P1[0, 2])
        self.cy = float(P1[1, 2])
        self.baseline_m = abs(float(P2[0, 3])) / float(P2[0, 0])

        init_map = cv2.fisheye.initUndistortRectifyMap if fisheye else cv2.initUndistortRectifyMap
        self._maps = [
            init_map(K, D, Rr, P, self.image_size, cv2.CV_16SC2)
            for K, D, Rr, P in ((c["K1"], c["D1"], R1, P1), (c["K2"], c["D2"], R2, P2))
        ]
        self.last_rectify_s = None

    def rectify(self, left_raw, right_raw):
        """Return (left_rect, right_rect). Inputs must match the calibration size."""
        w, h = self.image_size
        for name, img in (("left", left_raw), ("right", right_raw)):
            if img is None or img.ndim < 2 or img.shape[:2] != (h, w):
                got = "None" if img is None else f"{img.shape[1]}x{img.shape[0]}"
                raise ResolutionMismatchError(f"{name} image is {got}, calibration expects {w}x{h}")
        t0 = time.perf_counter()
        out = tuple(
            cv2.remap(img, m1, m2, cv2.INTER_LINEAR)
            for img, (m1, m2) in zip((left_raw, right_raw), self._maps)
        )
        self.last_rectify_s = time.perf_counter() - t0
        return out

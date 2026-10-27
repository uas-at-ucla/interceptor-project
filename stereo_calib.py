#!/usr/bin/env python3
"""
stereo_calib.py - per-lens intrinsic calibration for the ELP binocular USB camera.

The camera delivers ONE 3200x1200 side-by-side MJPEG frame: the left lens is the
left half and the right lens is the right half (1600x1200 each). This script
calibrates each lens on its own with a ChArUco board (intrinsics only, no
stereo yet) and then demos a live undistorted stream.

Subcommands (run `uv run stereo_calib.py <command> -h` for all options):
  detect-dict  find which ArUco dictionary the printed board uses
  capture      live preview with detections; SPACE or left click saves a left/right image pair
  calibrate    calibrate LEFT and RIGHT independently -> calib.yaml
  live         raw vs undistorted live view using calib.yaml

All lengths are in millimetres.

Adding stereo later: write cmd_stereo(args), register it in build_parser(), and
reuse CharucoFinder, calibrate_lens/LensCalib, load_calib and the saved
calib_images/left_XXX.png + right_XXX.png pairs (same index = same instant).
calib.yaml keeps each lens in its own section, so a `stereo` section can be
added without touching the existing ones.
"""

import argparse
import collections
import datetime
import glob
import os
import re
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

FRAME_W, FRAME_H = 3200, 1200   # full side-by-side frame from the camera
HALF_W = FRAME_W // 2           # width of one lens image
LENSES = ("left", "right")      # order matches the frame layout

# Dictionaries tried by detect-dict, smallest first within each family so the
# smallest one wins a tie.
DICTS = [
    "DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250",
    "DICT_5X5_50", "DICT_5X5_100", "DICT_5X5_250",
    "DICT_6X6_50", "DICT_6X6_100", "DICT_6X6_250",
    "DICT_7X7_250",
    "DICT_ARUCO_ORIGINAL",
]

# Our printed board, confirmed with detect-dict: generated as 5 across x 7 down
# (it looks 7x5 when held landscape), with 6x6 markers.
DEFAULT_SQUARES = (5, 7)
DEFAULT_DICT = "DICT_6X6_50"

# OpenCV 4.7 reworked the aruco API (CharucoDetector, board.matchImagePoints,
# CharucoBoard constructor) and removed the old free functions in 5.x.
HAS_ARUCO = hasattr(cv2, "aruco")
NEW_API = HAS_ARUCO and hasattr(cv2.aruco, "CharucoDetector")

BACKENDS = {"v4l2": cv2.CAP_V4L2, "dshow": cv2.CAP_DSHOW, "msmf": cv2.CAP_MSMF, "any": cv2.CAP_ANY}

MIN_VIEWS = 5            # hard minimum of usable images per lens
RECOMMENDED_VIEWS = 15   # the rational model has 12 intrinsics; fewer views is shaky
COVER_GRID = (8, 6)      # cells across/down used for the coverage score in capture
COVER_RADIUS = 18        # px painted around each saved corner in the coverage mask
STABILITY_FRAMES = 30    # frames of marker-count history shown in capture

GREEN, RED, YELLOW, WHITE = (0, 255, 0), (0, 0, 255), (0, 255, 255), (255, 255, 255)


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------

MAX_PROBE = 8   # camera indices tried by --device auto


def try_open(index: int, backend: str):
    """Open one camera index in 3200x1200 MJPEG mode. Returns (cap, (w, h)); cap is None if unusable."""
    cap = cv2.VideoCapture(index, BACKENDS[backend])
    if not cap.isOpened():
        return None, None

    # FOURCC is set both first and last. V4L2 needs it before the size (the
    # high-res modes are only offered as MJPG). DirectShow resets the format
    # whenever the size or FPS changes, falling back to raw YUY2 at ~2 fps,
    # so it has to be set again after them.
    mjpg = cv2.VideoWriter_fourcc(*"MJPG")
    cap.set(cv2.CAP_PROP_FOURCC, mjpg)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS, 60)
    cap.set(cv2.CAP_PROP_FOURCC, mjpg)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # prefer the newest frame (not every backend honours this)

    # The first reads after a mode switch can come back empty.
    for _ in range(30):
        ok, frame = cap.read()
        if ok and frame is not None:
            h, w = frame.shape[:2]
            return cap, (w, h)
    cap.release()
    return None, None


def open_camera(device: str, backend: str) -> cv2.VideoCapture:
    """Open the ELP camera in 3200x1200 MJPEG mode, or exit with an explanation.

    device is an index, or "auto" to use the first index that delivers 3200x1200
    (the ELP's side-by-side mode, which no ordinary webcam has).
    """
    if backend == "auto":
        backend = {"linux": "v4l2", "win32": "dshow"}.get(sys.platform, "any")

    if device == "auto":
        print(f"Looking for a {FRAME_W}x{FRAME_H} camera on indices 0-{MAX_PROBE - 1} ({backend})...")
        for index in range(MAX_PROBE):
            cap, size = try_open(index, backend)
            if cap is None:
                continue
            if size == (FRAME_W, FRAME_H):
                print(f"  camera {index}: {size[0]}x{size[1]}  <- using this one")
                return describe(cap, index, backend)
            print(f"  camera {index}: {size[0]}x{size[1]} (not the ELP)")
            cap.release()
        sys.exit(f"ERROR: no camera delivered {FRAME_W}x{FRAME_H}. Check the ELP is plugged in and not "
                 f"open in another app, or try another --backend.")

    index = int(device)
    cap, size = try_open(index, backend)
    if cap is None:
        sys.exit(f"ERROR: could not get frames from camera {index} with backend '{backend}'. "
                 f"Try --device auto or another --backend.")
    if size != (FRAME_W, FRAME_H):
        cap.release()
        sys.exit(f"ERROR: camera {index} gave {size[0]}x{size[1]}, expected {FRAME_W}x{FRAME_H}. "
                 f"It is probably not the ELP; try --device auto.")
    return describe(cap, index, backend)


def describe(cap: cv2.VideoCapture, index: int, backend: str) -> cv2.VideoCapture:
    code = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc = "".join(chr((code >> 8 * i) & 0xFF) for i in range(4))
    print(f"Camera {index} ({backend}): {FRAME_W}x{FRAME_H}, FOURCC '{fourcc}', "
          f"driver reports {cap.get(cv2.CAP_PROP_FPS):.0f} fps")
    if fourcc != "MJPG":
        print(f"WARNING: camera is sending '{fourcc}' instead of MJPG; expect only a few fps at "
              f"this resolution. Try another --backend.")
    return cap


def read_frame(cap: cv2.VideoCapture) -> np.ndarray:
    ok, frame = cap.read()
    if not ok or frame is None:
        sys.exit("ERROR: camera stopped delivering frames.")
    return frame


def split_frame(frame: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Side-by-side frame -> (left lens image, right lens image). These are views, not copies."""
    return frame[:, :HALF_W], frame[:, HALF_W:]


# ---------------------------------------------------------------------------
# Display helpers (display only; processing always runs at full resolution)
# ---------------------------------------------------------------------------

def fit_width(img: np.ndarray, width: int) -> np.ndarray:
    if width <= 0 or img.shape[1] <= width:
        return img
    scale = width / img.shape[1]
    return cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)


def put_lines(img, lines, org=(10, 30), scale=0.7):
    """Draw text lines (str or (str, colour)) with a dark outline so they read on any background."""
    x, y = org
    for line in lines:
        text, colour = (line, WHITE) if isinstance(line, str) else line
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, colour, 2, cv2.LINE_AA)
        y += int(34 * scale)


def wait_key() -> int:
    return cv2.waitKey(1) & 0xFF


def is_quit(key: int) -> bool:
    return key in (ord("q"), 27)  # q or ESC


# ---------------------------------------------------------------------------
# ChArUco board and detection (hides the old/new OpenCV API difference)
# ---------------------------------------------------------------------------

@dataclass
class BoardConfig:
    squares_x: int
    squares_y: int
    square_mm: float
    marker_mm: float
    dictionary: str
    legacy: bool = False

    @classmethod
    def from_args(cls, args, dictionary=None, legacy=None):
        return cls(args.squares_x, args.squares_y, args.square_mm, args.marker_mm,
                   dictionary if dictionary is not None else args.dict,
                   legacy if legacy is not None else args.legacy)


_legacy_warned = False


def make_board(cfg: BoardConfig):
    """Build the ChArUco board (and its dictionary) for either aruco API."""
    global _legacy_warned
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cfg.dictionary))
    if NEW_API:
        board = cv2.aruco.CharucoBoard((cfg.squares_x, cfg.squares_y),
                                       cfg.square_mm, cfg.marker_mm, dictionary)
    else:
        board = cv2.aruco.CharucoBoard_create(cfg.squares_x, cfg.squares_y,
                                              cfg.square_mm, cfg.marker_mm, dictionary)
    if cfg.legacy:
        if hasattr(board, "setLegacyPattern"):
            board.setLegacyPattern(True)
        elif not _legacy_warned:
            # OpenCV before the pattern change only knows the legacy layout anyway.
            print("note: this OpenCV has no setLegacyPattern; boards already use the legacy layout")
            _legacy_warned = True
    return board, dictionary


def legacy_supported() -> bool:
    board, _ = make_board(BoardConfig(5, 5, 1.0, 0.5, "DICT_4X4_50"))
    return hasattr(board, "setLegacyPattern")


@dataclass
class Detection:
    marker_corners: tuple
    marker_ids: Optional[np.ndarray]
    corners: Optional[np.ndarray]   # ChArUco (chessboard) corners, Nx1x2, image px
    ids: Optional[np.ndarray]       # ids of those corners, Nx1

    @property
    def n_markers(self) -> int:
        return 0 if self.marker_ids is None else len(self.marker_ids)

    @property
    def n_corners(self) -> int:
        return 0 if self.ids is None else len(self.ids)


class CharucoFinder:
    """Detects ChArUco corners and turns them into calibration points, on any OpenCV 4.x/5.x."""

    def __init__(self, cfg: BoardConfig):
        self.cfg = cfg
        self.board, self.dictionary = make_board(cfg)
        if NEW_API:
            self.detector = cv2.aruco.CharucoDetector(self.board)
        else:
            self.params = cv2.aruco.DetectorParameters_create()

    def detect(self, image: np.ndarray) -> Detection:
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if NEW_API:
            corners, ids, m_corners, m_ids = self.detector.detectBoard(gray)
        else:
            m_corners, m_ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
            corners = ids = None
            if m_ids is not None and len(m_ids) > 0:
                n, corners, ids = cv2.aruco.interpolateCornersCharuco(m_corners, m_ids, gray, self.board)
                if not n:
                    corners = ids = None
        # OpenCV 5 returns corners as (N, 2) and ids as (N,); 4.x uses (N, 1, 2) and (N, 1).
        # Normalise to the 4.x shapes, which every drawing/calibration function accepts.
        if ids is not None and len(ids) > 0:
            corners = np.asarray(corners, np.float32).reshape(-1, 1, 2)
            ids = np.asarray(ids, np.int32).reshape(-1, 1)
        else:
            corners = ids = None
        return Detection(m_corners, m_ids, corners, ids)

    def match_points(self, det: Detection) -> Tuple[np.ndarray, np.ndarray]:
        """Board points (mm, z=0) and the matching image points, as calibrateCamera wants them."""
        if NEW_API:
            obj, img = self.board.matchImagePoints(det.corners, det.ids)
        else:
            pts = (self.board.getChessboardCorners() if hasattr(self.board, "getChessboardCorners")
                   else self.board.chessboardCorners)
            obj = np.asarray(pts, np.float32)[det.ids.flatten()]
            img = det.corners
        return (np.asarray(obj, np.float32).reshape(-1, 1, 3),
                np.asarray(img, np.float32).reshape(-1, 1, 2))

    def draw(self, image: np.ndarray, det: Detection) -> None:
        if det.n_markers:
            cv2.aruco.drawDetectedMarkers(image, det.marker_corners, det.marker_ids)
        if det.n_corners:
            cv2.aruco.drawDetectedCornersCharuco(image, det.corners, det.ids, GREEN)


def spans_plane(obj: np.ndarray) -> bool:
    """calibrateCamera fails on corners that all lie on one board row or column."""
    xy = obj.reshape(-1, 3)[:, :2]
    return len(np.unique(xy[:, 0])) > 1 and len(np.unique(xy[:, 1])) > 1


# ---------------------------------------------------------------------------
# detect-dict
# ---------------------------------------------------------------------------

def cmd_detect_dict(args) -> None:
    cap = open_camera(args.device, args.backend)
    print("Hold the board where both lenses see it, then press SPACE to test every dictionary. q quits.")
    try:
        while True:
            frame = read_frame(cap)
            view = fit_width(frame, args.display_width)
            put_lines(view, ["SPACE: test dictionaries   q: quit"])
            cv2.imshow("detect-dict", view)
            key = wait_key()
            if is_quit(key):
                break
            if key == ord(" "):
                test_dictionaries(frame, args)
    finally:
        cap.release()
        cv2.destroyAllWindows()


def test_dictionaries(frame: np.ndarray, args) -> None:
    grays = [cv2.cvtColor(h, cv2.COLOR_BGR2GRAY) for h in split_frame(frame)]
    legacy_options = (False, True) if legacy_supported() else (False,)
    # Also try the board transposed. Marker IDs are numbered row by row, so a board
    # generated as 5x7 and one generated as 7x5 put different IDs in different places.
    # With the wrong one, every marker is found but no corner is.
    layouts = [(args.squares_x, args.squares_y)]
    if args.squares_x != args.squares_y:
        layouts.append((args.squares_y, args.squares_x))
    results = []
    for name in DICTS:
        for sx, sy in layouts:
            for legacy in legacy_options:
                cfg = BoardConfig(sx, sy, args.square_mm, args.marker_mm, name, legacy)
                dets = [CharucoFinder(cfg).detect(g) for g in grays]
                results.append({
                    "name": name, "legacy": legacy, "layout": (sx, sy),
                    "markers": [d.n_markers for d in dets], "corners": [d.n_corners for d in dets],
                    "ids": sorted({int(i) for d in dets if d.n_markers for i in d.marker_ids.flatten()}),
                })
    # Corners decide it: the legacy flag doesn't change how many markers are found, only
    # whether their IDs map onto the right squares, which shows up as missing corners.
    # The sort is stable, so ties keep DICTS order (smallest dictionary first).
    results.sort(key=lambda r: (sum(r["corners"]), sum(r["markers"])), reverse=True)

    max_markers = args.squares_x * args.squares_y // 2
    max_corners = (args.squares_x - 1) * (args.squares_y - 1)
    print(f"\nPer lens a full view is {max_markers} markers and {max_corners} corners.")
    print(f"{'dictionary':<22}{'board':<8}{'legacy':<8}{'markers L/R':<14}{'corners L/R':<14}")
    for r in results:
        if sum(r["markers"]) == 0:
            continue
        print(f"{r['name']:<22}{'%dx%d' % r['layout']:<8}{('yes' if r['legacy'] else 'no'):<8}"
              f"{'%d/%d' % tuple(r['markers']):<14}{'%d/%d' % tuple(r['corners']):<14}")

    best = results[0]
    if sum(best["markers"]) == 0:
        print("No markers found with any dictionary. Check lighting, focus and that the board fills more of the view.")
        return
    if sum(best["corners"]) == 0:
        print(f"\nMarkers found with {best['name']} but no ChArUco corners in any layout, so the board "
              f"geometry doesn't match. Marker IDs seen: {best['ids']}. A {args.squares_x}x{args.squares_y} "
              f"board uses IDs 0-{max_markers - 1}; other IDs mean a different square count, or a board "
              f"generated with a custom ID offset. Show the whole board to both lenses and try again.")
        return
    sx, sy = best["layout"]
    # Only list the flags that differ from the defaults.
    flags = []
    if best["name"] != DEFAULT_DICT:
        flags.append(f"--dict {best['name']}")
    if (sx, sy) != DEFAULT_SQUARES:
        flags.append(f"--squares-x {sx} --squares-y {sy}")
    if best["legacy"]:
        flags.append("--legacy")
    print(f"\nBest: {best['name']}, {sx}x{sy} board. "
          + (f"Use: {' '.join(flags)}" if flags else "That matches the defaults, so no flags are needed."))
    if (sx, sy) != (args.squares_x, args.squares_y):
        print(f"The board was generated as {sx} squares across x {sy} down (it only looks "
              f"{args.squares_x}x{args.squares_y} because it is rotated). Pass "
              f"--squares-x {sx} --squares-y {sy} to capture and calibrate; holding it either way round is fine.")
    twin = next((r for r in results if r["name"] == best["name"] and r["layout"] == best["layout"]
                 and r["legacy"] != best["legacy"]), None)
    if twin and twin["corners"] == best["corners"]:
        print("Legacy and current layouts gave the same corners. That is expected when the board has an "
              "odd number of rows, where the two layouts are identical, so --legacy doesn't matter.")
    print("Dictionaries of the same family (e.g. 5X5_50/100/250) share their low IDs and tie; "
          "any of the tied ones works.")


# ---------------------------------------------------------------------------
# capture
# ---------------------------------------------------------------------------

def pair_path(folder: str, lens: str, idx: int) -> str:
    return os.path.join(folder, f"{lens}_{idx:03d}.png")


def existing_pairs(folder: str) -> List[int]:
    """Indices of left_XXX.png files already in the folder, so numbering can continue."""
    found = []
    for path in glob.glob(os.path.join(folder, "left_*.png")):
        m = re.fullmatch(r"left_(\d+)\.png", os.path.basename(path))
        if m:
            found.append(int(m.group(1)))
    return sorted(found)


def add_coverage(mask: np.ndarray, det: Detection) -> None:
    for x, y in det.corners.reshape(-1, 2):
        cv2.circle(mask, (int(round(x)), int(round(y))), COVER_RADIUS, 255, -1)


def tint_coverage(panel: np.ndarray, mask: np.ndarray) -> None:
    """Shade areas already covered by saved corners green so the gaps (usually the corners) stand out."""
    # Blend the whole panel with OpenCV's uint8 ops and copy it back through the mask. Fancy-indexing
    # only the covered pixels in float slowed every frame down as coverage grew.
    tinted = cv2.add(cv2.convertScaleAbs(panel, alpha=0.6), tuple(0.4 * c for c in GREEN) + (0,))
    np.copyto(panel, tinted, where=mask[..., None] > 0)


def coverage_score(mask: np.ndarray) -> str:
    cx, cy = COVER_GRID
    h, w = mask.shape
    cells = mask[: h - h % cy, : w - w % cx].reshape(cy, h // cy, cx, w // cx).max(axis=(1, 3))
    return f"{int((cells > 0).sum())}/{cx * cy} cells"


def cmd_capture(args) -> None:
    finder = CharucoFinder(BoardConfig.from_args(args))
    os.makedirs(args.out_dir, exist_ok=True)

    # Accumulated corner positions per lens, painted as discs.
    coverage = {lens: np.zeros((FRAME_H, HALF_W), np.uint8) for lens in LENSES}
    pairs = existing_pairs(args.out_dir)
    next_idx = pairs[-1] + 1 if pairs else 0
    if pairs:
        print(f"{len(pairs)} pairs already in {args.out_dir}; numbering continues at {next_idx:03d}. "
              f"Re-detecting them to seed the coverage overlay...")
        for idx in pairs:
            for lens in LENSES:
                img = cv2.imread(pair_path(args.out_dir, lens, idx), cv2.IMREAD_GRAYSCALE)
                if img is not None:
                    det = finder.detect(img)
                    if det.n_corners >= args.min_corners:
                        add_coverage(coverage[lens], det)
    saved = len(pairs)

    # Recent marker counts per lens: a wide min..max spread on a still board means flaky detection.
    history = {lens: collections.deque(maxlen=STABILITY_FRAMES) for lens in LENSES}

    # A left click saves too, so the board can be held with one hand while clicking with the other.
    clicked = [False]

    def on_mouse(event, _x, _y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked[0] = True

    cap = open_camera(args.device, args.backend)
    cv2.namedWindow("capture")   # the callback needs the window to exist before the first imshow
    cv2.setMouseCallback("capture", on_mouse)
    print("SPACE or left click saves a pair, q quits.")
    try:
        while True:
            frame = read_frame(cap)
            halves = split_frame(frame)
            dets = [finder.detect(h) for h in halves]
            for lens, det in zip(LENSES, dets):
                history[lens].append(det.n_markers)
            if args.log_counts:
                print("markers L %2d R %2d   corners L %2d R %2d"
                      % (dets[0].n_markers, dets[1].n_markers, dets[0].n_corners, dets[1].n_corners))

            panels = []
            for lens, half, det in zip(LENSES, halves, dets):
                panel = half.copy()
                tint_coverage(panel, coverage[lens])
                finder.draw(panel, det)
                panels.append(panel)
            view = fit_width(cv2.hconcat(panels), args.display_width)

            panel_w = view.shape[1] // 2
            for i, (lens, det) in enumerate(zip(LENSES, dets)):
                good = det.n_corners >= args.min_corners
                hist = history[lens]
                put_lines(view, [
                    (f"{lens.upper()}  markers {det.n_markers}  corners {det.n_corners}", GREEN if good else RED),
                    f"markers last {len(hist)} frames: {min(hist)}..{max(hist)}",
                    f"coverage {coverage_score(coverage[lens])}",
                ], org=(10 + i * panel_w, 30))
            put_lines(view, [f"saved pairs: {saved}    SPACE/click save    q quit"], org=(10, view.shape[0] - 15))
            cv2.imshow("capture", view)

            key = wait_key()
            if is_quit(key):
                break
            save = key == ord(" ") or clicked[0]
            clicked[0] = False
            if not save:
                continue

            counts = [d.n_corners for d in dets]
            if max(counts) < args.min_corners:
                print(f"Not saved: neither lens sees {args.min_corners}+ corners (left {counts[0]}, right {counts[1]}).")
                continue
            # Save both halves even if only one detects, so the pair can be reused for stereo later.
            for lens, half in zip(LENSES, halves):
                cv2.imwrite(pair_path(args.out_dir, lens, next_idx), half)
            for lens, det in zip(LENSES, dets):
                if det.n_corners >= args.min_corners:
                    add_coverage(coverage[lens], det)
            msg = f"Saved pair {next_idx:03d}: left {counts[0]} corners, right {counts[1]} corners"
            weak = [lens for lens, c in zip(LENSES, counts) if c < args.min_corners]
            if weak:
                msg += f"  WARNING: {' and '.join(weak)} has < {args.min_corners} corners and will be skipped for that lens"
            print(msg)
            next_idx += 1
            saved += 1
    finally:
        cap.release()
        cv2.destroyAllWindows()
    print(f"{saved} pairs in {args.out_dir}.")


# ---------------------------------------------------------------------------
# calibrate
# ---------------------------------------------------------------------------

@dataclass
class LensCalib:
    K: np.ndarray                   # 3x3 camera matrix (px)
    dist: np.ndarray                # 1xN distortion: k1 k2 p1 p2 k3 k4 k5 k6 (OpenCV 5 pads to 14 with zeros)
    image_size: Tuple[int, int]     # (width, height)
    rms: float = 0.0                # overall reprojection error (px)
    images: List[str] = field(default_factory=list)   # files used, so stereo can match pairs later
    per_view_errors: Optional[np.ndarray] = None
    std_intrinsics: Optional[np.ndarray] = None


def run_calibration(obj_pts, img_pts, size, names) -> LensCalib:
    # Rational model: extra radial terms k4..k6 handle the strong distortion of the ~120 deg lens.
    flags = cv2.CALIB_RATIONAL_MODEL
    criteria = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 100, 1e-7)
    rms, K, dist, _rvecs, _tvecs, std_int, _std_ext, per_view = cv2.calibrateCameraExtended(
        obj_pts, img_pts, size, None, None, flags=flags, criteria=criteria)
    return LensCalib(K, dist.reshape(1, -1), size, float(rms), list(names),
                     per_view.flatten(), std_int.flatten())


def report(lens: str, c: LensCalib, factor: float) -> List[str]:
    """Print per-image and overall errors; return the names of outlier images."""
    median = float(np.median(c.per_view_errors))
    limit = factor * median
    outliers = []
    print(f"  per-image RMS (px); outlier = above {limit:.3f} ({factor:g} x median {median:.3f})")
    for name, err in zip(c.images, c.per_view_errors):
        mark = ""
        if err > limit:
            mark = "   <-- outlier"
            outliers.append(name)
        print(f"    {name:<18}{err:7.3f}{mark}")
    s = c.std_intrinsics
    print(f"  fx {c.K[0, 0]:.1f} +/-{s[0]:.1f}   fy {c.K[1, 1]:.1f} +/-{s[1]:.1f}   "
          f"cx {c.K[0, 2]:.1f} +/-{s[2]:.1f}   cy {c.K[1, 2]:.1f} +/-{s[3]:.1f}")
    print(f"  dist k1 k2 p1 p2 k3 k4 k5 k6: {np.array2string(c.dist.ravel()[:8], precision=4)}")
    verdict = "OK" if c.rms < 0.5 else "above the 0.5 px target"
    print(f"  {lens.upper()} RMS {c.rms:.3f} px over {len(c.images)} images ({verdict})")
    return outliers


def calibrate_lens(lens: str, paths: List[str], finder: CharucoFinder, args) -> LensCalib:
    print(f"\n=== {lens.upper()} lens: {len(paths)} images ===")
    obj_pts, img_pts, names = [], [], []
    size = None
    for path in paths:
        name = os.path.basename(path)
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            print(f"  skip {name}: unreadable")
            continue
        h, w = img.shape
        if size is None:
            size = (w, h)
        elif (w, h) != size:
            sys.exit(f"ERROR: {name} is {w}x{h} but earlier images are {size[0]}x{size[1]}.")
        det = finder.detect(img)
        if det.n_corners < args.min_corners:
            print(f"  skip {name}: {det.n_corners} corners < {args.min_corners} ({det.n_markers} markers)")
            continue
        obj, imgp = finder.match_points(det)
        if not spans_plane(obj):
            print(f"  skip {name}: corners all on one row/column")
            continue
        obj_pts.append(obj)
        img_pts.append(imgp)
        names.append(name)

    if len(names) < MIN_VIEWS:
        sys.exit(f"ERROR: only {len(names)} usable {lens} images; need at least {MIN_VIEWS} "
                 f"(aim for 25-40). Check --dict/--legacy and the board sizes.")
    if len(names) < RECOMMENDED_VIEWS:
        print(f"  warning: only {len(names)} usable images; the rational model wants 25-40")

    calib = run_calibration(obj_pts, img_pts, size, names)
    outliers = report(lens, calib, args.outlier_factor)
    if outliers and args.drop_outliers:
        keep = [i for i, n in enumerate(names) if n not in outliers]
        if len(keep) < MIN_VIEWS:
            print(f"  not dropping outliers: only {len(keep)} images would remain")
        else:
            print(f"\n  recalibrating {lens} without {len(outliers)} outlier(s): {', '.join(outliers)}")
            calib = run_calibration([obj_pts[i] for i in keep], [img_pts[i] for i in keep],
                                    size, [names[i] for i in keep])
            report(lens, calib, args.outlier_factor)
    elif outliers:
        print("  (re-run with --drop-outliers to calibrate without them, or re-shoot those views)")
    return calib


def cmd_calibrate(args) -> None:
    finder = CharucoFinder(BoardConfig.from_args(args))
    results = {}
    for lens in LENSES:
        paths = sorted(glob.glob(os.path.join(args.images, f"{lens}_*.png")))
        if not paths:
            sys.exit(f"ERROR: no {lens}_*.png images in {args.images}. Run capture first.")
        results[lens] = calibrate_lens(lens, paths, finder, args)
    save_calib(args.output, results, finder.cfg)
    print("\nSummary: " + "   ".join(f"{lens} RMS {c.rms:.3f} px ({len(c.images)} images)"
                                     for lens, c in results.items()))
    print(f"Saved {args.output}")


# ---------------------------------------------------------------------------
# calib.yaml (cv2.FileStorage)
# ---------------------------------------------------------------------------

def save_calib(path: str, results: Dict[str, LensCalib], cfg: BoardConfig) -> None:
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    if not fs.isOpened():
        sys.exit(f"ERROR: cannot write {path}")
    fs.write("date", datetime.datetime.now().isoformat(timespec="seconds"))
    fs.write("opencv_version", cv2.__version__)

    fs.startWriteStruct("board", cv2.FileNode_MAP)
    fs.write("squares_x", cfg.squares_x)
    fs.write("squares_y", cfg.squares_y)
    fs.write("square_mm", float(cfg.square_mm))
    fs.write("marker_mm", float(cfg.marker_mm))
    fs.write("dictionary", cfg.dictionary)
    fs.write("legacy", int(cfg.legacy))
    fs.endWriteStruct()

    for lens, c in results.items():
        fs.startWriteStruct(lens, cv2.FileNode_MAP)
        fs.write("image_width", int(c.image_size[0]))
        fs.write("image_height", int(c.image_size[1]))
        fs.write("K", c.K)
        fs.write("dist", c.dist)
        fs.write("rms", c.rms)
        fs.write("num_images", len(c.images))
        fs.write("images", " ".join(c.images))
        fs.write("per_view_errors", c.per_view_errors.reshape(-1, 1))
        fs.endWriteStruct()
    fs.release()


def load_calib(path: str) -> Dict[str, LensCalib]:
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        sys.exit(f"ERROR: cannot read {path}. Run calibrate first.")
    out = {}
    for lens in LENSES:
        node = fs.getNode(lens)
        if node.empty():
            sys.exit(f"ERROR: {path} has no '{lens}' section.")
        size = (int(node.getNode("image_width").real()), int(node.getNode("image_height").real()))
        out[lens] = LensCalib(node.getNode("K").mat(), node.getNode("dist").mat(), size,
                              node.getNode("rms").real(), node.getNode("images").string().split())
    fs.release()
    return out


# ---------------------------------------------------------------------------
# live
# ---------------------------------------------------------------------------

def build_maps(c: LensCalib, alpha: float):
    """Undistortion lookup tables for one lens. Built once (and on alpha change); each frame is just a remap."""
    new_K, _roi = cv2.getOptimalNewCameraMatrix(c.K, c.dist, c.image_size, alpha, c.image_size)
    # Fixed-point CV_16SC2 maps are smaller and remap faster than float maps, which matters on a Pi/Jetson.
    return cv2.initUndistortRectifyMap(c.K, c.dist, None, new_K, c.image_size, cv2.CV_16SC2)


def undistort(img: np.ndarray, maps) -> np.ndarray:
    return cv2.remap(img, maps[0], maps[1], cv2.INTER_LINEAR)


def draw_grid(img: np.ndarray, spacing: int) -> None:
    """Straight reference lines: real-world straight edges should run parallel to them once undistorted."""
    h, w = img.shape[:2]
    for x in range(spacing, w, spacing):
        cv2.line(img, (x, 0), (x, h - 1), YELLOW, 1)
    for y in range(spacing, h, spacing):
        cv2.line(img, (0, y), (w - 1, y), YELLOW, 1)


def cmd_live(args) -> None:
    calib = load_calib(args.calib)
    for lens in LENSES:
        if calib[lens].image_size != (HALF_W, FRAME_H):
            sys.exit(f"ERROR: {args.calib} {lens} was calibrated at {calib[lens].image_size}, "
                     f"but the camera delivers {HALF_W}x{FRAME_H} per lens.")
    alpha = args.alpha
    maps = {lens: build_maps(calib[lens], alpha) for lens in LENSES}

    cap = open_camera(args.device, args.backend)
    view, show_grid = 1, False
    fps, last = 0.0, time.perf_counter()
    print("1: left raw|undistorted   2: right raw|undistorted   3: both undistorted   "
          "a/z: alpha +/-   g: grid   q: quit")
    try:
        while True:
            halves = dict(zip(LENSES, split_frame(read_frame(cap))))
            if view in (1, 2):
                # Only the lens on screen is remapped, to save CPU.
                lens = LENSES[view - 1]
                panels = [halves[lens], undistort(halves[lens], maps[lens])]
                labels = [f"{lens} raw", f"{lens} undistorted"]
            else:
                panels = [undistort(halves[lens], maps[lens]) for lens in LENSES]
                labels = ["left undistorted", "right undistorted"]

            disp = fit_width(cv2.hconcat(panels), args.display_width)
            if show_grid:
                draw_grid(disp, args.grid_px)
            panel_w = disp.shape[1] // 2
            for i, label in enumerate(labels):
                put_lines(disp, [label], org=(10 + i * panel_w, 30))

            now = time.perf_counter()
            dt, last = now - last, now
            fps = 1.0 / dt if fps == 0 else 0.9 * fps + 0.1 / dt   # smoothed
            put_lines(disp, [f"{fps:5.1f} fps   alpha {alpha:.1f}   1/2/3 view   a/z alpha   g grid   q quit"],
                      org=(10, disp.shape[0] - 15))
            cv2.imshow("live", disp)

            key = wait_key()
            if is_quit(key):
                break
            elif key in (ord("1"), ord("2"), ord("3")):
                view = key - ord("0")
            elif key in (ord("a"), ord("z")):
                step = 0.1 if key == ord("a") else -0.1
                alpha = round(min(1.0, max(0.0, alpha + step)), 1)
                maps = {lens: build_maps(calib[lens], alpha) for lens in LENSES}
                print(f"alpha {alpha:.1f} (0 = only valid pixels, 1 = full field of view)")
            elif key == ord("g"):
                show_grid = not show_grid
    finally:
        cap.release()
        cv2.destroyAllWindows()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def dict_name(value: str) -> str:
    """Accept '5X5_100' or 'DICT_5X5_100' (any case)."""
    value = value.upper()
    return value if value.startswith("DICT_") else "DICT_" + value


def build_parser() -> argparse.ArgumentParser:
    cam = argparse.ArgumentParser(add_help=False)
    cam.add_argument("--device", default="auto",
                     help="camera index, or 'auto' (default) to pick the camera that delivers 3200x1200")
    cam.add_argument("--backend", choices=["auto"] + list(BACKENDS), default="auto",
                     help="capture backend; auto = V4L2 on Linux, DirectShow on Windows")
    cam.add_argument("--display-width", type=int, default=1600,
                     help="downscale windows to this width (display only, default 1600)")

    geometry = argparse.ArgumentParser(add_help=False)
    geometry.add_argument("--squares-x", type=int, default=DEFAULT_SQUARES[0],
                          help=f"squares across, as the board was generated (default {DEFAULT_SQUARES[0]})")
    geometry.add_argument("--squares-y", type=int, default=DEFAULT_SQUARES[1],
                          help=f"squares down, as the board was generated (default {DEFAULT_SQUARES[1]})")
    geometry.add_argument("--square-mm", type=float, default=35.0, help="square side in mm (default 35)")
    geometry.add_argument("--marker-mm", type=float, default=25.0, help="marker side in mm (default 25)")

    board = argparse.ArgumentParser(add_help=False, parents=[geometry])
    board.add_argument("--dict", type=dict_name, choices=DICTS, default=DEFAULT_DICT,
                       help=f"ArUco dictionary, e.g. 5X5_100 (default {DEFAULT_DICT[5:]}; check with detect-dict)")
    board.add_argument("--legacy", action="store_true",
                       help="board was generated by OpenCV < 4.6 (setLegacyPattern)")

    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("detect-dict", parents=[cam, geometry],
                       help="find which ArUco dictionary the board uses")
    p.set_defaults(func=cmd_detect_dict)

    p = sub.add_parser("capture", parents=[cam, board], help="save left/right calibration image pairs")
    p.add_argument("--out-dir", default="calib_images", help="where pairs are saved (default calib_images)")
    p.add_argument("--min-corners", type=int, default=6, help="warn below this many corners (default 6)")
    p.add_argument("--log-counts", action="store_true", help="print marker/corner counts every frame")
    p.set_defaults(func=cmd_capture)

    p = sub.add_parser("calibrate", parents=[board], help="calibrate each lens -> calib.yaml")
    p.add_argument("--images", default="calib_images", help="folder with left_XXX/right_XXX.png")
    p.add_argument("--output", default="calib.yaml", help="output file (default calib.yaml)")
    p.add_argument("--min-corners", type=int, default=6, help="skip images with fewer corners (default 6)")
    p.add_argument("--outlier-factor", type=float, default=2.0,
                   help="flag images whose error exceeds this x the median (default 2)")
    p.add_argument("--drop-outliers", action="store_true", help="recalibrate without flagged images")
    p.set_defaults(func=cmd_calibrate)

    p = sub.add_parser("live", parents=[cam], help="raw vs undistorted live view")
    p.add_argument("--calib", default="calib.yaml", help="calibration file (default calib.yaml)")
    p.add_argument("--alpha", type=float, default=0.0,
                   help="0 = crop to valid pixels, 1 = keep full field of view (default 0)")
    p.add_argument("--grid-px", type=int, default=50, help="grid spacing on the display (default 50)")
    p.set_defaults(func=cmd_live)

    # Future: p = sub.add_parser("stereo", parents=[board], ...); p.set_defaults(func=cmd_stereo)
    return parser


def main() -> None:
    print(f"OpenCV {cv2.__version__}")
    if not HAS_ARUCO:
        sys.exit("ERROR: cv2.aruco is missing. Replace opencv-python with opencv-contrib-python: "
                 "uv remove opencv-python && uv add opencv-contrib-python")
    args = build_parser().parse_args()
    args.func(args)


if __name__ == "__main__":
    main()

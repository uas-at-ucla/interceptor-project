"""Visual + numeric check of stereo rectification.

Examples:
    python check_rectification.py --left L.png --right R.png --pattern 9x6
    python check_rectification.py --sbs pair.png --pattern 9x6 --out out/

Writes raw_sbs.png and rectified_sbs.png (with horizontal guide lines) to
--out. If --pattern is given and the checkerboard is found in both rectified
images, prints the vertical offset between matching corners.
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from rectify import StereoRectifier

LINE_SPACING_PX = 40
GOOD_DY_PX = 1.0
WARN_DY_PX = 2.0


def load_pair(args):
    if args.sbs:
        sbs = cv2.imread(args.sbs, cv2.IMREAD_UNCHANGED)
        if sbs is None:
            sys.exit(f"could not read {args.sbs}")
        half = sbs.shape[1] // 2
        return sbs[:, :half], sbs[:, half:2 * half]
    left = cv2.imread(args.left, cv2.IMREAD_UNCHANGED)
    right = cv2.imread(args.right, cv2.IMREAD_UNCHANGED)
    for path, img in ((args.left, left), (args.right, right)):
        if img is None:
            sys.exit(f"could not read {path}")
    return left, right


def side_by_side(left, right):
    sbs = np.hstack([left, right])
    if sbs.ndim == 2:
        sbs = cv2.cvtColor(sbs, cv2.COLOR_GRAY2BGR)
    for y in range(LINE_SPACING_PX, sbs.shape[0], LINE_SPACING_PX):
        cv2.line(sbs, (0, y), (sbs.shape[1] - 1, y), (0, 255, 0), 1)
    return sbs


def find_corners(img, pattern):
    gray = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ok, corners = cv2.findChessboardCornersSB(gray, pattern, flags=cv2.CALIB_CB_EXHAUSTIVE)
    if not ok:
        ok, corners = cv2.findChessboardCorners(gray, pattern)
        if ok:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-3)
            corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
    return corners.reshape(-1, 2) if ok else None


def row_offsets(left_pts, right_pts):
    """|dy| per matched corner. Detection can number the board from opposite
    ends in the two views, so use whichever ordering matches better."""
    dy = np.abs(left_pts[:, 1] - right_pts[:, 1])
    dy_rev = np.abs(left_pts[:, 1] - right_pts[::-1, 1])
    return dy if dy.mean() <= dy_rev.mean() else dy_rev


def parse_pattern(s):
    try:
        cols, rows = (int(v) for v in s.lower().split("x"))
    except ValueError:
        raise argparse.ArgumentTypeError("pattern must look like 9x6 (inner corners)")
    return cols, rows


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--calib", default="stereo_calibration.yaml")
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--sbs", help="single side-by-side image (left half | right half)")
    src.add_argument("--left", help="left image (requires --right)")
    p.add_argument("--right", help="right image")
    p.add_argument("--pattern", type=parse_pattern, help="checkerboard inner corners, COLSxROWS, e.g. 9x6")
    p.add_argument("--out", default="rectification_check", help="output directory")
    p.add_argument("--bench", type=int, default=50, help="rectify() iterations to time (0 to skip)")
    args = p.parse_args()
    if args.left and not args.right:
        p.error("--left requires --right")

    rect = StereoRectifier(args.calib)
    w, h = rect.image_size
    print(f"calibration : {args.calib} ({rect.model})")
    print(f"image size  : {w}x{h}")
    print(f"baseline    : {rect.baseline_m:.5f} m")
    print(f"fx, cx, cy  : {rect.fx:.2f}, {rect.cx:.2f}, {rect.cy:.2f} px")

    left, right = load_pair(args)
    left_r, right_r = rect.rectify(left, right)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out / "raw_sbs.png"), side_by_side(left, right))
    cv2.imwrite(str(out / "rectified_sbs.png"), side_by_side(left_r, right_r))
    print(f"wrote       : {out / 'raw_sbs.png'}, {out / 'rectified_sbs.png'}")

    if args.bench > 0:
        times = []
        for _ in range(args.bench):
            t0 = time.perf_counter()
            rect.rectify(left, right)
            times.append(time.perf_counter() - t0)
        times_ms = np.array(times) * 1e3
        print(f"rectify()   : mean {times_ms.mean():.2f} ms, max {times_ms.max():.2f} ms over {args.bench} pairs")

    if args.pattern:
        pts_l, pts_r = find_corners(left_r, args.pattern), find_corners(right_r, args.pattern)
        if pts_l is None or pts_r is None:
            missing = " and ".join(n for n, c in (("left", pts_l), ("right", pts_r)) if c is None)
            print(f"checkerboard {args.pattern[0]}x{args.pattern[1]} not found in rectified {missing}")
            return 1
        dy = row_offsets(pts_l, pts_r)
        print(f"row offset  : mean {dy.mean():.3f} px, max {dy.max():.3f} px over {len(dy)} corners")
        if dy.mean() > WARN_DY_PX:
            print(f"WARNING: mean vertical offset > {WARN_DY_PX} px; rectification or calibration is off")
        elif dy.mean() <= GOOD_DY_PX:
            print("OK: rows aligned to within 1 px")
        else:
            print("marginal: between 1 and 2 px")
    return 0


if __name__ == "__main__":
    sys.exit(main())

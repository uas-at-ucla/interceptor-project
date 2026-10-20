"""Live preview of the rectified stereo stream.

One side-by-side camera (left | right in one frame):
    python preview_rectified.py --sbs 0
Two cameras:
    python preview_rectified.py --left 0 --right 1

Keys: q quit, s save snapshot, r toggle raw/rectified.
Green lines every 40 px: the same feature should sit on the same line in both
halves of the rectified view.
"""

import argparse
import os
import sys
import time

# Windows Media Foundation hangs opening the stereo camera with hardware
# transforms on; must be set before cv2 is imported.
os.environ.setdefault("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS", "0")

import cv2
import numpy as np

from check_rectification import side_by_side
from rectify import StereoRectifier


def open_cam(index, width, height, fps):
    backend = cv2.CAP_V4L2 if sys.platform.startswith("linux") else cv2.CAP_ANY
    cap = cv2.VideoCapture(index, backend)
    if not cap.isOpened():
        raise SystemExit(f"could not open camera {index}")
    # Uncompressed YUYV at stereo resolutions usually exceeds USB 2 bandwidth; ask for MJPG.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    got_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    got_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"camera {index}: requested {width}x{height}@{fps}, got {got_w}x{got_h}@{cap.get(cv2.CAP_PROP_FPS):g}")
    if (got_w, got_h) != (width, height):
        print("WARNING: camera did not accept the calibration resolution; rectify() will reject its frames")
    return cap


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--calib", default="stereo_calibration.yaml")
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--sbs", type=int, help="device index of a side-by-side stereo camera")
    src.add_argument("--left", type=int, help="left camera device index (requires --right)")
    p.add_argument("--right", type=int)
    p.add_argument("--fps", type=int, default=60, help="frame rate to request from the camera")
    p.add_argument("--display-width", type=int, default=1600, help="window width in px")
    args = p.parse_args()
    if args.left is not None and args.right is None:
        p.error("--left requires --right")

    rect = StereoRectifier(args.calib)
    w, h = rect.image_size
    if args.sbs is not None:
        caps = [open_cam(args.sbs, 2 * w, h, args.fps)]
    else:
        caps = [open_cam(args.left, w, h, args.fps), open_cam(args.right, w, h, args.fps)]

    show_raw, n, t_sum, t_last = False, 0, 0.0, time.perf_counter()
    fps = 0.0
    while True:
        frames = []
        for cap in caps:
            ok, f = cap.read()
            if not ok:
                raise SystemExit("camera read failed")
            frames.append(f)
        if len(frames) == 1:
            half = frames[0].shape[1] // 2
            frames = [frames[0][:, :half], frames[0][:, half:2 * half]]
        left, right = frames

        left_r, right_r = rect.rectify(left, right)  # raises if the camera size != calibration size
        t_sum += rect.last_rectify_s
        n += 1
        now = time.perf_counter()
        if now - t_last >= 1.0:
            fps = n / (now - t_last)
            print(f"{fps:5.1f} fps   rectify() {1e3 * t_sum / n:6.2f} ms/pair")
            n, t_sum, t_last = 0, 0.0, now

        view = side_by_side(left, right) if show_raw else side_by_side(left_r, right_r)
        scale = args.display_width / view.shape[1]
        view = cv2.resize(view, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        label = "RAW" if show_raw else "RECTIFIED"
        cv2.putText(view, f"{label}  {fps:.1f} fps", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.imshow("stereo", view)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("r"):
            show_raw = not show_raw
        if key == ord("s"):
            stamp = time.strftime("%Y%m%d_%H%M%S")
            cv2.imwrite(f"snap_{stamp}_left.png", left)
            cv2.imwrite(f"snap_{stamp}_right.png", right)
            print(f"saved snap_{stamp}_left.png / _right.png (raw; feed to check_rectification.py)")

    for cap in caps:
        cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

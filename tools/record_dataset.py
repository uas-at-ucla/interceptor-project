"""Record raw stereo frames from the ELP camera into a dataset folder.

Each session goes into its own folder:

    datasets/<session>/
        frames/000000.png ...   full 3200x1200 side-by-side frames (left | right), lossless
        frames.csv              index, file, capture time (ns since epoch), distance label (m)
        meta.json               camera settings, marker info, notes

Frames are saved exactly as the camera delivers them (raw, not rectified), so the
vision node can be tested on the same input it gets live. Rectify and split them later
with calib.yaml.

Usage:
    python tools/record_dataset.py --session aruco_2m --distance 2.0
    python tools/record_dataset.py --device 1 --rate 15
    python tools/record_dataset.py --device clip.mp4 --record   # replay a video file (testing)

Keys (in the preview window):
    SPACE   save one frame
    r       start/stop continuous recording (at --rate fps)
    [ / ]   lower/raise the distance label by --distance-step
    d       toggle ArUco detection overlay (preview only, not saved)
    q/ESC   quit
"""

import argparse
import csv
import datetime
import json
import os
import queue
import sys
import threading
import time
from pathlib import Path

# Windows Media Foundation hangs opening the stereo camera with hardware
# transforms on; must be set before cv2 is imported.
os.environ.setdefault("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS", "0")

import cv2
import numpy as np

FRAME_W, FRAME_H = 3200, 1200  # ELP side-by-side mode
MAX_PROBE = 8
BACKENDS = {"v4l2": cv2.CAP_V4L2, "dshow": cv2.CAP_DSHOW, "msmf": cv2.CAP_MSMF, "any": cv2.CAP_ANY}
GREEN, RED, YELLOW, WHITE = (0, 255, 0), (0, 0, 255), (0, 255, 255), (255, 255, 255)


# ---------------------------------------------------------------------------
# Camera (same open sequence as stereo_calib.py on the calibration branch)
# ---------------------------------------------------------------------------

def try_open(index, backend, fps):
    cap = cv2.VideoCapture(index, BACKENDS[backend])
    if not cap.isOpened():
        return None, None
    # FOURCC goes first (V4L2 only offers high-res modes as MJPG) and last
    # (DirectShow resets the format when the size or fps changes).
    mjpg = cv2.VideoWriter_fourcc(*"MJPG")
    cap.set(cv2.CAP_PROP_FOURCC, mjpg)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_FOURCC, mjpg)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for _ in range(30):  # first reads after a mode switch can be empty
        ok, frame = cap.read()
        if ok and frame is not None:
            return cap, (frame.shape[1], frame.shape[0])
    cap.release()
    return None, None


def open_source(device, backend, fps):
    """Return (cap, description). device is 'auto', a camera index, or a video file path."""
    if device not in ("auto",) and not device.isdigit():
        cap = cv2.VideoCapture(device)
        if not cap.isOpened():
            sys.exit(f"ERROR: could not open video file {device}")
        return cap, f"file:{device}"

    if backend == "auto":
        backend = {"linux": "v4l2", "win32": "dshow"}.get(sys.platform, "any")
    indices = range(MAX_PROBE) if device == "auto" else [int(device)]
    for index in indices:
        cap, size = try_open(index, backend, fps)
        if cap is None:
            continue
        if size == (FRAME_W, FRAME_H):
            print(f"Camera {index} ({backend}): {size[0]}x{size[1]}, driver reports "
                  f"{cap.get(cv2.CAP_PROP_FPS):.0f} fps")
            return cap, f"camera:{index}:{backend}"
        print(f"  camera {index}: {size[0]}x{size[1]} (not the ELP, skipping)")
        cap.release()
    sys.exit(f"ERROR: no camera delivered {FRAME_W}x{FRAME_H}. Check the ELP is plugged in and "
             f"not open in another app, or try --device N / --backend.")


# ---------------------------------------------------------------------------
# Background writer so PNG encoding doesn't stall the capture loop
# ---------------------------------------------------------------------------

class FrameWriter:
    def __init__(self, session_dir, ext, threads=3, max_queue=64):
        self.frames_dir = session_dir / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.ext = ext
        self.params = [cv2.IMWRITE_PNG_COMPRESSION, 1] if ext == "png" else [cv2.IMWRITE_JPEG_QUALITY, 95]
        self.queue = queue.Queue(maxsize=max_queue)
        self.dropped = 0
        self.saved = 0
        self.lock = threading.Lock()

        csv_path = session_dir / "frames.csv"
        new_file = not csv_path.exists()
        self.csv_path = csv_path
        self.csv_file = open(csv_path, "a", newline="")
        self.csv = csv.writer(self.csv_file)
        if new_file:
            self.csv.writerow(["index", "file", "t_ns", "distance_m"])
        self.next_index = self._first_free_index()

        self.workers = [threading.Thread(target=self._run, daemon=True) for _ in range(threads)]
        for w in self.workers:
            w.start()

    def _first_free_index(self):
        existing = [int(p.stem) for p in self.frames_dir.iterdir() if p.stem.isdigit()]
        return max(existing) + 1 if existing else 0

    def submit(self, frame, t_ns, distance, block=False):
        name = f"{self.next_index:06d}.{self.ext}"
        try:
            self.queue.put((name, frame, t_ns, distance, self.next_index), block=block)
            self.next_index += 1
        except queue.Full:
            self.dropped += 1

    def _run(self):
        while True:
            item = self.queue.get()
            if item is None:
                return
            name, frame, t_ns, distance, index = item
            ok = cv2.imwrite(str(self.frames_dir / name), frame, self.params)
            with self.lock:
                if ok:
                    self.saved += 1
                    dist = "" if distance is None else f"{distance:.3f}"
                    self.csv.writerow([index, f"frames/{name}", t_ns, dist])
                    self.csv_file.flush()  # keep labels if the process is killed
                else:
                    print(f"ERROR: failed to write {name}")

    def close(self):
        pending = self.queue.qsize()
        if pending:
            print(f"Flushing {pending} queued frames...")
        for _ in self.workers:
            self.queue.put(None)
        for w in self.workers:
            w.join()
        self.csv_file.close()
        # Worker threads finish out of order; sort rows by frame index.
        with open(self.csv_path, newline="") as f:
            header, *rows = list(csv.reader(f))
        rows.sort(key=lambda r: int(r[0]))
        with open(self.csv_path, "w", newline="") as f:
            csv.writer(f).writerows([header, *rows])


# ---------------------------------------------------------------------------
# Preview
# ---------------------------------------------------------------------------

def make_detector(dict_name):
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    return cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())


def draw_detections(view, detector, n_halves):
    """Detect markers on the (downscaled) preview and label each half. Returns ids seen per half."""
    half_w = view.shape[1] // n_halves
    seen = []
    for i in range(n_halves):
        half = view[:, i * half_w:(i + 1) * half_w]
        corners, ids, _ = detector.detectMarkers(half)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(half, corners, ids)
            seen.append(sorted(int(x) for x in ids.flatten()))
        else:
            seen.append([])
    return seen


def put_lines(img, lines, org=(10, 28), scale=0.65):
    x, y = org
    for text, color in lines:
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4)
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2)
        y += int(32 * scale / 0.65)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default="datasets", help="parent folder for sessions (default: datasets)")
    p.add_argument("--session", help="session folder name (default: timestamp). Reusing a name appends.")
    p.add_argument("--device", default="auto", help="'auto', a camera index, or a video file path")
    p.add_argument("--backend", default="auto", choices=["auto", *BACKENDS])
    p.add_argument("--fps", type=int, default=60, help="frame rate requested from the camera")
    p.add_argument("--record", action="store_true", help="start recording immediately")
    p.add_argument("--no-preview", action="store_true",
                   help="no window (e.g. over SSH): records immediately until Ctrl+C or --duration")
    p.add_argument("--duration", type=float, help="stop after this many seconds")
    p.add_argument("--rate", type=float, default=10.0,
                   help="frames per second saved while recording (0 = every frame)")
    p.add_argument("--format", choices=["png", "jpg"], default="png",
                   help="png is lossless (best for depth); jpg is ~10x smaller")
    p.add_argument("--distance", type=float, help="measured camera-to-marker distance in meters")
    p.add_argument("--distance-step", type=float, default=0.5, help="step for [ and ] keys (m)")
    p.add_argument("--dict", default="DICT_6X6_250", help="ArUco dictionary for the preview overlay")
    p.add_argument("--marker-mm", type=float, help="printed marker side length in mm (saved to meta.json)")
    p.add_argument("--notes", default="", help="free text saved to meta.json (lighting, location...)")
    p.add_argument("--display-width", type=int, default=1600)
    args = p.parse_args()

    cap, source = open_source(args.device, args.backend, args.fps)

    session = args.session or datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = Path(args.out) / session
    session_dir.mkdir(parents=True, exist_ok=True)
    writer = FrameWriter(session_dir, args.format)

    meta_path = session_dir / "meta.json"
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else {"runs": []}
    run = {
        "started": datetime.datetime.now().isoformat(timespec="seconds"),
        "source": source,
        "frame_size": [FRAME_W, FRAME_H],
        "layout": "side_by_side_left_right",
        "rectified": False,
        "requested_fps": args.fps,
        "save_rate": args.rate,
        "format": args.format,
        "aruco_dict": args.dict,
        "marker_mm": args.marker_mm,
        "distance_m": args.distance,
        "notes": args.notes,
        "opencv": cv2.__version__,
    }
    meta["runs"].append(run)
    meta_path.write_text(json.dumps(meta, indent=2))

    detector = make_detector(args.dict)
    distance = args.distance
    preview = not args.no_preview
    recording, show_det = args.record or not preview, True
    last_save = 0.0
    fps_n, fps_t, fps = 0, time.perf_counter(), 0.0
    is_file = source.startswith("file:")
    window = "record_dataset"
    if preview:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    print(f"Saving to {session_dir.resolve()}")
    if preview:
        print("SPACE save frame | r record | [ ] distance | d detection overlay | q quit")
    else:
        print("Recording without preview; Ctrl+C to stop.")
    t_start = time.perf_counter()

    try:
        while True:
            ok, frame = cap.read()
            t_ns = time.time_ns()
            if not ok or frame is None:
                if is_file:
                    print("End of video file.")
                    break
                sys.exit("ERROR: camera stopped delivering frames.")

            fps_n += 1
            now = time.perf_counter()
            fps_updated = now - fps_t >= 1.0
            if fps_updated:
                fps, fps_n, fps_t = fps_n / (now - fps_t), 0, now

            if args.duration and now - t_start >= args.duration:
                break
            if recording and (args.rate <= 0 or now - last_save >= 1.0 / args.rate):
                # Video files can be read faster than PNGs are written, so wait instead of dropping.
                writer.submit(frame, t_ns, distance, block=is_file)  # frame is not modified after this
                last_save = now
            if not preview:
                if fps_updated:
                    print(f"  cam {fps:.1f} fps  saved {writer.saved}  dropped {writer.dropped}")
                continue

            scale = args.display_width / frame.shape[1]
            view = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            n_halves = 2 if frame.shape[1] == FRAME_W else 1
            lines = []
            if show_det:
                seen = draw_detections(view, detector, n_halves)
                labels = ["L", "R"] if n_halves == 2 else ["img"]
                det = "  ".join(f"{lab}:{ids or '-'}" for lab, ids in zip(labels, seen))
                lines.append((f"markers {det}", GREEN if all(seen) else YELLOW))
            dist_txt = "unset" if distance is None else f"{distance:.2f} m"
            status = "REC" if recording else "idle"
            lines.insert(0, (f"{status}  cam {fps:.1f} fps  saved {writer.saved}  "
                             f"dropped {writer.dropped}  distance {dist_txt}",
                             RED if recording else WHITE))
            put_lines(view, lines)
            if recording:
                cv2.circle(view, (view.shape[1] - 25, 25), 10, RED, -1)
            cv2.imshow(window, view)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            elif key == ord(" "):
                writer.submit(frame, t_ns, distance)
            elif key == ord("r"):
                recording = not recording
                print("Recording started" if recording else f"Recording stopped ({writer.saved} saved)")
            elif key in (ord("["), ord("]")):
                base = distance if distance is not None else 0.0
                step = args.distance_step if key == ord("]") else -args.distance_step
                distance = max(0.0, round(base + step, 3))
                print(f"Distance label: {distance:.2f} m")
            elif key == ord("d"):
                show_det = not show_det
    except KeyboardInterrupt:
        print()
    finally:
        cap.release()
        if preview:
            cv2.destroyAllWindows()
        writer.close()
        run["ended"] = datetime.datetime.now().isoformat(timespec="seconds")
        run["frames_saved"] = writer.saved
        run["frames_dropped"] = writer.dropped
        meta_path.write_text(json.dumps(meta, indent=2))
        print(f"Saved {writer.saved} frames to {session_dir} ({writer.dropped} dropped)")


if __name__ == "__main__":
    main()

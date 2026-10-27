# interceptor-project
This repository contains software for Research and Development's interceptor drone project

## Setup

```
uv sync
```

## Stereo camera lens calibration (`stereo_calib.py`)

Calibrates the two lenses of the ELP binocular camera (3200x1200 side-by-side MJPEG)
individually with a ChArUco board, then shows a live undistorted stream.
All lengths are in mm.

The defaults match our board: 6X6_50 markers, generated as 5 squares across x 7 down
(it looks 7x5 when held landscape; either orientation works), 35 mm squares, 25 mm markers.

```
uv run stereo_calib.py detect-dict     # SPACE on a live frame -> checks dictionary and layout
uv run stereo_calib.py capture         # SPACE saves calib_images/left_XXX.png + right_XXX.png
uv run stereo_calib.py calibrate       # -> calib.yaml (add --drop-outliers to refine)
uv run stereo_calib.py live            # 1/2/3 views, a/z alpha, g grid, q quit
```

`--device auto` (the default) picks the first camera that delivers 3200x1200, so a built-in
webcam is skipped; pass an index to force one. `--backend auto`
uses V4L2 on Linux and DirectShow on Windows. Board options: `--square-mm` (default 35),
`--marker-mm` (default 25), `--squares-x/--squares-y`, `--dict`, and `--legacy` for boards made
by old OpenCV. For a different board, use the same options for `capture` and `calibrate`.

### Capture technique

- **Focus first.** Focus both lenses at your working distance and lock them (set screw or glue).
  Any focus change after calibrating invalidates it.
- **25-40 pairs.** Fewer than ~15 usable images per lens is not enough for the rational model.
- **Cover the edges and corners.** Distortion is worst there, and that's where calibrations go wrong.
  Watch the green coverage overlay and the `cells` score for each lens, and keep going until the
  frame corners are filled.
- **Vary distance and tilt.** Mix near views (board fills the frame) and far views, and tilt the
  board about 20-45 degrees in different directions. Flat-on views alone give a poor focal length.
- **Flat and still.** Mount the print on something rigid (foam board or glass), keep it still when
  you press SPACE, and avoid glare. Motion blur and a curled print both show up as high error.
- **Watch detection stability.** If the marker min..max range jumps around while the board is still,
  the faint streaks in the print are confusing the detector. Improve lighting or reprint.
- **Check the result.** Aim for an overall RMS below 0.5 px per lens. Re-shoot or drop images flagged
  as outliers (more than 2x the median error), then run `live` with the grid (`g`) and confirm that
  straight edges near the frame borders come out straight.

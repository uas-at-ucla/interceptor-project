import cv2
import numpy as np


def load_stereo_calib(yaml_path):
  fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
  w = int(fs.getNode("image_width").real())
  h = int(fs.getNode("image_height").real())
  calib = {
      "size": (w, h),
      "K1": fs.getNode("K1").mat(),
      "D1": fs.getNode("D1").mat(),
      "K2": fs.getNode("K2").mat(),
      "D2": fs.getNode("D2").mat(),
      "R1": fs.getNode("R1").mat(),
      "R2": fs.getNode("R2").mat(),
      "P1": fs.getNode("P1").mat(),
      "P2": fs.getNode("P2").mat(),
      "Q": fs.getNode("Q").mat(),
  }
  fs.release()
  return calib


# Load stereo calibration parameters
calib = load_stereo_calib("calib.yaml")
size = calib["size"]

# Pre-compute rectification maps
mapL1, mapL2 = cv2.initUndistortRectifyMap(
    calib["K1"], calib["D1"], calib["R1"], calib["P1"], size, cv2.CV_16SC2
)
mapR1, mapR2 = cv2.initUndistortRectifyMap(
    calib["K2"], calib["D2"], calib["R2"], calib["P2"], size, cv2.CV_16SC2
)

# ArUco Detector Setup (matching your target tag family)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

cap = cv2.VideoCapture(0)

while True:
  ret, frame = cap.read()
  if not ret:
    break

  half_w = size[0]
  raw_l, raw_r = frame[:, :half_w], frame[:, half_w:]

  # 1. Rectify images so corresponding features share identical row positions (y_L == y_R)
  rect_l = cv2.remap(raw_l, mapL1, mapL2, cv2.INTER_LINEAR)
  rect_r = cv2.remap(raw_r, mapR1, mapR2, cv2.INTER_LINEAR)

  # 2. Detect ArUco tags in both rectified frames
  corners_l, ids_l, _ = detector.detectMarkers(rect_l)
  corners_r, ids_r, _ = detector.detectMarkers(rect_r)

  # 3. Process tags visible to BOTH cameras simultaneously
  if ids_l is not None and ids_r is not None:
    ids_l_flat = ids_l.flatten()
    ids_r_flat = ids_r.flatten()

    for i, tag_id in enumerate(ids_l_flat):
      if tag_id in ids_r_flat:
        j = np.where(ids_r_flat == tag_id)[0][0]

        # Get 4 corners (shape: 4x2)
        pts_l = corners_l[i][0]
        pts_r = corners_r[j][0]

        # Calculate disparity per corner: d = x_left - x_right
        disparities = pts_l[:, 0] - pts_r[:, 0]

        # Reproject 2D corners + disparity to 3D world points (X, Y, Z in meters)
        corners_3d = []
        for (x, y), d in zip(pts_l, disparities):
          if d > 0:
            # Homogeneous coordinate vector [x, y, disparity, 1]
            vec_4d = np.array([x, y, d, 1.0])
            point_3d = calib["Q"] @ vec_4d
            point_3d /= point_3d[3]  # Normalize by W component
            corners_3d.append(point_3d[:3])

        if len(corners_3d) == 4:
          # Average the 4 corners to get the Tag Center 3D Coordinate
          target_3d = np.mean(corners_3d, axis=0)
          X, Y, Z = target_3d

          # Draw tag overlay & output distance
          cv2.aruco.drawDetectedMarkers(rect_l, corners_l)
          text = f"ID {tag_id}: X={X:.2f}m Y={Y:.2f}m Z={Z:.2f}m"
          cv2.putText(
              rect_l,
              text,
              (10, 50),
              cv2.FONT_HERSHEY_SIMPLEX,
              0.7,
              (0, 255, 0),
              2,
          )

          print(
              f"Target Position -> X (Right/Left): {X:+.2f}m | Y (Down/Up):"
              f" {Y:+.2f}m | Z (Forward): {Z:.2f}m"
          )

  cv2.imshow("Drone Interceptor Feed (Rectified Left)", rect_l)
  if cv2.waitKey(1) & 0xFF == ord("q"):
    break

cap.release()
cv2.destroyAllWindows()
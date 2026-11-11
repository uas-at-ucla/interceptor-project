# Vehicle state from the flight controller's local position estimate (via MAVROS).
# All quantities are in the ENU local frame named by header.frame_id ("map"),
# whose origin is the PX4 EKF origin (normally where the vehicle booted).

std_msgs/Header header

geometry_msgs/Point position        # m, ENU
geometry_msgs/Vector3 velocity      # m/s, ENU (world frame, not body frame)
geometry_msgs/Quaternion orientation  # body (FLU) relative to ENU

bool connected  # FCU link is up
bool armed
string mode     # PX4 flight mode, e.g. "POSCTL", "OFFBOARD"

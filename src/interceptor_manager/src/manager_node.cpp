// Manager node: turns high-level commands into MAVROS commands.
//
// Inputs
//   /setpoint (geometry_msgs/msg/Point): position offset in metres, relative to
//     the drone's current position. Axes depend on the `setpoint_frame` param:
//       "local" (default): ENU, x east, y north, z up
//       "body":            x forward, y left, z up (rotated by current yaw only)
//
// Behaviour
//   - Streams a position target to MAVROS at `setpoint_rate_hz` once a local
//     pose is available. With no active command it holds the current position,
//     so switching to OFFBOARD never causes a jump.
//   - On a /setpoint, latches target = current position + offset, keeping the
//     current yaw. If `auto_engage` is true it then switches to OFFBOARD and arms.
//   - If the vehicle leaves OFFBOARD (pilot takeover, failsafe) or disarms, the
//     active command is dropped and the node stops trying to engage.
//
// Debug from a terminal:
//   ros2 topic pub --once /setpoint geometry_msgs/msg/Point "{x: 0.0, y: 0.0, z: 3.0}"

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "interceptor_manager/mavros_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace interceptor_manager
{

namespace
{

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(yaw / 2.0);
  q.z = std::sin(yaw / 2.0);
  return q;
}

}  // namespace

class ManagerNode : public rclcpp::Node
{
public:
  ManagerNode()
  : Node("manager_node"),
    setpoint_frame_(declare_parameter<std::string>("setpoint_frame", "local")),
    auto_engage_(declare_parameter<bool>("auto_engage", true)),
    max_offset_m_(declare_parameter<double>("max_offset_m", 50.0)),
    mavros_(*this, declare_parameter<std::string>("mavros_ns", "/mavros"))
  {
    if (setpoint_frame_ != "local" && setpoint_frame_ != "body") {
      RCLCPP_WARN(
        get_logger(), "Unknown setpoint_frame '%s', using 'local'", setpoint_frame_.c_str());
      setpoint_frame_ = "local";
    }

    const double rate_hz = declare_parameter<double>("setpoint_rate_hz", 20.0);

    setpoint_sub_ = create_subscription<geometry_msgs::msg::Point>(
      "/setpoint", 10,
      [this](geometry_msgs::msg::Point::ConstSharedPtr msg) {on_relative_setpoint(*msg);});

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz), [this]() {on_timer();});

    RCLCPP_INFO(
      get_logger(), "Manager ready: /setpoint offsets in '%s' frame, auto_engage=%s",
      setpoint_frame_.c_str(), auto_engage_ ? "true" : "false");
  }

private:
  void on_relative_setpoint(const geometry_msgs::msg::Point & offset)
  {
    if (!std::isfinite(offset.x) || !std::isfinite(offset.y) || !std::isfinite(offset.z)) {
      RCLCPP_WARN(get_logger(), "Rejected setpoint: non-finite value");
      return;
    }
    const double norm = std::sqrt(offset.x * offset.x + offset.y * offset.y + offset.z * offset.z);
    if (norm > max_offset_m_) {
      RCLCPP_WARN(
        get_logger(), "Rejected setpoint: offset %.2f m exceeds max_offset_m (%.2f m)",
        norm, max_offset_m_);
      return;
    }
    if (!mavros_.connected()) {
      RCLCPP_WARN(get_logger(), "Rejected setpoint: FCU not connected");
      return;
    }
    const auto & current = mavros_.local_pose();
    if (!current) {
      RCLCPP_WARN(get_logger(), "Rejected setpoint: no local position yet");
      return;
    }

    const auto & position = current->pose.position;
    const double yaw = yaw_from_quaternion(current->pose.orientation);

    // Rotate the offset into the local ENU frame.
    double dx = offset.x;
    double dy = offset.y;
    if (setpoint_frame_ == "body") {
      dx = offset.x * std::cos(yaw) - offset.y * std::sin(yaw);
      dy = offset.x * std::sin(yaw) + offset.y * std::cos(yaw);
    }

    geometry_msgs::msg::Pose target;
    target.position.x = position.x + dx;
    target.position.y = position.y + dy;
    target.position.z = position.z + offset.z;
    target.orientation = quaternion_from_yaw(yaw);
    target_ = target;

    RCLCPP_INFO(
      get_logger(), "New target (%.2f, %.2f, %.2f) [offset %.2f, %.2f, %.2f %s]",
      target.position.x, target.position.y, target.position.z,
      offset.x, offset.y, offset.z, setpoint_frame_.c_str());
  }

  void on_timer()
  {
    const auto & current = mavros_.local_pose();
    if (!current) {
      return;  // Nothing sensible to stream until we know where we are.
    }

    // Drop the command if someone else took control or the vehicle disarmed,
    // so we don't fight the pilot or re-arm after landing.
    if (target_ &&
      ((was_offboard_ && !mavros_.in_offboard()) || (was_armed_ && !mavros_.armed())))
    {
      RCLCPP_WARN(
        get_logger(), "Vehicle left OFFBOARD or disarmed (mode %s, %s); clearing target",
        mavros_.mode().c_str(), mavros_.armed() ? "armed" : "disarmed");
      target_.reset();
    }
    was_offboard_ = mavros_.in_offboard();
    was_armed_ = mavros_.armed();

    // In OFFBOARD with no command (e.g. switched in from a GCS): hold here.
    if (!target_ && mavros_.in_offboard()) {
      geometry_msgs::msg::Pose hold = current->pose;
      hold.orientation = quaternion_from_yaw(yaw_from_quaternion(current->pose.orientation));
      target_ = hold;
    }

    mavros_.send_position_target(target_ ? *target_ : current->pose);

    if (target_ && auto_engage_) {
      engage();
    }
  }

  // Get into OFFBOARD + armed, retrying at most once per second.
  void engage()
  {
    if (mavros_.in_offboard() && mavros_.armed()) {
      return;
    }
    const auto now = this->now();
    if (last_engage_attempt_ && (now - *last_engage_attempt_) < rclcpp::Duration::from_seconds(1.0)) {
      return;
    }
    last_engage_attempt_ = now;

    if (!mavros_.in_offboard()) {
      mavros_.request_mode("OFFBOARD");
    } else {
      mavros_.request_arming(true);
    }
  }

  std::string setpoint_frame_;
  const bool auto_engage_;
  const double max_offset_m_;

  MavrosInterface mavros_;

  std::optional<geometry_msgs::msg::Pose> target_;
  bool was_offboard_ = false;
  bool was_armed_ = false;
  std::optional<rclcpp::Time> last_engage_attempt_;

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr setpoint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace interceptor_manager

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<interceptor_manager::ManagerNode>());
  rclcpp::shutdown();
  return 0;
}

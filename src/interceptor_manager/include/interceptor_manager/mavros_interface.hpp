#pragma once

#include <optional>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"

namespace interceptor_manager
{

// Thin wrapper around the MAVROS topics/services the manager needs.
//
// Everything MAVROS-specific (topic names, message types, service calls) lives
// here so the manager's decision logic doesn't depend on it directly. To add a
// new kind of command (velocity, attitude, trajectory...), add a method here and
// call it from the manager.
//
// All positions are in MAVROS's local ENU frame ("map"): x east, y north, z up.
class MavrosInterface
{
public:
  // `mavros_ns` is the MAVROS node namespace, e.g. "/mavros" or "/uav1/mavros".
  MavrosInterface(rclcpp::Node & node, const std::string & mavros_ns);

  // --- Vehicle state ---------------------------------------------------------
  bool connected() const {return state_.connected;}
  bool armed() const {return state_.armed;}
  const std::string & mode() const {return state_.mode;}
  bool in_offboard() const {return state_.mode == "OFFBOARD";}

  // Latest local position estimate, or nullopt if none has been received yet.
  const std::optional<geometry_msgs::msg::PoseStamped> & local_pose() const {return local_pose_;}

  // --- Commands --------------------------------------------------------------
  // Publish one local-frame position target. PX4 offboard needs a continuous
  // stream of these (> 2 Hz), so call this periodically.
  void send_position_target(const geometry_msgs::msg::Pose & pose);

  // Asynchronous requests. Results are logged; state changes show up via
  // mode()/armed() once MAVROS reports them.
  void request_mode(const std::string & mode);
  void request_arming(bool arm);

private:
  rclcpp::Node & node_;

  mavros_msgs::msg::State state_;
  std::optional<geometry_msgs::msg::PoseStamped> local_pose_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr position_target_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
};

}  // namespace interceptor_manager

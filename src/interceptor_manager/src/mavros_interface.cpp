#include "interceptor_manager/mavros_interface.hpp"

namespace interceptor_manager
{

MavrosInterface::MavrosInterface(rclcpp::Node & node, const std::string & mavros_ns)
: node_(node)
{
  state_sub_ = node_.create_subscription<mavros_msgs::msg::State>(
    mavros_ns + "/state", 10,
    [this](mavros_msgs::msg::State::ConstSharedPtr msg) {
      if (msg->connected != state_.connected) {
        RCLCPP_INFO(node_.get_logger(), "FCU %s", msg->connected ? "connected" : "disconnected");
      }
      if (msg->mode != state_.mode) {
        RCLCPP_INFO(node_.get_logger(), "Mode: %s", msg->mode.c_str());
      }
      if (msg->armed != state_.armed) {
        RCLCPP_INFO(node_.get_logger(), "%s", msg->armed ? "Armed" : "Disarmed");
      }
      state_ = *msg;
    });

  // MAVROS publishes local position with sensor-data (best-effort) QoS.
  pose_sub_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
    mavros_ns + "/local_position/pose", rclcpp::SensorDataQoS(),
    [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {local_pose_ = *msg;});

  position_target_pub_ = node_.create_publisher<geometry_msgs::msg::PoseStamped>(
    mavros_ns + "/setpoint_position/local", 10);

  set_mode_client_ = node_.create_client<mavros_msgs::srv::SetMode>(mavros_ns + "/set_mode");
  arming_client_ = node_.create_client<mavros_msgs::srv::CommandBool>(mavros_ns + "/cmd/arming");
}

void MavrosInterface::send_position_target(const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = node_.now();
  msg.header.frame_id = "map";
  msg.pose = pose;
  position_target_pub_->publish(msg);
}

void MavrosInterface::request_mode(const std::string & mode)
{
  if (!set_mode_client_->service_is_ready()) {
    RCLCPP_WARN(node_.get_logger(), "set_mode service not available; is MAVROS running?");
    return;
  }
  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->custom_mode = mode;
  RCLCPP_INFO(node_.get_logger(), "Requesting mode %s", mode.c_str());
  set_mode_client_->async_send_request(
    request,
    [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
      if (!future.get()->mode_sent) {
        RCLCPP_WARN(node_.get_logger(), "Mode change to %s was rejected", mode.c_str());
      }
    });
}

void MavrosInterface::request_arming(bool arm)
{
  if (!arming_client_->service_is_ready()) {
    RCLCPP_WARN(node_.get_logger(), "arming service not available; is MAVROS running?");
    return;
  }
  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = arm;
  RCLCPP_INFO(node_.get_logger(), "Requesting %s", arm ? "arm" : "disarm");
  arming_client_->async_send_request(
    request,
    [this, arm](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
      if (!future.get()->success) {
        RCLCPP_WARN(node_.get_logger(), "%s request was rejected", arm ? "Arm" : "Disarm");
      }
    });
}

}  // namespace interceptor_manager

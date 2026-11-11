// Manager node: flies the drone to a position relative to where it is now.
//
// Debug from a terminal:
//   ros2 topic pub --once /setpoint geometry_msgs/msg/Point '{x: 0.0, y: 0.0, z: 3.0}'
//
// x = east, y = north, z = up (metres). See README.md.

#include <chrono>
#include <memory>
#include <optional>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::PoseStamped;
using mavros_msgs::msg::State;
using mavros_msgs::srv::CommandBool;
using mavros_msgs::srv::SetMode;

class ManagerNode : public rclcpp::Node
{
public:
  ManagerNode()
  : Node("manager_node")
  {
    // Inputs
    setpoint_sub_ = create_subscription<Point>(
      "/setpoint", 10, [this](Point::SharedPtr msg) {on_setpoint(*msg);});
    state_sub_ = create_subscription<State>(
      "/mavros/state", 10, [this](State::SharedPtr msg) {state_ = *msg;});
    pose_sub_ = create_subscription<PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      [this](PoseStamped::SharedPtr msg) {pose_ = msg;});

    // Outputs
    target_pub_ = create_publisher<PoseStamped>("/mavros/setpoint_position/local", 10);
    mode_client_ = create_client<SetMode>("/mavros/set_mode");
    arm_client_ = create_client<CommandBool>("/mavros/cmd/arming");

    stream_timer_ = create_wall_timer(50ms, [this]() {stream_target();});  // 20 Hz
    engage_timer_ = create_wall_timer(1s, [this]() {engage();});
  }

private:
  void on_setpoint(const Point & offset)
  {
    if (!pose_) {
      RCLCPP_WARN(get_logger(), "No position from MAVROS yet; is it running?");
      return;
    }
    target_ = *pose_;
    target_->pose.position.x += offset.x;
    target_->pose.position.y += offset.y;
    target_->pose.position.z += offset.z;
    RCLCPP_INFO(
      get_logger(), "New target (%.2f, %.2f, %.2f)",
      target_->pose.position.x, target_->pose.position.y, target_->pose.position.z);
  }

  // PX4 drops out of OFFBOARD if targets stop arriving, so send one every tick.
  // Before the first setpoint, send the current position so switching to
  // OFFBOARD doesn't make the drone jump.
  void stream_target()
  {
    if (!pose_) {
      return;
    }
    PoseStamped msg = target_ ? *target_ : *pose_;
    msg.header.stamp = now();
    target_pub_->publish(msg);
  }

  // Once there is a target, get into OFFBOARD and armed (retried every second).
  void engage()
  {
    if (!target_) {
      return;
    }
    if (state_.mode != "OFFBOARD") {
      auto request = std::make_shared<SetMode::Request>();
      request->custom_mode = "OFFBOARD";
      mode_client_->async_send_request(request);
    } else if (!state_.armed) {
      auto request = std::make_shared<CommandBool::Request>();
      request->value = true;
      arm_client_->async_send_request(request);
    }
  }

  State state_;
  PoseStamped::SharedPtr pose_;
  std::optional<PoseStamped> target_;

  rclcpp::Subscription<Point>::SharedPtr setpoint_sub_;
  rclcpp::Subscription<State>::SharedPtr state_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr target_pub_;
  rclcpp::Client<SetMode>::SharedPtr mode_client_;
  rclcpp::Client<CommandBool>::SharedPtr arm_client_;
  rclcpp::TimerBase::SharedPtr stream_timer_;
  rclcpp::TimerBase::SharedPtr engage_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManagerNode>());
  rclcpp::shutdown();
  return 0;
}

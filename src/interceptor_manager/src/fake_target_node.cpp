// Fake target node: TEST TOOL ONLY. It stands in for the Tracker so the
// manager's FOLLOW and SEARCH states can be tested in sim before the real
// Tracker exists.
//
// Publishes interceptor_msgs/TargetData on /target_data at 10 Hz for a target
// flying a horizontal circle (counterclockwise seen from above) at constant
// speed. last_seen only updates while `visible` is true. To simulate losing
// the target, run:
//   ros2 param set /fake_target_node visible false

#include <chrono>
#include <cmath>
#include <memory>

#include "interceptor_msgs/msg/target_data.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using interceptor_msgs::msg::TargetData;

class FakeTargetNode : public rclcpp::Node
{
public:
  FakeTargetNode()
  : Node("fake_target_node")
  {
    center_x_ = declare_parameter("center_x", 20.0);
    center_y_ = declare_parameter("center_y", 20.0);
    radius_ = declare_parameter("radius", 15.0);
    altitude_ = declare_parameter("altitude", 10.0);
    speed_ = declare_parameter("speed", 3.0);
    // Read every tick (not once) so it can be flipped at runtime.
    declare_parameter("visible", true);

    start_time_ = now();
    publisher_ = create_publisher<TargetData>("/target_data", 10);
    timer_ = create_wall_timer(100ms, [this]() {tick();});  // 10 Hz
  }

private:
  void tick()
  {
    const rclcpp::Time stamp = now();
    const double angle = (speed_ / radius_) * (stamp - start_time_).seconds();

    // A real Tracker only updates last_seen on an actual camera detection.
    if (get_parameter("visible").as_bool()) {
      last_seen_ = stamp;
    }

    TargetData msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";  // same frame as /mavros/local_position/pose
    msg.position.x = center_x_ + radius_ * std::cos(angle);
    msg.position.y = center_y_ + radius_ * std::sin(angle);
    msg.position.z = altitude_;
    // Tangent to the circle, pointing along the direction of motion.
    msg.velocity.x = -speed_ * std::sin(angle);
    msg.velocity.y = speed_ * std::cos(angle);
    msg.velocity.z = 0.0;
    msg.last_seen = last_seen_;
    publisher_->publish(msg);
  }

  double center_x_;
  double center_y_;
  double radius_;
  double altitude_;
  double speed_;

  rclcpp::Time start_time_;
  rclcpp::Time last_seen_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<TargetData>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeTargetNode>());
  rclcpp::shutdown();
  return 0;
}

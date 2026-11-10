#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TrackerNode : public rclcpp::Node
{
public:
  TrackerNode()
  : Node("tracker_node")
  {
    publisher_ = create_publisher<std_msgs::msg::String>("test/tracker", 10);
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      std_msgs::msg::String message;
      message.data = "Hello World!";
      publisher_->publish(message);
      RCLCPP_INFO(get_logger(), "%s", message.data.c_str());
    });
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackerNode>());
  rclcpp::shutdown();
  return 0;
}

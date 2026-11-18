#include "interceptor_manager/manager_math.hpp"

#include <algorithm>
#include <cmath>

namespace interceptor_manager
{

double distance(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot(b.x - a.x, b.y - a.y, b.z - a.z);
}

double yaw_toward(const geometry_msgs::msg::Point & from, const geometry_msgs::msg::Point & to)
{
  return std::atan2(to.y - from.y, to.x - from.x);
}

std::vector<geometry_msgs::msg::Point> make_sweep_waypoints(
  double center_x, double center_y, double radius, double lane_spacing, double altitude)
{
  // Each lane covers half a spacing on either side, so this many lanes fit
  // inside the diameter.
  const int lane_count = std::max(1, static_cast<int>(std::floor(2.0 * radius / lane_spacing)));

  std::vector<geometry_msgs::msg::Point> waypoints;
  for (int i = 0; i < lane_count; ++i) {
    // Offset of this lane from the center, symmetric about 0.
    const double y_offset = (i - (lane_count - 1) / 2.0) * lane_spacing;
    // Half the lane's length: where the line y = y_offset meets the circle.
    const double half_length = std::sqrt(radius * radius - y_offset * y_offset);

    geometry_msgs::msg::Point start;
    start.x = center_x - half_length;
    start.y = center_y + y_offset;
    start.z = altitude;

    geometry_msgs::msg::Point end = start;
    end.x = center_x + half_length;

    if (i % 2 == 1) {
      std::swap(start, end);
    }
    waypoints.push_back(start);
    waypoints.push_back(end);
  }
  return waypoints;
}

geometry_msgs::msg::Point standoff_point(
  const geometry_msgs::msg::Point & target, const geometry_msgs::msg::Point & drone,
  double standoff_distance, double min_altitude)
{
  double dx = drone.x - target.x;
  double dy = drone.y - target.y;
  double horizontal_distance = std::hypot(dx, dy);
  if (horizontal_distance < 0.1) {
    dx = 1.0;
    dy = 0.0;
    horizontal_distance = 1.0;
  }

  geometry_msgs::msg::Point point;
  point.x = target.x + standoff_distance * dx / horizontal_distance;
  point.y = target.y + standoff_distance * dy / horizontal_distance;
  point.z = std::max(target.z, min_altitude);
  return point;
}

geometry_msgs::msg::Vector3 standoff_velocity(
  const geometry_msgs::msg::Vector3 & target_velocity, double target_altitude,
  double min_altitude)
{
  geometry_msgs::msg::Vector3 velocity = target_velocity;
  if (target_altitude < min_altitude) {
    velocity.z = 0.0;
  }
  return velocity;
}

}  // namespace interceptor_manager

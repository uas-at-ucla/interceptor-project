#include "interceptor_manager/manager_math.hpp"

#include <algorithm>
#include <cmath>

namespace interceptor_manager
{

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

}  // namespace interceptor_manager

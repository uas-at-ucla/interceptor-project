// Pure geometry helpers for the manager node. No ROS node code here, so these
// can be unit-tested on their own (see test/test_manager_math.cpp).

#pragma once

#include <vector>

#include "geometry_msgs/msg/point.hpp"

namespace interceptor_manager
{

// Snake ("lawnmower") sweep over a circular search area, at a fixed altitude.
//
// Lanes run along the x-axis, lane_spacing apart, centered on the circle.
// With 2 * radius an exact multiple of lane_spacing, the first and last lanes
// sit half a spacing inside the circle; otherwise the leftover margin is split
// evenly at both edges. If lane_spacing is wider than the circle, there is one
// lane through the center.
//
// Each lane gives two waypoints, where it crosses the circle. Direction
// alternates every lane: lane 0 flies toward +x, lane 1 toward -x, and so on.
std::vector<geometry_msgs::msg::Point> make_sweep_waypoints(
  double center_x, double center_y, double radius, double lane_spacing, double altitude);

}  // namespace interceptor_manager

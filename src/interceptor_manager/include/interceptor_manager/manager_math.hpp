// Pure geometry helpers for the manager node. No ROS node code here, so these
// can be unit-tested on their own (see test/test_manager_math.cpp).

#pragma once

#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace interceptor_manager
{

// Straight-line (3D) distance between two points.
double distance(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b);

// Yaw (radians, ENU: 0 = east, counterclockwise) that faces from `from`
// toward `to` in the horizontal plane.
double yaw_toward(const geometry_msgs::msg::Point & from, const geometry_msgs::msg::Point & to);

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

// Where to hold while following a target.
//
// Horizontally: standoff_distance from the target, on the side the drone is
// already on (along the horizontal line from the target toward the drone).
// If the drone is almost directly above or below the target (under 0.1 m
// apart horizontally), that line is undefined, so it uses the +x direction.
// Vertically: the target's altitude, but never below min_altitude.
geometry_msgs::msg::Point standoff_point(
  const geometry_msgs::msg::Point & target, const geometry_msgs::msg::Point & drone,
  double standoff_distance, double min_altitude);

// Velocity feedforward to use with standoff_point: the target's velocity, so
// the drone keeps pace instead of lagging behind. When the target is below
// min_altitude, the standoff point is pinned to the floor, so the vertical
// part is dropped; it would fight the fixed altitude.
geometry_msgs::msg::Vector3 standoff_velocity(
  const geometry_msgs::msg::Vector3 & target_velocity, double target_altitude,
  double min_altitude);

}  // namespace interceptor_manager

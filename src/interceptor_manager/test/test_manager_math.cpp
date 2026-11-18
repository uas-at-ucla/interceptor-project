#include <gtest/gtest.h>

#include <cmath>

#include "interceptor_manager/manager_math.hpp"

using interceptor_manager::distance;
using interceptor_manager::make_sweep_waypoints;
using interceptor_manager::standoff_point;
using interceptor_manager::standoff_velocity;
using interceptor_manager::yaw_toward;

geometry_msgs::msg::Point make_point(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

TEST(SweepWaypoints, AllInsideOrOnCircle)
{
  const double cx = 5.0, cy = -3.0, radius = 152.4;
  const auto waypoints = make_sweep_waypoints(cx, cy, radius, 30.0, 20.0);

  ASSERT_FALSE(waypoints.empty());
  for (const auto & p : waypoints) {
    EXPECT_LE(std::hypot(p.x - cx, p.y - cy), radius + 1e-9);
    EXPECT_DOUBLE_EQ(p.z, 20.0);
  }
}

TEST(SweepWaypoints, DirectionAlternates)
{
  const auto waypoints = make_sweep_waypoints(0.0, 0.0, 152.4, 30.0, 20.0);

  // Waypoints come in (start, end) pairs, one pair per lane.
  for (size_t lane = 0; lane < waypoints.size() / 2; ++lane) {
    const double dx = waypoints[2 * lane + 1].x - waypoints[2 * lane].x;
    if (lane % 2 == 0) {
      EXPECT_GT(dx, 0.0) << "lane " << lane << " should fly toward +x";
    } else {
      EXPECT_LT(dx, 0.0) << "lane " << lane << " should fly toward -x";
    }
  }
}

TEST(SweepWaypoints, LaneCountForSimpleCase)
{
  // Diameter 20, spacing 5: four lanes at y = -7.5, -2.5, 2.5, 7.5,
  // so the outer lanes sit exactly half a spacing inside the circle.
  const auto waypoints = make_sweep_waypoints(0.0, 0.0, 10.0, 5.0, 20.0);

  ASSERT_EQ(waypoints.size(), 8u);
  const double expected_y[] = {-7.5, -2.5, 2.5, 7.5};
  for (int lane = 0; lane < 4; ++lane) {
    EXPECT_DOUBLE_EQ(waypoints[2 * lane].y, expected_y[lane]);
    EXPECT_DOUBLE_EQ(waypoints[2 * lane + 1].y, expected_y[lane]);
  }
}

TEST(StandoffPoint, SameSideAtTargetAltitudeWithFloor)
{
  // Drone 3-4-5 away from the target: hold 10 m out along that line, at the
  // target's altitude.
  auto p = standoff_point(make_point(1.0, 2.0, 12.0), make_point(4.0, 6.0, 30.0), 10.0, 5.0);
  EXPECT_DOUBLE_EQ(p.x, 1.0 + 6.0);
  EXPECT_DOUBLE_EQ(p.y, 2.0 + 8.0);
  EXPECT_DOUBLE_EQ(p.z, 12.0);

  // Target below the floor: altitude is clamped to min_altitude.
  p = standoff_point(make_point(0.0, 0.0, 2.0), make_point(20.0, 0.0, 15.0), 10.0, 5.0);
  EXPECT_DOUBLE_EQ(p.x, 10.0);
  EXPECT_DOUBLE_EQ(p.z, 5.0);

  // Drone directly above the target: falls back to the +x direction.
  p = standoff_point(make_point(3.0, 3.0, 10.0), make_point(3.0, 3.0, 25.0), 10.0, 5.0);
  EXPECT_DOUBLE_EQ(p.x, 13.0);
  EXPECT_DOUBLE_EQ(p.y, 3.0);
}

TEST(StandoffVelocity, DropsVerticalOnlyBelowFloor)
{
  geometry_msgs::msg::Vector3 target_velocity;
  target_velocity.x = 2.0;
  target_velocity.z = 1.5;

  auto v = standoff_velocity(target_velocity, 10.0, 5.0);  // above the floor
  EXPECT_DOUBLE_EQ(v.x, 2.0);
  EXPECT_DOUBLE_EQ(v.z, 1.5);

  v = standoff_velocity(target_velocity, 2.0, 5.0);  // below the floor
  EXPECT_DOUBLE_EQ(v.x, 2.0);
  EXPECT_DOUBLE_EQ(v.z, 0.0);
}

TEST(SmallHelpers, DistanceAndYawToward)
{
  EXPECT_DOUBLE_EQ(distance(make_point(1.0, 2.0, 3.0), make_point(3.0, 5.0, 9.0)), 7.0);
  // ENU: east is 0, north is +pi/2; altitude is ignored.
  EXPECT_DOUBLE_EQ(yaw_toward(make_point(0.0, 0.0, 0.0), make_point(5.0, 0.0, 9.0)), 0.0);
  EXPECT_DOUBLE_EQ(yaw_toward(make_point(1.0, 1.0, 0.0), make_point(1.0, 4.0, 0.0)), M_PI / 2);
}

#include <gtest/gtest.h>

#include <cmath>

#include "interceptor_manager/manager_math.hpp"

using interceptor_manager::make_sweep_waypoints;

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

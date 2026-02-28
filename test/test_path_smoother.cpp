#include "trajectory_tracking/b_spline_smoother.hpp"
#include <gtest/gtest.h>

using namespace trajectory_tracking;

TEST(PathSmootherTest, HandlesLessThanThreeWaypoints) {
  std::vector<Point> waypoints = {{0.0, 0.0}, {1.0, 1.0}};
  EXPECT_THROW(
    BSplineSmoother::smoothPath(waypoints, 0.05),
    std::invalid_argument);
}

TEST(PathSmootherTest, GeneratesSmoothPath) {
  std::vector<Point> waypoints = {
    {0.0, 0.0}, {1.0, 0.5}, {2.0, -0.5}, {3.0, 0.0}};

  auto smoothed_path = BSplineSmoother::smoothPath(waypoints, 0.05);

  // Check that we got an output path with a reasonable number of points
  EXPECT_GT(smoothed_path.size(), waypoints.size());

  // Check that start and end points are approximate clamped to original
  // waypoints
  EXPECT_NEAR(smoothed_path.front().x, waypoints.front().x, 1e-2);
  EXPECT_NEAR(smoothed_path.front().y, waypoints.front().y, 1e-2);

  EXPECT_NEAR(smoothed_path.back().x, waypoints.back().x, 1e-2);
  EXPECT_NEAR(smoothed_path.back().y, waypoints.back().y, 1e-2);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include "trajectory_tracking/trajectory_generator.hpp"
#include <gtest/gtest.h>
#include <vector>

using namespace trajectory_tracking;

TEST(TrajectoryGeneratorTest, HandlesEmptyPath) {
  std::vector<Point> empty_path;
  EXPECT_THROW(
    TrajectoryGenerator::generateTrajectory(empty_path, 1.0, 1.0),
    std::invalid_argument);
}

TEST(TrajectoryGeneratorTest, GeneratesValidProfile) {
  std::vector<Point> path;
  // Straight line of 5 meters
  for (int i = 0; i <= 100; ++i) {
    path.push_back({i * 0.05, 0.0});
  }

  double max_v = 1.0;
  double max_a = 0.5;

  auto trajectory = TrajectoryGenerator::generateTrajectory(path, max_v, max_a);

  EXPECT_EQ(trajectory.size(), path.size());

  // Check velocity limits
  for (const auto & tp : trajectory) {
    EXPECT_LE(tp.linear_velocity, max_v + 1e-3);
    EXPECT_GE(tp.linear_velocity, 0.0 - 1e-3);
  }

  // Midpoint should be at max velocity or close if ramp distance is long
  EXPECT_NEAR(trajectory[50].linear_velocity, max_v, 1e-2);

  // End point velocity should be near zero
  EXPECT_LT(trajectory.back().linear_velocity, 0.1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

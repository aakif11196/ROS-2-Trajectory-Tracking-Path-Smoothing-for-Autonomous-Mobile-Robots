#include "trajectory_tracking/pure_pursuit.hpp"
#include <gtest/gtest.h>
#include <vector>

using namespace trajectory_tracking;

TEST(PurePursuit_controllerTest, HandlesEmptyTrajectory) {
  PurePursuit_controller controller(1.0);
  std::vector<TrajectoryPoint> empty_traj;
  RobotState state = {0.0, 0.0, 0.0};

  EXPECT_THROW(
    controller.computeCommand(state, empty_traj),
    std::invalid_argument);
}

TEST(PurePursuit_controllerTest, TracksStraightLine) {
  PurePursuit_controller controller(1.0);

  std::vector<TrajectoryPoint> trajectory;
  for (int i = 0; i < 50; ++i) {
    trajectory.push_back(
      {static_cast<double>(i) * 0.1, 0.0, i * 0.1, 1.0, 0.0});
  }

  // Robot is slightly offset in Y
  RobotState state = {0.0, 0.5, 0.0};

  auto cmd = controller.computeCommand(state, trajectory);

  // Should steer right (negative angular velocity) to get back on line
  EXPECT_LT(cmd.angular, 0.0);
  EXPECT_GT(cmd.linear, 0.0);
}

TEST(PurePursuit_controllerTest, ReachesEndOfTrajectory) {
  PurePursuit_controller controller(1.0);

  std::vector<TrajectoryPoint> trajectory = {{5.0, 0.0, 0.0, 0.0, 0.0}};

  // Robot is at the end
  RobotState state = {5.0, 0.0, 0.0};

  auto cmd = controller.computeCommand(state, trajectory);

  EXPECT_NEAR(cmd.linear, 0.0, 1e-3);
  EXPECT_NEAR(cmd.angular, 0.0, 1e-3);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#ifndef TRAJECTORY_TRACKING__PURE_PURSUIT_HPP_
#define TRAJECTORY_TRACKING__PURE_PURSUIT_HPP_

#include "trajectory_tracking/trajectory_generator.hpp"
#include <vector>

namespace trajectory_tracking {

/**
 * @brief Represents velocity commands to be sent to the differential drive
 * robot.
 */
struct VelocityCommand {
  double linear;
  double angular;
};

/**
 * @brief Represents the current basic state of the robot.
 */
struct RobotState {
  double x;
  double y;
  double theta;
};

/**
 * @brief Class for calculating control commands to track a given trajectory
 * using the Pure Pursuit algorithm.
 */
class PurePursuit_controller {
public:
  /**
   * @brief Constructs a new Pure Pursuit Controller
   *
   * @param lookahead_distance The lookahead distance for the algorithm
   */
  explicit PurePursuit_controller(double lookahead_distance);

  /**
   * @brief Calculates the velocity command to track the trajectory given the
   * current state.
   *
   * @param current_state The current (x, y, theta) state of the robot.
   * @param trajectory The target parameterizable trajectory.
   * @return VelocityCommand The required linear and angular velocities.
   * @throws std::invalid_argument if trajectory is empty.
   */
  VelocityCommand
  computeCommand(const RobotState &current_state,
                 const std::vector<TrajectoryPoint> &trajectory);

  /**
   * @brief Sets the lookahead distance dynamically.
   *
   * @param lookahead_distance The new lookahead distance for the algorithm
   */
  void setLookaheadDistance(double lookahead_distance);

private:
  double lookahead_distance_;
};

} // namespace trajectory_tracking

#endif // TRAJECTORY_TRACKING__PURE_PURSUIT_HPP_

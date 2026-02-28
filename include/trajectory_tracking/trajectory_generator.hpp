#ifndef TRAJECTORY_TRACKING__TRAJECTORY_GENERATOR_HPP_
#define TRAJECTORY_TRACKING__TRAJECTORY_GENERATOR_HPP_

#include "trajectory_tracking/b_spline_smoother.hpp"
#include <vector>

namespace trajectory_tracking {

/**
 * @brief Represents a time-parameterized point on the trajectory.
 */
struct TrajectoryPoint {
  double x;
  double y;
  double time;
  double linear_velocity;
  double angular_velocity;
};

/**
 * @brief Class for generating a time-parameterized trajectory with a
 * trapezoidal velocity profile.
 */
class TrajectoryGenerator {
public:
  /**
   * @brief Generates a trajectory from a smoothed path.
   *
   * @param path The dense sequence of (x,y) points from the path smoother.
   * @param max_v Maximum allowable linear velocity.
   * @param max_a Maximum allowable linear acceleration.
   * @return std::vector<TrajectoryPoint> The time-parameterized trajectory.
   * @throws std::invalid_argument if path is empty.
   */
  static std::vector<TrajectoryPoint>
  generateTrajectory(const std::vector<Point> &path, double max_v,
                     double max_a);

  /**
   * @brief Updates only a localized segment of an existing trajectory.
   *
   * @param path The updated full smoothed path array.
   * @param start_idx The array index of the first modified point.
   * @param end_idx The array index of the last modified point.
   * @param trajectory The existing trajectory array to update in place.
   */
  static void updateLocalTrajectory(const std::vector<Point> &path,
                                    int start_idx, int end_idx,
                                    std::vector<TrajectoryPoint> &trajectory);
};

} // namespace trajectory_tracking

#endif // TRAJECTORY_TRACKING__TRAJECTORY_GENERATOR_HPP_

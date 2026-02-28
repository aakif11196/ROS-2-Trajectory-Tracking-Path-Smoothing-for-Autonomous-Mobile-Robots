#include "trajectory_tracking/pure_pursuit.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace trajectory_tracking {

PurePursuit_controller::PurePursuit_controller(
    double lookahead_distance)
    : lookahead_distance_(lookahead_distance) {}

void PurePursuit_controller::setLookaheadDistance(
    double lookahead_distance) {
  lookahead_distance_ = lookahead_distance;
}

VelocityCommand PurePursuit_controller::computeCommand(
    const RobotState &current_state,
    const std::vector<TrajectoryPoint> &trajectory) {
  if (trajectory.empty()) {
    throw std::invalid_argument("Cannot track an empty trajectory.");
  }

  // Find the closest point and the target point on the trajectory
  size_t closest_idx = 0;
  double min_dist_sq = std::numeric_limits<double>::max();

  for (size_t i = 0; i < trajectory.size(); ++i) {
    double dx = trajectory[i].x - current_state.x;
    double dy = trajectory[i].y - current_state.y;
    double dist_sq = dx * dx + dy * dy;

    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_idx = i;
    }
  }

  // Start looking for the lookahead point from the closest index forwards
  size_t target_idx = closest_idx;
  bool found_lookahead = false;

  for (size_t i = closest_idx; i < trajectory.size(); ++i) {
    double dx = trajectory[i].x - current_state.x;
    double dy = trajectory[i].y - current_state.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist >= lookahead_distance_) {
      target_idx = i;
      found_lookahead = true;
      break;
    }
  }

  // If no point is further than lookahead, just target the last point
  if (!found_lookahead) {
    target_idx = trajectory.size() - 1;
  }

  // Retrieve target details
  double tx = trajectory[target_idx].x;
  double ty = trajectory[target_idx].y;
  double target_v = trajectory[target_idx].linear_velocity;

  // Convert target to robot's local frame
  double dx = tx - current_state.x;
  double dy = ty - current_state.y;

  // Rotate into local frame:
  // x_local = x * cos(theta) + y * sin(theta)
  // y_local = -x * sin(theta) + y * cos(theta)
  double local_tx =
      std::cos(-current_state.theta) * dx - std::sin(-current_state.theta) * dy;
  double local_ty =
      std::sin(-current_state.theta) * dx + std::cos(-current_state.theta) * dy;

  double dist_to_target_sq = local_tx * local_tx + local_ty * local_ty;

  VelocityCommand cmd;

  if (dist_to_target_sq < 0.0001) {
    // We are virtually at the target point
    cmd.linear = 0.0;
    cmd.angular = 0.0;
    return cmd;
  }

  double curvature = 2.0 * local_ty / dist_to_target_sq;

  // Command linear velocity from the profile
  cmd.linear = target_v;

  // Command angular velocity: w = v * curvature
  cmd.angular = cmd.linear * curvature;

  // Optional: Stop logic if we are very close to the END of the trajectory
  if (target_idx == trajectory.size() - 1) {
    double dist_to_end =
        std::sqrt(std::pow(trajectory.back().x - current_state.x, 2) +
                  std::pow(trajectory.back().y - current_state.y, 2));

    if (dist_to_end < 0.1) {
      // Reached goal area, slow down and stop
      cmd.linear = 0.0;
      cmd.angular = 0.0;
    }
  }

  return cmd;
}

} // namespace trajectory_tracking

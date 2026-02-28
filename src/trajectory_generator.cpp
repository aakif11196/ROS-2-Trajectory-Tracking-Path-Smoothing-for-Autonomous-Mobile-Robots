#include "trajectory_tracking/trajectory_generator.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace trajectory_tracking {

std::vector<TrajectoryPoint>
TrajectoryGenerator::generateTrajectory(const std::vector<Point> &path,
                                        double max_v, double max_a) {
  if (path.empty()) {
    throw std::invalid_argument(
        "Cannot generate trajectory for an empty path.");
  }

  std::vector<TrajectoryPoint> trajectory;

  if (path.size() == 1) {
    trajectory.push_back({path[0].x, path[0].y, 0.0, 0.0, 0.0});
    return trajectory;
  }

  // Step 1: Calculate cumulative distances along the path
  std::vector<double> distances;
  distances.push_back(0.0);
  double total_distance = 0.0;

  // Also calculate headings to compute angular velocities later
  std::vector<double> headings;

  for (size_t i = 1; i < path.size(); ++i) {
    double dx = path[i].x - path[i - 1].x;
    double dy = path[i].y - path[i - 1].y;
    double dist = std::sqrt(dx * dx + dy * dy);
    total_distance += dist;
    distances.push_back(total_distance);

    headings.push_back(std::atan2(dy, dx));
  }
  // Final heading repeated for the last point
  headings.push_back(headings.back());

  // Step 2: Calculate trapezoidal velocity profile parameters
  // Check if max_v can be reached
  double ramp_distance = 0.5 * max_v * max_v / max_a;
  double actual_max_v = max_v;

  // If the path is too short to reach max_v and decelerate
  if (2.0 * ramp_distance > total_distance) {
    actual_max_v = std::sqrt(max_a * total_distance);
    ramp_distance = total_distance / 2.0;
  }

  double cruise_distance = total_distance - 2.0 * ramp_distance;

  // Step 3: Assign time and velocity to each point based on distance
  double current_time = 0.0;
  double prev_distance = 0.0;

  for (size_t i = 0; i < path.size(); ++i) {
    double s = distances[i];
    double ds = s - prev_distance;

    double v = 0.0;
    double dt = 0.0;

    // Determine phase of the trapezoid
    if (s <= ramp_distance) {
      // Acceleration phase
      v = std::sqrt(2.0 * max_a * s);
      if (v > 0.001) {
        dt = ds / v; // Approximating time step dt = ds / v
      } else if (i == 0) {
        dt = 0; // First point
      } else {
        dt = std::sqrt(2.0 * ds / max_a); // For very small v
      }
    } else if (s <= ramp_distance + cruise_distance) {
      // Cruise phase
      v = actual_max_v;
      dt = ds / v;
    } else {
      // Deceleration phase
      double decel_s = total_distance - s;
      if (decel_s < 0.0) {
        decel_s = 0.0;
      }
      v = std::sqrt(2.0 * max_a * decel_s);
      if (v > 0.001) {
        dt = ds / v;
      } else {
        // If velocity becomes theoretically 0, we can fall back to max_a
        // derived
        double prev_v = std::sqrt(2.0 * max_a * (decel_s + ds));
        double avg_v = (prev_v + v) / 2.0;
        if (avg_v > 0.001) {
          dt = ds / avg_v;
        } else {
          dt = std::sqrt(2.0 * ds / max_a);
        }
      }
    }

    current_time += dt;
    prev_distance = s;

    // Angular velocity: w = d(theta)/dt
    double w = 0.0;
    if (i > 0 && dt > 0.001) {
      double dtheta = headings[i] - headings[i - 1];
      // Normalize angle difference between -pi and pi
      while (dtheta > M_PI) {
        dtheta -= 2.0 * M_PI;
      }
      while (dtheta < -M_PI) {
        dtheta += 2.0 * M_PI;
      }
      w = dtheta / dt;
    }

    trajectory.push_back(
        {path[i].x, path[i].y, current_time, std::max(v, 0.0), w});
  }

  return trajectory;
}

void TrajectoryGenerator::updateLocalTrajectory(
    const std::vector<Point> &path, int start_idx, int end_idx,
    std::vector<TrajectoryPoint> &trajectory) {
  if (start_idx < 0 || end_idx >= static_cast<int>(path.size()) ||
      start_idx > end_idx) {
    return;
  }

  // We only update geometry and local heading for the affected segment.
  // The global time parameterization and velocity limits remain unchanged
  // as per the requirement: "Update the TrajectoryGenerator only for that
  // specific segment".
  for (int i = start_idx; i <= end_idx; ++i) {
    trajectory[i].x = path[i].x;
    trajectory[i].y = path[i].y;

    // Recompute heading-derived angular velocity if possible
    if (i > 0 && i < static_cast<int>(trajectory.size())) {
      double dx = path[i].x - path[i - 1].x;
      double dy = path[i].y - path[i - 1].y;
      double curr_heading = std::atan2(dy, dx);

      double prev_heading = curr_heading;
      if (i > 1) {
        double pdx = path[i - 1].x - path[i - 2].x;
        double pdy = path[i - 1].y - path[i - 2].y;
        prev_heading = std::atan2(pdy, pdx);
      }

      double dt = trajectory[i].time - trajectory[i - 1].time;
      if (dt > 0.001) {
        double dtheta = curr_heading - prev_heading;
        while (dtheta > M_PI) {
          dtheta -= 2.0 * M_PI;
        }
        while (dtheta < -M_PI) {
          dtheta += 2.0 * M_PI;
        }
        trajectory[i].angular_velocity = dtheta / dt;
      }
    }
  }
}

} // namespace trajectory_tracking

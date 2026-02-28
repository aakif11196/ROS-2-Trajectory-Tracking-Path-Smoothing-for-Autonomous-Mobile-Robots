#include "trajectory_tracking/b_spline_smoother.hpp"
#include <algorithm>
#include <cmath>

namespace trajectory_tracking {

Point BSplineSmoother::evaluateCubicBSpline(double t, const Point &p0,
                                            const Point &p1, const Point &p2,
                                            const Point &p3) {
  // Basic uniform cubic B-spline matrix formulation
  double t2 = t * t;
  double t3 = t2 * t;

  double b0 = (-t3 + 3.0 * t2 - 3.0 * t + 1.0) / 6.0;
  double b1 = (3.0 * t3 - 6.0 * t2 + 4.0) / 6.0;
  double b2 = (-3.0 * t3 + 3.0 * t2 + 3.0 * t + 1.0) / 6.0;
  double b3 = t3 / 6.0;

  Point result;
  result.x = b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x;
  result.y = b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y;
  return result;
}

std::vector<Point>
BSplineSmoother::smoothPath(const std::vector<Point> &waypoints,
                            double resolution, std::vector<int> *segment_steps,
                            std::vector<int> *segment_start_idx) {
  if (waypoints.size() < 3) {
    throw std::invalid_argument(
        "B-Spline smoothing requires at least 3 waypoints.");
  }

  // To ensure the spline anchors to the start and end waypoints, we clamp the
  // curve by repeating the first and last waypoints 2 extra times (since we
  // need 3 duplicated ends to clamp cubic spline).
  std::vector<Point> control_points;
  control_points.push_back(waypoints.front());
  control_points.push_back(waypoints.front());
  for (const auto &wp : waypoints) {
    control_points.push_back(wp);
  }
  control_points.push_back(waypoints.back());
  control_points.push_back(waypoints.back());

  std::vector<Point> smoothed_path;

  // Number of segments is control_points.size() - 3
  size_t num_segments = control_points.size() - 3;

  if (segment_steps) {
    segment_steps->clear();
  }
  if (segment_start_idx) {
    segment_start_idx->clear();
  }

  for (size_t i = 0; i < num_segments; ++i) {
    if (segment_start_idx) {
      segment_start_idx->push_back(smoothed_path.size());
    }
    // Approximate length of the control polygon segment to determine number of
    // samples
    double dx = control_points[i + 2].x - control_points[i + 1].x;
    double dy = control_points[i + 2].y - control_points[i + 1].y;
    double seg_length = std::sqrt(dx * dx + dy * dy);

    // Estimate number of steps based on resolution. At least 5 steps to
    // maintain C2 smoothness.
    int steps =
        std::max(static_cast<int>(std::ceil(seg_length / resolution)), 5);

    if (segment_steps) {
      segment_steps->push_back(steps);
    }

    // Only include t=1.0 on the very last segment to avoid duplicates
    int iter_steps = (i == num_segments - 1) ? steps : steps - 1;

    for (int j = 0; j <= iter_steps; ++j) {
      double t = static_cast<double>(j) / steps;
      Point p =
          evaluateCubicBSpline(t, control_points[i], control_points[i + 1],
                               control_points[i + 2], control_points[i + 3]);
      smoothed_path.push_back(p);
    }
  }

  return smoothed_path;
}

void BSplineSmoother::updateLocalControlPoint(
    const std::vector<Point> &waypoints, int modified_wp_idx,
    const std::vector<int> &segment_steps,
    const std::vector<int> &segment_start_idx,
    std::vector<Point> &smoothed_path) {
  std::vector<Point> control_points;
  control_points.push_back(waypoints.front());
  control_points.push_back(waypoints.front());
  for (const auto &wp : waypoints) {
    control_points.push_back(wp);
  }
  control_points.push_back(waypoints.back());
  control_points.push_back(waypoints.back());

  // Map waypoint index to control point index
  int cp_idx = modified_wp_idx + 2;

  // A cubic uniform B-Spline segment i uses control points i, i+1, i+2, i+3.
  // Modifying control point `cp_idx` affects segments where:
  // i <= cp_idx <= i + 3
  // => cp_idx - 3 <= i <= cp_idx
  int start_seg = std::max(0, cp_idx - 3);
  int end_seg = std::min(static_cast<int>(segment_steps.size()) - 1, cp_idx);

  for (int i = start_seg; i <= end_seg; ++i) {
    int steps = segment_steps[i];
    int start_idx = segment_start_idx[i];

    // Use the existing number of steps to perfectly match the pre-allocated
    // smoothed_path segments
    int iter_steps =
        (i == static_cast<int>(segment_steps.size()) - 1) ? steps : steps - 1;

    for (int j = 0; j <= iter_steps; ++j) {
      double t = static_cast<double>(j) / steps;
      Point p =
          evaluateCubicBSpline(t, control_points[i], control_points[i + 1],
                               control_points[i + 2], control_points[i + 3]);

      // Update in O(1) time
      smoothed_path[start_idx + j] = p;
    }
  }
}

} // namespace trajectory_tracking

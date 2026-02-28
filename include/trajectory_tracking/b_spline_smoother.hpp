#ifndef TRAJECTORY_TRACKING__B_SPLINE_SMOOTHER_HPP_
#define TRAJECTORY_TRACKING__B_SPLINE_SMOOTHER_HPP_

#include <stdexcept>
#include <vector>

namespace trajectory_tracking {

/**
 * @brief Represents a 2D Cartesian point.
 */
struct Point {
  double x;
  double y;
};

/**
 * @brief Class for generating a smoothed path using Uniform Cubic B-Splines.
 *
 * This mathematically rigorous implementation guarantees piecewise C^2
 * continuity.
 */
class BSplineSmoother {
public:
  /**
   * @brief Smooths a discrete set of waypoints into a continuous path.
   *
   * @param waypoints The input list of (x, y) waypoints.
   * @param resolution The distance delta for sampling points along the spline.
   * @return std::vector<Point> The smoothed, dense path.
   * @throws std::invalid_argument if waypoints.size() < 3.
   */
  static std::vector<Point>
  smoothPath(const std::vector<Point> &waypoints, double resolution = 0.05,
             std::vector<int> *segment_steps = nullptr,
             std::vector<int> *segment_start_idx = nullptr);

  /**
   * @brief Modifies a local segment of the smoothed path directly in O(1) time
   * without recalculating the global path.
   *
   * @param waypoints The global waypoints (one has been locally shifted).
   * @param modified_wp_idx The index of the waypoint that was changed.
   * @param segment_steps The number of steps for each segment (from
   * smoothPath).
   * @param segment_start_idx The start index in smoothed_path for each segment.
   * @param smoothed_path The global dense path to update in place.
   */
  static void updateLocalControlPoint(const std::vector<Point> &waypoints,
                                      int modified_wp_idx,
                                      const std::vector<int> &segment_steps,
                                      const std::vector<int> &segment_start_idx,
                                      std::vector<Point> &smoothed_path);

private:
  /**
   * @brief Evaluates a Uniform Cubic B-Spline segment at parameter t in [0, 1].
   */
  static Point evaluateCubicBSpline(double t, const Point &p0, const Point &p1,
                                    const Point &p2, const Point &p3);
};

} // namespace trajectory_tracking

#endif // TRAJECTORY_TRACKING__B_SPLINE_SMOOTHER_HPP_

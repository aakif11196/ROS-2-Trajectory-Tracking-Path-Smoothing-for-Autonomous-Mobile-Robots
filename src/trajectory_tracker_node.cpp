/**
 * @file trajectory_tracker_node.cpp
 * @brief ROS 2 node for evaluating and executing trajectory tracking using
 * B-Spline interpolation and Pure Pursuit control, with dynamic obstacle
 * avoidance.
 */

#include <atomic>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

#include "trajectory_tracking/b_spline_smoother.hpp"
#include "trajectory_tracking/pure_pursuit.hpp"
#include "trajectory_tracking/trajectory_generator.hpp"

using namespace std::chrono_literals;
using namespace trajectory_tracking;

/**
 * @class TrajectoryTrackerNode
 * @brief Node responsible for orchestrating trajectory generation, obstacle
 * avoidance, and motion control on a differential drive robot.
 */
class TrajectoryTrackerNode : public rclcpp::Node {
public:
  /**
   * @brief Initializes the Trajectory Tracker Node, declares parameters,
   *        computes the initial trajectory, and starts the control loop.
   */
  TrajectoryTrackerNode() : Node("trajectory_tracker_node") {
    declareParameters();
    loadParameters();
    initializeTrajectory();
    setupROSInterfaces();

    RCLCPP_INFO(this->get_logger(),
                "Trajectory Tracker Node Initialized. Tracking %zu points.",
                trajectory_.size());
  }

private:

  // INITIALIZATION AND SETUP

  /**
   * @brief Declares all ROS 2 parameters used by this node.
   */
  void declareParameters() {
    this->declare_parameter("lookahead_distance", 0.5);
    this->declare_parameter("max_v", 0.3);
    this->declare_parameter("max_a", 0.1);
    this->declare_parameter("resolution", 0.05);
    this->declare_parameter("control_rate", 20.0);
    this->declare_parameter("emergency_stop_distance", 0.5);
    this->declare_parameter("dodge_offset", 0.9);
    this->declare_parameter("dodging_commitment_distance", 2.0);
    this->declare_parameter("waypoints_x",
                            std::vector<double>{0.0, 1.0, 2.0, 3.0});
    this->declare_parameter("waypoints_y",
                            std::vector<double>{0.0, 0.5, -0.5, 0.0});
  }

  /**
   * @brief Retrieves and stores parameter values into class members.
   */
  void loadParameters() {
    double lookahead = this->get_parameter("lookahead_distance").as_double();
    default_lookahead_ = lookahead;

    max_v_ = this->get_parameter("max_v").as_double();
    max_a_ = this->get_parameter("max_a").as_double();
    resolution_ = this->get_parameter("resolution").as_double();
    control_rate_ = this->get_parameter("control_rate").as_double();

    emergency_stop_distance_ =
        this->get_parameter("emergency_stop_distance").as_double();
    dodge_offset_ = this->get_parameter("dodge_offset").as_double();
    dodging_commitment_distance_ =
        this->get_parameter("dodging_commitment_distance").as_double();

    auto wp_x = this->get_parameter("waypoints_x").as_double_array();
    auto wp_y = this->get_parameter("waypoints_y").as_double_array();

    if (wp_x.size() != wp_y.size() || wp_x.empty()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Waypoints X and Y arrays must be the same size and non-empty.");
      return;
    }

    for (size_t i = 0; i < wp_x.size(); ++i) {
      waypoints_.push_back({wp_x[i], wp_y[i]});
    }

    // Automatically pad the start and end waypoints to achieve B-Spline
    // clamping
    if (!waypoints_.empty()) {
      Point start_point = waypoints_.front();
      Point end_point = waypoints_.back();
      waypoints_.insert(waypoints_.begin(), 3, start_point);
      waypoints_.insert(waypoints_.end(), 3, end_point);
    }

    global_waypoints_ = waypoints_;
  }

  /**
   * @brief Sets up publishers, subscribers, the control timer, and the tracking
   * controller.
   */
  void setupROSInterfaces() {
    controller_ =
        std::make_unique<PurePursuit_controller>(default_lookahead_);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&TrajectoryTrackerNode::odomCallback, this,
                  std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&TrajectoryTrackerNode::scanCallback, this,
                  std::placeholders::_1));

    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    path_pub_ =
        this->create_publisher<nav_msgs::msg::Path>("/smoothed_path", 10);
    raw_path_pub_ =
        this->create_publisher<nav_msgs::msg::Path>("/raw_waypoints", 10);

    int period_ms = static_cast<int>(1000.0 / control_rate_);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&TrajectoryTrackerNode::controlLoop, this));

    publishPath(smoothed_path_);
    publishRawWaypoints(waypoints_);
  }

  // PATH GENERATION (B-SPLINE)

  /**
   * @brief Generates the smoothed B-Spline path and calculates the kinematic
   * trajectory profile.
   */
  void initializeTrajectory() {
    if (waypoints_.size() < 3) {
      RCLCPP_WARN(this->get_logger(),
                  "Less than 3 waypoints provided. Duplicating to satisfy "
                  "B-Spline constraints.");
      while (waypoints_.size() < 3) {
        waypoints_.push_back(waypoints_.back());
      }
      global_waypoints_ = waypoints_;
    }

    smoothed_path_ = BSplineSmoother::smoothPath(
        waypoints_, resolution_, &segment_steps_, &segment_start_idx_);

    trajectory_ =
        TrajectoryGenerator::generateTrajectory(smoothed_path_, max_v_, max_a_);
  }

  // SAFETY MONITOR (OBSTACLE AVOIDANCE)

  /**
   * @brief Processes LiDAR scan data to detect obstacles forward of the robot
   *        and injects dynamic avoidance maneuvers into the trajectory.
   *
   * @param msg LaserScan message.
   */
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(scan_mutex_);

    // Evaluate Forward Cone Area
    bool current_obstacle = false;
    double min_angle = -0.5; // ~28 degrees left
    double max_angle = 0.5;  // ~28 degrees right
    double closest_obs_dist = std::numeric_limits<double>::max();
    double closest_obs_angle = 0.0;
    double check_dist = std::max(emergency_stop_distance_ * 2.5, 0.8);

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      double angle = msg->angle_min + i * msg->angle_increment;
      while (angle > M_PI)
        angle -= 2.0 * M_PI;
      while (angle < -M_PI)
        angle += 2.0 * M_PI;

      if (angle >= min_angle && angle <= max_angle) {
        if (msg->ranges[i] < check_dist && msg->ranges[i] > msg->range_min) {
          current_obstacle = true;
          if (msg->ranges[i] < closest_obs_dist) {
            closest_obs_dist = msg->ranges[i];
            closest_obs_angle = angle;
          }
        }
      }
    }

    // Commitment Distance Logic: Keep dodging until minimum clearance distance
    // is met
    if (is_dodging_) {
      double dist_since_dodge =
          std::sqrt(std::pow(current_state_.x - dodge_start_position_.x, 2) +
                    std::pow(current_state_.y - dodge_start_position_.y, 2));
      if (dist_since_dodge < dodging_commitment_distance_) {
        current_obstacle = true; // Maintain dodge state explicitly
      }
    }

    if (current_obstacle) {
      initiateDodgeManeuver(closest_obs_angle, closest_obs_dist);
    } else {
      clearDodgeManeuver();
    }
  }

  /**
   * @brief Initiates a dodge maneuver if not already dodging, dynamically
   * injecting laterally shifted control points to recompute the trajectory
   * round the obstacle.
   */
  void initiateDodgeManeuver(double closest_obs_angle,
                             double closest_obs_dist) {
    if (is_dodging_)
      return;

    std::lock_guard<std::mutex> odom_lock(odom_mutex_);

    // Find the nearest control waypoint physically ahead of the robot
    int best_wp_idx = -1;
    double min_dist = std::numeric_limits<double>::max();

    for (size_t wp_i = 1; wp_i < global_waypoints_.size() - 1; ++wp_i) {
      double dx = global_waypoints_[wp_i].x - current_state_.x;
      double dy = global_waypoints_[wp_i].y - current_state_.y;

      // Rotate into local frame to check if strictly ahead
      double local_x = std::cos(-current_state_.theta) * dx -
                       std::sin(-current_state_.theta) * dy;

      if (local_x > 0.1 && local_x < 4.0) {
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist < min_dist) {
          min_dist = dist;
          best_wp_idx = wp_i;
        }
      }
    }

    if (best_wp_idx != -1) {
      is_dodging_ = true;
      dodge_start_position_ = current_state_;

      // Dynamically tighten lookahead logic during the dodge
      controller_->setLookaheadDistance(std::min(default_lookahead_, 0.3));

      // Calculate transversal shift using a local duplicate of waypoints
      double shift_dir = (closest_obs_angle > 0.0) ? -M_PI / 2.0 : M_PI / 2.0;
      double shift_angle = current_state_.theta + shift_dir;

      std::vector<Point> local_waypoints = global_waypoints_;
      Point new_wp = local_waypoints[best_wp_idx];
      new_wp.x += dodge_offset_ * std::cos(shift_angle);
      new_wp.y += dodge_offset_ * std::sin(shift_angle);

      // Inject point with multiplicity 2
      local_waypoints[best_wp_idx] = new_wp;
      local_waypoints.insert(local_waypoints.begin() + best_wp_idx, new_wp);

      waypoints_ = local_waypoints;

      recomputeTrajectory();

      RCLCPP_INFO(this->get_logger(),
                  "Obstacle %.2fm ahead! Shifted waypoint %d transversally by "
                  "%.2fm and recomputed trajectory.",
                  closest_obs_dist, best_wp_idx, dodge_offset_);
    }
  }

  /**
   * @brief Clears dodge state and restores the global pristine trajectory path.
   */
  void clearDodgeManeuver() {
    if (!is_dodging_)
      return;

    is_dodging_ = false;
    std::lock_guard<std::mutex> odom_lock(odom_mutex_);
    waypoints_ = global_waypoints_;

    // Restore nominal tracking performance
    controller_->setLookaheadDistance(default_lookahead_);

    recomputeTrajectory();

    RCLCPP_INFO(this->get_logger(),
                "Obstacle cleared. Resuming strictly on global path.");
  }

  /**
   * @brief Helper to quickly re-invoke BSpline generation and Kinematic
   * Profiling
   */
  void recomputeTrajectory() {
    smoothed_path_ = BSplineSmoother::smoothPath(
        waypoints_, resolution_, &segment_steps_, &segment_start_idx_);
    trajectory_ =
        TrajectoryGenerator::generateTrajectory(smoothed_path_, max_v_, max_a_);
    publishPath(smoothed_path_);
  }

  
  // TRACKING CONTROLLER

  /**
   * @brief Processes robot odometry and updates the current state tracking.
   *
   * @param msg Odometry message.
   */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    current_state_.x = msg->pose.pose.position.x;
    current_state_.y = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_state_.theta = yaw;

    last_odom_time_ = this->now();
    odom_received_ = true;
  }

  /**
   * @brief Main continuous control loop evaluating the Pure Pursuit commands.
   */
  void controlLoop() {
    if (trajectory_.empty())
      return;

    std::lock_guard<std::mutex> lock(odom_mutex_);
    geometry_msgs::msg::Twist cmd_msg;

    // Safety timeout check
    if (!odom_received_ || (this->now() - last_odom_time_).seconds() > 0.5) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Odometry timeout/missing. Stopping robot.");
      cmd_msg.linear.x = 0.0;
      cmd_msg.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd_msg);
      return;
    }

    // Execute Pure Pursuit Control Output
    try {
      VelocityCommand cmd =
          controller_->computeCommand(current_state_, trajectory_);
      cmd_msg.linear.x = cmd.linear;
      cmd_msg.angular.z = cmd.angular;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Controller Error: %s", e.what());
      cmd_msg.linear.x = 0.0;
      cmd_msg.angular.z = 0.0;
    }

    cmd_vel_pub_->publish(cmd_msg);
  }

  // UTILITIES AND VISUALIZATION

  /**
   * @brief Publishes the smoothed trajectory to RViz.
   */
  void publishPath(const std::vector<Point> &path_points) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "odom";

    for (const auto &p : path_points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = p.x;
      pose.pose.position.y = p.y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
    }
    path_pub_->publish(path_msg);
  }

  /**
   * @brief Publishes the raw discrete waypoints to RViz.
   */
  void publishRawWaypoints(const std::vector<Point> &raw_waypoints) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "odom";

    for (const auto &p : raw_waypoints) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = p.x;
      pose.pose.position.y = p.y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
    }
    raw_path_pub_->publish(path_msg);
  }

  // MEMBER VARIABLES

  // Node Configuration
  double max_v_;
  double max_a_;
  double resolution_;
  double control_rate_;

  // ROS Interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Trajectory System
  std::vector<Point> global_waypoints_;
  std::vector<Point> waypoints_;
  std::vector<int> segment_steps_;
  std::vector<int> segment_start_idx_;
  std::vector<Point> smoothed_path_;
  std::vector<TrajectoryPoint> trajectory_;
  std::unique_ptr<PurePursuit_controller> controller_;

  // State Monitoring
  RobotState current_state_ = {0.0, 0.0, 0.0};
  std::mutex odom_mutex_;
  rclcpp::Time last_odom_time_;
  bool odom_received_ = false;

  // Obstacle Avoidance
  std::mutex scan_mutex_;
  bool is_dodging_ = false;
  RobotState dodge_start_position_ = {0.0, 0.0, 0.0};
  double emergency_stop_distance_;
  double dodge_offset_;
  double dodging_commitment_distance_;
  double default_lookahead_ = 0.5;
};

/**
 * @brief Node entry point.
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryTrackerNode>());
  rclcpp::shutdown();
  return 0;
}

// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef BUFF_TRACKER__TRACKER_HPP_
#define BUFF_TRACKER__TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>

// tf2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

// STD
#include <angles/angles.h>

#include <buff_interfaces/msg/blade.hpp>
#include <buff_interfaces/msg/blade_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <string>

#include "buff_tracker/extended_kalman_filter.hpp"
#include "buff_tracker/gauss_newton_solver.hpp"

#define PI 3.1415926
#define BLADE_R_OFFSET 700.0
// #define OMEGA 1.0 / 3 * PI
#define OMEGA 0.0
namespace rm_buff
{
class Tracker
{
public:
  Tracker(double max_match_theta, double max_match_center_xoy);

  void init(const buff_interfaces::msg::BladeArray::SharedPtr & blades_msg);

  void update(const buff_interfaces::msg::BladeArray::SharedPtr & blades_msg);

  void solve(const rclcpp::Time & time);

  ExtendedKalmanFilter ekf;

  GaussNewtonSolver gns;

  ExtendedKalmanFilter ekf_gns;

  double a_start, w_start, c_start;
  double min_first_solve_time;

  int tracking_threshold;
  int lost_threshold;
  // bullet flight duration not enough ~ ms
  int timeout_threshold;

  double blade_z_ground;
  double robot_z_ground;
  double distance;
  double max_distance_diff;
  double center_xoy_diff;

  enum State {
    LOST,
    DETECTING,
    TRACKING,
    TIMEOUT,
    TEMP_LOST,
  } tracker_state;

  enum SolverStatus {
    WAITING,
    NOT_ENOUGH_OBS,
    VALID,
    INVALID,
  } solver_status;

  // blade info
  // first TRACKING blade set to blade_id = 0
  // diff with blade_0: phi
  // blade_id: 0 <== phi = 0/5 * pi
  //           1 <== phi = 2/5 * pi
  //           2 <== phi = 4/5 * pi
  //           3 <== phi = 6/5 * pi
  //           4 <== phi = 8/5 * pi
  int blade_id;

  struct blade_transform
  {
    geometry_msgs::msg::Point blade_position;
    geometry_msgs::msg::Point center_position;
    double theta;
  };

  blade_transform tracked_blade;

  Eigen::VectorXd measurement;

  Eigen::VectorXd target_state;

  Eigen::VectorXd spd_state;

  rclcpp::Time obs_start_time;

  void getTrackerPosition(blade_transform & blade);

private:
  void initEKF(const blade_transform & blade);

  blade_transform bladeTransform(const buff_interfaces::msg::Blade & blade);

  bool handleBladeJump(double theta_diff);

  geometry_msgs::msg::Point rotateBlade(const blade_transform blade, int idx);

  void calculateMeasurementFromPrediction(blade_transform & blade, const Eigen::VectorXd & state);

  double max_match_theta_;
  double max_match_center_xoy_;

  int detect_count_;
  int lost_count_;
  int timeout_count_;  // ms

  double last_theta_;
};
}  // namespace rm_buff

#endif  // BUFF_TRACKER__TRACKER_HPP_
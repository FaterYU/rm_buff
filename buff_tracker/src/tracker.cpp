// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#include "buff_tracker/tracker.hpp"

namespace rm_buff
{

Tracker::Tracker(double max_match_theta, double max_match_center_xoy)
: tracker_state(LOST),
  blade_id(0),
  measurement(Eigen::VectorXd::Zero(4)),
  target_state(Eigen::VectorXd::Zero(9)),
  spd_state(Eigen::VectorXd::Zero(3)),
  max_match_theta_(max_match_theta),
  max_match_center_xoy_(max_match_center_xoy)
{
}

void Tracker::init(const buff_interfaces::msg::BladeArray::SharedPtr & blades_msg)
{
  if (blades_msg->blades.empty()) {
    tracker_state = LOST;
    return;
  }

  // sort blades by prob
  std::sort(
    blades_msg->blades.begin(), blades_msg->blades.end(),
    [](const buff_interfaces::msg::Blade & a, const buff_interfaces::msg::Blade & b) {
      return a.prob > b.prob;
    });

  last_theta_ = 0.0;
  // retain top 1 blade with transform
  tracked_blade = bladeTransform(blades_msg->blades[0]);

  // init EKF
  initEKF(tracked_blade);
  RCLCPP_DEBUG(rclcpp::get_logger("buff_tracker"), "Init EKF");

  blade_id = 0;
  tracker_state = DETECTING;
  detect_count_ = 0;

  gns.obs.clear();
  solver_status = WAITING;
}

void Tracker::update(const buff_interfaces::msg::BladeArray::SharedPtr & blades_msg)
{
  Eigen::VectorXd ekf_prediction = ekf.predict();
  RCLCPP_DEBUG(rclcpp::get_logger("buff_tracker"), "EKF predict");

  bool is_detected = false;
  target_state = ekf_prediction;
  blade_transform predict_blade;
  calculateMeasurementFromPrediction(predict_blade, ekf_prediction);

  double theta_diff = 0.0;
  center_xoy_diff = 0.0;

  if (!blades_msg->blades.empty()) {
    // sort blades by prob
    std::sort(
      blades_msg->blades.begin(), blades_msg->blades.end(),
      [](const buff_interfaces::msg::Blade & a, const buff_interfaces::msg::Blade & b) {
        return a.prob > b.prob;
      });

    // retain top 1 blade with transform
    tracked_blade = bladeTransform(blades_msg->blades[0]);

    theta_diff = angles::shortest_angular_distance(predict_blade.theta, tracked_blade.theta);
    center_xoy_diff = sqrt(
      pow(tracked_blade.center_position.x - predict_blade.center_position.x, 2) +
      pow(tracked_blade.center_position.y - predict_blade.center_position.y, 2));

    RCLCPP_DEBUG(
      rclcpp::get_logger("buff_tracker"), "tracked_blade.theta: %f, predict_blade.theta: %f",
      tracked_blade.theta, predict_blade.theta);
    RCLCPP_DEBUG(
      rclcpp::get_logger("buff_tracker"), "theta_diff: %f, center_xoy_diff: %f", theta_diff,
      center_xoy_diff);
    if (center_xoy_diff < max_match_center_xoy_ || tracker_state == DETECTING) {
      is_detected = true;
      measurement = Eigen::VectorXd::Zero(4);
      bool need_update = true;
      if (abs(theta_diff) > max_match_theta_) {
        if (!handleBladeJump(theta_diff)) {
          // use predict val
          // lost count plus 1
          measurement << predict_blade.blade_position.x, predict_blade.blade_position.y,
            predict_blade.blade_position.z, predict_blade.theta;
          is_detected = false;
          need_update = false;
        } else {
          // use tracked val to update ekf
          measurement << tracked_blade.blade_position.x, tracked_blade.blade_position.y,
            tracked_blade.blade_position.z, tracked_blade.theta;
        }
      } else {
        measurement << tracked_blade.blade_position.x, tracked_blade.blade_position.y,
          tracked_blade.blade_position.z, tracked_blade.theta;
      }
      // transfer theta from [-pi, pi] to [-inf, inf]
      measurement(3) = last_theta_ + angles::shortest_angular_distance(last_theta_, measurement(3));
      if (need_update) {
        last_theta_ = measurement(3);
        target_state = ekf.update(measurement);
        RCLCPP_DEBUG(rclcpp::get_logger("buff_tracker"), "EKF update");
      }
    } else {
      detect_count_ = detect_count_ ? detect_count_ - 1 : 0;
      lost_count_++;
      RCLCPP_WARN(rclcpp::get_logger("buff_tracker"), "Blade center xoy diff too large");
    }
  }

  // Limit fixed-length variables
  target_state(6) = BLADE_R_OFFSET / 1000;
  // auto distance = pow(target_state(0), 2) + pow(target_state(1), 0.5) * 1000;
  // if (distance > distance + max_distance_diff || distance < distance - max_distance_diff) {
  //   target_state(0) *= distance / distance;
  //   target_state(1) *= distance / distance;
  //   RCLCPP_DEBUG(rclcpp::get_logger("buff_tracker"), "ERROR center distance");
  // }
  ekf.setState(target_state);

  // update tracker state
  if (tracker_state == DETECTING) {
    if (is_detected) {
      detect_count_++;
      if (detect_count_ > tracking_threshold) {
        detect_count_ = 0;
        tracker_state = TRACKING;
      }
    } else {
      detect_count_ = 0;
      tracker_state = LOST;
    }
  } else if (tracker_state == TRACKING) {
    if (!is_detected) {
      lost_count_++;
      tracker_state = TEMP_LOST;
    }
  } else if (tracker_state == TEMP_LOST) {
    if (!is_detected) {
      lost_count_++;
      if (lost_count_ > lost_threshold) {
        lost_count_ = 0;
        tracker_state = LOST;
      }
    } else {
      lost_count_ = 0;
      tracker_state = TRACKING;
    }
  }
}

void Tracker::solve(const rclcpp::Time & time)
{
  if (tracker_state == DETECTING) {
    solver_status = WAITING;
    return;
  } else if (tracker_state == TRACKING && solver_status == WAITING) {
    // first frame
    solver_status = NOT_ENOUGH_OBS;
    obs_start_time = time;
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(3);
    x0 << a_start, w_start, c_start;
    gns.setStartValue(x0);
    spd_state = Eigen::VectorXd::Zero(3);

    ekf_gns.setInitState(x0);
  } else if (tracker_state == TRACKING || tracker_state == TEMP_LOST) {
    double dt = (time - obs_start_time).seconds();
    double spd = abs(target_state(8));

    gns.addObservation(dt, spd);

    if (dt > min_first_solve_time) {
      int fail_count = 0;
      while (gns.solve() != GaussNewtonSolver::SUCCESS) {
        Eigen::VectorXd xn = gns.getState();
        xn(2) += 0.5;
        if (xn(2) > 8) {
          xn(2) -= 8;
        }
        gns.setStartValue(xn);
        fail_count++;
        if (fail_count > 20) {
          RCLCPP_WARN(rclcpp::get_logger("buff_tracker"), "Fail to solve");
          if (solver_status == VALID) {
            spd_state = ekf_gns.predict();
            gns.obs.erase(gns.obs.end() - 1);
          } else {
            solver_status = INVALID;
            gns.obs.erase(gns.obs.begin());
            lost_count_++;
            tracker_state = TEMP_LOST;
          }
          return;
        }
      }
      ekf_gns.predict();
      solver_status = VALID;
      auto gns_state = gns.getState();
      spd_state = ekf_gns.update(gns_state);
      spd_state(2) = angles::normalize_angle_positive(spd_state(2));
    } else {
      solver_status = NOT_ENOUGH_OBS;
    }
  }
  if (lost_count_ > lost_threshold) {
    lost_count_ = 0;
    tracker_state = LOST;
  }
}

void Tracker::initEKF(const blade_transform & blade)
{
  double xc = blade.center_position.x;
  double yc = blade.center_position.y;
  // double zc = (blade_z_ground - robot_z_ground) / 1000;
  double zc = blade.center_position.z;
  double r = BLADE_R_OFFSET / 1000;
  double theta = blade.theta;
  double omega = OMEGA;
  target_state = Eigen::VectorXd::Zero(9);
  target_state << xc, yc, zc, 0, 0, 0, r, theta, omega;
  ekf.setInitState(target_state);
}

bool Tracker::handleBladeJump(double theta_diff)
// theta_diff = measurement - prediction
{
  for (int i = 1; i < 5; i++) {
    double new_theta_diff = theta_diff + i * 2.0 / 5 * PI;
    if (abs(angles::normalize_angle(new_theta_diff)) < max_match_theta_) {
      tracked_blade.theta = angles::normalize_angle(tracked_blade.theta + i * 2.0 / 5 * PI);
      tracked_blade.blade_position = rotateBlade(tracked_blade, i);

      blade_id = (blade_id + i) % 5;
      return true;
    }
  }
  RCLCPP_DEBUG(rclcpp::get_logger("buff_tracker"), "Blade jump to illegal angle, use predict");
  return false;
}

Tracker::blade_transform Tracker::bladeTransform(const buff_interfaces::msg::Blade & blade)
{
  blade_transform blade_tf;
  geometry_msgs::msg::Point blade_pos = blade.pose.position;
  tf2::Quaternion q;
  tf2::convert(blade.pose.orientation, q);
  // rotate [0,0,-BLADE_R_OFFSET/1000] by q
  tf2::Vector3 offset(0, 0, -BLADE_R_OFFSET / 1000);
  tf2::Vector3 offset_rotated = tf2::quatRotate(q, offset);
  Eigen::Vector3d blade_position = Eigen::Vector3d(blade_pos.x, blade_pos.y, blade_pos.z);
  Eigen::Vector3d offset_rotated_eigen(offset_rotated.x(), offset_rotated.y(), offset_rotated.z());
  Eigen::Vector3d center_position = blade_position + offset_rotated_eigen;
  blade_tf.center_position = geometry_msgs::msg::Point();
  blade_tf.center_position.x = center_position.x();
  blade_tf.center_position.y = center_position.y();
  blade_tf.center_position.z = center_position.z();
  // Eigen::Vector3d vector_theta_0(0, 0, 1);
  // Eigen::Vector3d vector_center_blade = blade_position - center_position;
  // double theta = acos(vector_theta_0.dot(vector_center_blade) /
  //                     vector_center_blade.norm()) *
  //                ((center_position.y() * blade_position.x() -
  //                  blade_position.y() * center_position.x()) > 0
  //                     ? 1
  //                     : -1);
  // get Roll
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  double theta = roll;
  // back to first detect blade theta & position
  blade_tf.theta = angles::normalize_angle(theta + blade_id * 2.0 / 5 * PI);
  blade_tf.blade_position = blade_pos;
  blade_tf.blade_position = rotateBlade(blade_tf, blade_id);
  return blade_tf;
}

geometry_msgs::msg::Point Tracker::rotateBlade(const blade_transform blade, int idx)
{
  geometry_msgs::msg::Point rotated_blade_position;
  double theta = idx * 2.0 / 5 * PI;
  double xc, yc, zc;
  xc = blade.center_position.x;
  yc = blade.center_position.y;
  zc = blade.center_position.z;
  tf2::Vector3 offset(-xc, -yc, -zc);
  tf2::Vector3 blade_point(blade.blade_position.x, blade.blade_position.y, blade.blade_position.z);
  tf2::Quaternion q(
    sin(theta / 2) * xc * pow(pow(xc, 2) + pow(yc, 2), -0.5),
    sin(theta / 2) * yc * pow(pow(xc, 2) + pow(yc, 2), -0.5), 0, cos(theta / 2));
  q.normalize();
  tf2::Vector3 rotated_blade_point = tf2::quatRotate(q, blade_point + offset);
  rotated_blade_point -= offset;
  rotated_blade_position.x = rotated_blade_point.x();
  rotated_blade_position.y = rotated_blade_point.y();
  rotated_blade_position.z = rotated_blade_point.z();
  return rotated_blade_position;
}

void Tracker::calculateMeasurementFromPrediction(
  blade_transform & blade, const Eigen::VectorXd & state)
{
  double xc = state(0);
  double yc = state(1);
  double zc = state(2);
  double r = state(6);
  double theta = state(7);

  blade.blade_position.x = xc + r * (sin(theta) * yc * pow(pow(xc, 2) + pow(yc, 2), -0.5));
  blade.blade_position.y = yc + r * (-sin(theta) * xc * pow(pow(xc, 2) + pow(yc, 2), -0.5));
  blade.blade_position.z = zc + r * cos(theta);
  blade.center_position.x = xc;
  blade.center_position.y = yc;
  blade.center_position.z = zc;
  blade.theta = angles::normalize_angle(theta);
}

void Tracker::getTrackerPosition(blade_transform & blade)
{
  calculateMeasurementFromPrediction(blade, target_state);
  blade.theta = angles::normalize_angle(blade.theta - blade_id * 2.0 / 5 * PI);
  blade.blade_position = rotateBlade(blade, -blade_id);
}

}  // namespace rm_buff

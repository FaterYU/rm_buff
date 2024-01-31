#include "buff_tracker/tracker.hpp"

namespace rm_buff
{

Tracker::Tracker(double max_match_theta, double max_match_center_xoy)
: tracker_state(LOST),
  blade_id(0),
  measurement(Eigen::VectorXd::Zero(6)),
  target_state(Eigen::VectorXd::Zero(9)),
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
  double center_xoy_diff = 0.0;

  if (!blades_msg->blades.empty()) {
    // sort blades by prob
    std::sort(
      blades_msg->blades.begin(), blades_msg->blades.end(),
      [](const buff_interfaces::msg::Blade & a, const buff_interfaces::msg::Blade & b) {
        return a.prob > b.prob;
      });

    // retain top 1 blade with transform
    tracked_blade = bladeTransform(blades_msg->blades[0]);

    theta_diff = tracked_blade.theta - predict_blade.theta;
    center_xoy_diff = sqrt(
      pow(tracked_blade.center_position.x - predict_blade.center_position.x, 2) +
      pow(tracked_blade.center_position.y - predict_blade.center_position.y, 2));

    RCLCPP_DEBUG(
      rclcpp::get_logger("buff_tracker"), "tracked_blade.theta: %f, predict_blade.theta: %f",
      tracked_blade.theta, predict_blade.theta);
    RCLCPP_DEBUG(
      rclcpp::get_logger("buff_tracker"), "theta_diff: %f, center_xoy_diff: %f", theta_diff,
      center_xoy_diff);
    if (center_xoy_diff < max_match_center_xoy_) {
      is_detected = true;
      measurement = Eigen::VectorXd::Zero(6);
      if (abs(theta_diff) > max_match_theta_) {
        if (!handleBladeJump(theta_diff)) {
          // use predict val to update ekf
          // lost count plus 1
          measurement << predict_blade.blade_position.x, predict_blade.blade_position.y,
            predict_blade.blade_position.z, predict_blade.center_position.x,
            predict_blade.center_position.y, predict_blade.center_position.z;
          lost_count_++;
        } else {
          // use tracked val to update ekf
          measurement << tracked_blade.blade_position.x, tracked_blade.blade_position.y,
            tracked_blade.blade_position.z, tracked_blade.center_position.x,
            tracked_blade.center_position.y, tracked_blade.center_position.z;
        }
      } else {
        measurement << tracked_blade.blade_position.x, tracked_blade.blade_position.y,
          tracked_blade.blade_position.z, tracked_blade.center_position.x,
          tracked_blade.center_position.y, tracked_blade.center_position.z;
      }
      target_state = ekf.update(measurement);
      RCLCPP_DEBUG(rclcpp::get_logger("buff_tracker"), "EKF update");
    } else {
      RCLCPP_DEBUG(rclcpp::get_logger("buff_tracker"), "Blade center xoy diff too large");
    }
  }

  // Limit fixed-length variables
  target_state(2) = (BLADE_Z_GROUND - ROBOT_Z_GROUND) / 1000;
  target_state(6) = BLADE_R_OFFSET / 1000;
  target_state(8) = OMEGA;
  // auto distance = pow(target_state(0), 2) + pow(target_state(1), 0.5) * 1000;
  // if (distance > DISTENCE + MAX_DISTENCE_DIFF || distance < DISTENCE - MAX_DISTENCE_DIFF) {
  //   target_state(0) *= DISTENCE / distance;
  //   target_state(1) *= DISTENCE / distance;
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
  RCLCPP_DEBUG(rclcpp::get_logger("buff_tracker"), "Tracker state: %d", tracker_state);
}

void Tracker::initEKF(const blade_transform & blade)
{
  double xc = blade.center_position.x;
  double yc = blade.center_position.y;
  double zc = (BLADE_Z_GROUND - ROBOT_Z_GROUND) / 1000;
  double r = BLADE_R_OFFSET / 1000;
  double theta = blade.theta;
  double omega = OMEGA;
  target_state = Eigen::VectorXd::Zero(9);
  target_state << xc, yc, zc, 0, 0, 0, r, theta, omega;
  ekf.setInitState(target_state);
}

bool Tracker::handleBladeJump(double theta_diff)
//theta_diff = measurement - prediction
{
  for (int i = 1; i < 5; i++) {
    double new_theta_diff = norm(theta_diff + i * 2 / 5 * PI, 0, 2 * PI);
    if (abs(new_theta_diff) < max_match_theta_) {
      tracked_blade.blade_position = rotateBlade(tracked_blade.blade_position, -i);
      blade_id = (blade_id + i) % 5;
      RCLCPP_DEBUG(rclcpp::get_logger("buff_tracker"), "Blade jump");
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
  Eigen::Vector3d vector_theta_0(0, 0, 1);
  Eigen::Vector3d vector_center_blade = blade_position - center_position;
  double theta =
    acos(vector_theta_0.dot(vector_center_blade) / vector_center_blade.norm()) *
    ((center_position.y() * blade_position.x() - blade_position.y() * center_position.x()) > 0
       ? 1
       : -1);
  // back to first detect blade theta & position
  RCLCPP_DEBUG(rclcpp::get_logger("buff_tracker"), "last_theta_: %f", last_theta_);
  RCLCPP_DEBUG(rclcpp::get_logger("buff_tracker"), "theta: %f", theta);
  RCLCPP_DEBUG(
    rclcpp::get_logger("buff_tracker"), "distance: %f",
    angles::shortest_angular_distance(last_theta_, theta));
  blade_tf.theta = last_theta_ + angles::shortest_angular_distance(last_theta_, theta);
  last_theta_ = blade_tf.theta;
  blade_tf.blade_position = blade_id == 0 ? blade_pos : rotateBlade(blade_pos, -blade_id);
  return blade_tf;
}

geometry_msgs::msg::Point Tracker::rotateBlade(
  const geometry_msgs::msg::Point & blade_position, int idx)
{
  geometry_msgs::msg::Point rotated_blade_position;
  double theta = idx * 2 / 5 * PI;
  double xc, yc, zc;
  xc = target_state(0);
  yc = target_state(1);
  zc = target_state(2);
  tf2::Vector3 offset(-xc, -yc, -zc);
  tf2::Vector3 blade_point(blade_position.x, blade_position.y, blade_position.z);
  tf2::Quaternion q(cos(theta / 2), sin(theta / 2) * xc, sin(theta / 2) * yc, 0);
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
  blade.theta = theta;
}

double Tracker::norm(double theta, double min_trh, double max_trh)
{
  double diff = max_trh - min_trh;
  double t = (theta - min_trh) / diff;
  double n = floor(t);
  return theta - n * diff;
}

}  // namespace rm_buff
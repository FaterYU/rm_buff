// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#include "buff_tracker/tracker_node.hpp"

namespace rm_buff
{

BuffTrackerNode::BuffTrackerNode(const rclcpp::NodeOptions & options)
: Node("buff_tracker", options)
{
  RCLCPP_INFO(this->get_logger(), "Tracker node initialized");

  // Parameters
  target_frame_ = this->declare_parameter("target_frame", "odom");
  lost_time_threshold_ = this->declare_parameter("tracker.lost_time_threshold", 0.5);
  double max_match_theta = this->declare_parameter("tracker.max_match_theta", 0.628);
  double max_match_center_xoy = this->declare_parameter("tracker.max_match_center_xoy", 10.0);

  tracker_ = std::make_unique<Tracker>(max_match_theta, max_match_center_xoy);
  tracker_->tracking_threshold = this->declare_parameter("tracker.tracking_threshold", 4);
  tracker_->blade_z_ground = this->declare_parameter("tracker.blade_z_ground", 200.0);
  tracker_->robot_z_ground = this->declare_parameter("tracker.robot_z_ground", 200.0);
  tracker_->distance = this->declare_parameter("tracker.distance", 6626.0);
  tracker_->max_distance_diff = this->declare_parameter("tracker.max_distance_diff", 905.0);

  // visulization Initialize
  blade_marker_ = visualization_msgs::msg::Marker();
  blade_marker_.header.frame_id = target_frame_;
  blade_marker_.ns = "blade";
  blade_marker_.id = 0;
  blade_marker_.type = visualization_msgs::msg::Marker::CYLINDER;
  blade_marker_.action = visualization_msgs::msg::Marker::ADD;
  blade_marker_.scale.x = 0.3;
  blade_marker_.scale.y = 0.3;
  blade_marker_.scale.z = 0.01;
  blade_marker_.color.r = 1.0;
  blade_marker_.color.g = 0.0;
  blade_marker_.color.b = 0.0;
  blade_marker_.color.a = 1.0;

  center_marker_ = visualization_msgs::msg::Marker();
  center_marker_.header.frame_id = target_frame_;
  center_marker_.ns = "center";
  center_marker_.id = 0;
  center_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  center_marker_.action = visualization_msgs::msg::Marker::ADD;
  center_marker_.scale.x = 0.1;
  center_marker_.scale.y = 0.1;
  center_marker_.scale.z = 0.1;
  center_marker_.color.r = 0.0;
  center_marker_.color.g = 1.0;
  center_marker_.color.b = 0.0;
  center_marker_.color.a = 1.0;

  measure_marker_ = visualization_msgs::msg::Marker();
  measure_marker_.header.frame_id = target_frame_;
  measure_marker_.ns = "measure";
  measure_marker_.id = 0;
  measure_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  measure_marker_.action = visualization_msgs::msg::Marker::ADD;
  measure_marker_.scale.x = 0.1;
  measure_marker_.scale.y = 0.1;
  measure_marker_.scale.z = 0.1;
  measure_marker_.color.r = 0.0;
  measure_marker_.color.g = 0.0;
  measure_marker_.color.b = 1.0;
  measure_marker_.color.a = 1.0;

  // EKF
  // xc = x_rune_center, xb = x_blade_center
  // state: x, y, z, vx, vy, vz, r, theta, omega
  // measurement: xb, yb, zb, theta
  // f - Process function
  auto f = [this](const Eigen::VectorXd & x) {
    Eigen::VectorXd x_new = x;
    x_new(0) += x(3) * dt_;
    x_new(1) += x(4) * dt_;
    x_new(2) += x(5) * dt_;
    x_new(7) += x(8) * dt_;
    return x_new;
  };
  // J_f - Jacobian of process function
  auto j_f = [this](const Eigen::VectorXd &) {
    Eigen::MatrixXd f(9, 9);
    // clang-format off
    f <<  1,   0,   0,   dt_, 0,   0,   0,   0,   0,
          0,   1,   0,   0,   dt_, 0,   0,   0,   0,
          0,   0,   1,   0,   0,   dt_, 0,   0,   0, 
          0,   0,   0,   1,   0,   0,   0,   0,   0,
          0,   0,   0,   0,   1,   0,   0,   0,   0,
          0,   0,   0,   0,   0,   1,   0,   0,   0,
          0,   0,   0,   0,   0,   0,   1,   0,   0,
          0,   0,   0,   0,   0,   0,   0,   1,   dt_,
          0,   0,   0,   0,   0,   0,   0,   0,   1;
    // clang-format on
    return f;
  };
  // h - Observation function
  auto h = [](const Eigen::VectorXd & x) {
    Eigen::VectorXd z(4);
    double xc = x(0), yc = x(1), zc = x(2), r = x(6), theta = x(7);
    double st = sin(theta), ct = cos(theta);
    double dn_1_2 = pow(xc * xc + yc * yc, -0.5);

    z(0) = xc + r * (st * yc * dn_1_2);   // xb
    z(1) = yc + r * (-st * xc * dn_1_2);  // yb
    z(2) = zc + r * ct;                   // zb
    z(3) = theta;                         // theta
    return z;
  };
  // J_h - Jacobian of observation function
  auto j_h = [](const Eigen::VectorXd & x) {
    Eigen::MatrixXd h(4, 9);
    double xc = x(0), yc = x(1), r = x(6), theta = x(7);
    double st = sin(theta), ct = cos(theta);
    double dn_1_2 = pow(xc * xc + yc * yc, -0.5);
    double dn_3_2 = pow(xc * xc + yc * yc, -1.5);
    // clang-format off
    //    x                      y                      z    v_x  v_y  v_z  r                theta            omega
    h <<  1-r*xc*yc*st*dn_3_2,   r*xc*xc*st*dn_3_2,     0,   0,   0,   0,   yc*st*dn_1_2,    yc*ct*dn_1_2,    0,
          -r*yc*yc*st*dn_3_2,    1+r*xc*yc*st*dn_3_2,   0,   0,   0,   0,   -xc*st*dn_1_2,   -xc*ct*dn_1_2,   0,
          0,                     0,                     1,   0,   0,   0,   ct,              -r*st,           0,
          0,                     0,                     0,   0,   0,   0,   0,               1,               0;
    // clang-format on
    return h;
  };
  // update_Q - process noise covariance matrix
  s2qxyz_ = declare_parameter("ekf.sigma2_q_xyz", 1e-4);
  s2qtheta_ = declare_parameter("ekf.sigma2_q_theta", 1e-2);
  s2qr_ = declare_parameter("ekf.sigma2_q_r", 80.0);
  auto u_q = [this]() {
    Eigen::MatrixXd q(9, 9);
    // double t = dt_, x = s2qxyz_, y = s2qyaw_, r = s2qr_;
    // double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx =
    // pow(t, 2) * x; double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 *
    // x, q_vy_vy = pow(t, 2) * y; double q_r = pow(t, 4) / 4 * r;
    double t = dt_;
    double x = s2qxyz_, y = s2qxyz_, z = s2qxyz_, theta = s2qtheta_, r = s2qr_;
    double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
    double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
    double q_z_z = pow(t, 4) / 4 * z, q_z_vz = pow(t, 3) / 2 * z, q_vz_vz = pow(t, 2) * z;
    double q_r = pow(t, 4) / 4 * r;
    double q_theta = pow(t, 4) / 4 * theta, q_theta_omega = pow(t, 3) / 2 * theta,
           q_omega_omega = pow(t, 2) * theta;
    // clang-format off
    //    x       y       z       v_x     v_y     v_z     r       theta          omega
    q <<  q_x_x,  0,      0,      q_x_vx, 0,      0,      0,      0,             0,
          0,      q_y_y,  0,      0,      q_y_vy, 0,      0,      0,             0,
          0,      0,      q_z_z,  0,      0,      0,      q_z_vz, 0,             0,
          q_x_vx, 0,      0,      q_vx_vx,0,      0,      0,      0,             0,
          0,      q_y_vy, 0,      0,      q_vy_vy,0,      0,      0,             0,
          0,      0,      q_z_vz, 0,      0,      q_vz_vz,0,      0,             0,
          0,      0,      0,      0,      0,      0,      q_r,    0,             0,
          0,      0,      0,      0,      0,      0,      0,      q_theta,       q_theta_omega,
          0,      0,      0,      0,      0,      0,      0,      q_theta_omega, q_omega_omega;
    // clang-format on
    return q;
  };
  // update_R - measurement noise covariance matrix
  r_blade_ = declare_parameter("ekf.r_blade", 1e-8);
  r_center_ = declare_parameter("ekf.r_center", 1e-8);
  auto u_r = [this](const Eigen::VectorXd & z) {
    Eigen::DiagonalMatrix<double, 4> r;
    double xb = r_blade_;
    double xc = r_center_;
    r.diagonal() << abs(xb * z(0)), abs(xb * z(1)), abs(xb * z(2)), abs(xc * z(3));
    return r;
  };
  // P - error estimate covariance matrix
  Eigen::DiagonalMatrix<double, 9> p0;
  p0.setIdentity();
  tracker_->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};

  // GaussNewtonSolver
  auto u_fx = [](const Eigen::VectorXd & x, const std::vector<double> & ob) {
    double t = ob.at(0);
    double y = ob.at(1);
    double a = x(0), w = x(1), c = x(2);
    Eigen::MatrixXd fx(1, 1);

    fx << y - (a * sin(w * t + c) + (2.09 - a));
    return fx;
  };

  auto u_J = [](const Eigen::VectorXd & x, const std::vector<double> & ob) {
    double t = ob.at(0);
    double a = x(0), w = x(1), c = x(2);
    Eigen::MatrixXd J(1, 3);

    // clang-format off
    //   a                     w                         c
    J << -sin(w * t + c) + 1,  -t * a * cos(w * t + c),  -a * cos(w * t + c);
    // clang-format on
    return J;
  };

  min_a_ = declare_parameter("gns.min_a", 0.4);
  max_a_ = declare_parameter("gns.max_a", 1.3);
  min_w_ = declare_parameter("gns.min_w", 1.5);
  max_w_ = declare_parameter("gns.max_w", 2.3);
  auto constraint = [this](const Eigen::VectorXd & x) {
    double a = x(0), w = x(1);
    bool a_con = a > min_a_ && a < max_a_;
    bool w_con = w > min_w_ && w < max_w_;
    return a_con && w_con;
  };

  max_iter_ = declare_parameter("gns.max_iter", 50);
  min_step_ = declare_parameter("gns.min_step", 1e-10);
  obs_max_size_ = declare_parameter("gns.obs_max_size", 150);
  tracker_->gns = GaussNewtonSolver{u_fx, u_J, constraint, max_iter_, min_step_, obs_max_size_};

  tracker_->a_start = declare_parameter("gns.a_start", 0.9125);
  tracker_->w_start = declare_parameter("gns.w_start", 1.942);
  tracker_->c_start = declare_parameter("gns.c_start", 0.0);
  tracker_->min_first_solve_time = declare_parameter("gns.min_first_solve_time", 2.0);

  // GNS EKF
  // state: a, w, c
  // measurement: a, w, c
  // f - Process function
  auto f_gns = [this](const Eigen::VectorXd & x) { return x; };
  // J_f - Jacobian of process function
  auto j_f_gns = [this](const Eigen::VectorXd &) {
    Eigen::MatrixXd f(3, 3);
    f.setIdentity();
    return f;
  };
  // h - Observation function
  auto h_gns = [](const Eigen::VectorXd & x) { return x; };
  // J_h - Jacobian of observation function
  auto j_h_gns = [this](const Eigen::VectorXd &) {
    Eigen::MatrixXd h(3, 3);
    h.setIdentity();
    return h;
  };
  // update_Q - process noise covariance matrix
  s2q_a_ = declare_parameter("ekf_gns.sigma2_q_a", 0.1);
  s2q_w_ = declare_parameter("ekf_gns.sigma2_q_w", 0.1);
  s2q_c_ = declare_parameter("ekf_gns.sigma2_q_c", 100.0);
  auto u_q_gns = [this]() {
    Eigen::MatrixXd q(3, 3);
    // clang-format off
    q <<  s2q_a_, 0,       0,
          0,       s2q_w_, 0,
          0,       0,       s2q_c_;
    // clang-format on
    return q;
  };
  // update_R - measurement noise covariance matrix
  r_a_ = declare_parameter("ekf_gns.r_a", 1e-8);
  r_w_ = declare_parameter("ekf_gns.r_w", 5e-4);
  r_c_ = declare_parameter("ekf_gns.r_c", 1e-8);
  auto u_r_gns = [this](const Eigen::VectorXd & z) {
    Eigen::DiagonalMatrix<double, 3> r;
    r.diagonal() << abs(r_a_ * z(0)), abs(r_w_ * z(1)), abs(r_c_ * z(2));
    return r;
  };
  // P - error estimate covariance matrix
  Eigen::DiagonalMatrix<double, 3> p0_gns;
  p0_gns.setIdentity();
  tracker_->ekf_gns =
    ExtendedKalmanFilter{f_gns, h_gns, j_f_gns, j_h_gns, u_q_gns, u_r_gns, p0_gns};

  // Task subscriber
  task_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/task_mode", 10, std::bind(&BuffTrackerNode::taskCallback, this, std::placeholders::_1));

  // Create publishers
  rune_publisher_ = this->create_publisher<buff_interfaces::msg::Rune>("tracker/rune", 10);
  rune_info_publisher_ =
    this->create_publisher<buff_interfaces::msg::RuneInfo>("tracker/rune_info", 10);
  blade_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("tracker/blade_marker", 10);
  center_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("tracker/center_marker", 10);
  measure_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("tracker/measure_marker", 10);
  pnp_result_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/debug/buff_pnp", 10);

  // Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // subscriber and filter
  blades_sub_.subscribe(this, "/detector/blade_array", rmw_qos_profile_sensor_data);
  tf2_filter_ = std::make_shared<tf2_filter>(
    blades_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(),
    this->get_node_clock_interface(), std::chrono::duration<int>(1));
  // Register a callback with tf2_ros::MessageFilter to be called when
  // transforms are available
  tf2_filter_->registerCallback(&BuffTrackerNode::bladesCallback, this);
}

void BuffTrackerNode::taskCallback(const std_msgs::msg::String::SharedPtr task_msg)
{
  task_mode_ = task_msg->data;
}

void BuffTrackerNode::bladesCallback(const buff_interfaces::msg::BladeArray::SharedPtr blades_msg)
{
  // Tranform blade position from image frame to world coordinate
  for (auto & blade : blades_msg->blades) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf2_buffer_->lookupTransform(
        target_frame_, blades_msg->header.frame_id, blades_msg->header.stamp,
        rclcpp::Duration::from_seconds(0.0));
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }
    tf2::doTransform(blade.pose, blade.pose, transform_stamped);
  }

  // filter ignored blades
  blades_msg->blades.erase(
    std::remove_if(
      blades_msg->blades.begin(), blades_msg->blades.end(),
      [](const buff_interfaces::msg::Blade & blade) { return blade.label % 2 == 1; }),
    blades_msg->blades.end());

  // Init message
  rclcpp::Time time = blades_msg->header.stamp;
  buff_interfaces::msg::RuneInfo rune_info_msg;
  rune_info_msg.header.stamp = time;
  rune_info_msg.header.frame_id = target_frame_;
  buff_interfaces::msg::Rune rune_msg;
  rune_msg.header.stamp = time;
  rune_msg.header.frame_id = target_frame_;
  geometry_msgs::msg::PoseArray pnp_result_msg;
  pnp_result_msg.header = rune_msg.header;

  measure_marker_.header.stamp = time;
  if (blades_msg->blades.size() != 0) {
    measure_marker_.pose.position.x = blades_msg->blades[0].pose.position.x;
    measure_marker_.pose.position.y = blades_msg->blades[0].pose.position.y;
    measure_marker_.pose.position.z = blades_msg->blades[0].pose.position.z;
    measure_marker_pub_->publish(measure_marker_);
    pnp_result_msg.poses.emplace_back(blades_msg->blades[0].pose);
    pnp_result_pub_->publish(pnp_result_msg);
  }

  // Update tracker
  if (tracker_->tracker_state == Tracker::State::LOST) {
    tracker_->init(blades_msg);
    rune_msg.tracking = false;
  } else {
    dt_ = (time - last_time_).seconds();
    tracker_->lost_threshold = static_cast<int>(lost_time_threshold_ / dt_);
    tracker_->update(blades_msg);
    if (task_mode_ == "large_buff") {
      tracker_->solve(time);
    }

    // Publish rune
    rune_info_msg.blade.x = tracker_->tracked_blade.blade_position.x;
    rune_info_msg.blade.y = tracker_->tracked_blade.blade_position.y;
    rune_info_msg.blade.z = tracker_->tracked_blade.blade_position.z;
    rune_info_msg.center.x = tracker_->tracked_blade.center_position.x;
    rune_info_msg.center.y = tracker_->tracked_blade.center_position.y;
    rune_info_msg.center.z = tracker_->tracked_blade.center_position.z;

    if (tracker_->tracker_state == Tracker::State::DETECTING) {
      rune_msg.tracking = false;
    } else if (
      tracker_->tracker_state == Tracker::State::TRACKING ||
      tracker_->tracker_state == Tracker::State::TEMP_LOST) {
      rune_msg.tracking = true;
      const auto & state = tracker_->target_state;

      // calculate from prediction
      Tracker::blade_transform predict_blade;
      tracker_->getTrackerPosition(predict_blade);

      rune_msg.position = predict_blade.center_position;
      rune_msg.velocity.x = state(3);
      rune_msg.velocity.y = state(4);
      rune_msg.velocity.z = state(5);
      rune_msg.r = state(6);
      rune_msg.theta = predict_blade.theta;
      rune_info_msg.speed = state(8);
      rune_msg.a = 0.0;
      rune_msg.w = 0.0;
      rune_msg.c = 0.0;
      rune_msg.b = state(8);
      auto now_sec = time.seconds();
      auto obs_time = tracker_->obs_start_time.seconds();
      if (task_mode_ == "large_buff") {
        const auto & gns_state = tracker_->spd_state;
        int sign = state(8) > 0 ? 1 : -1;
        if (tracker_->solver_status == Tracker::SolverStatus::VALID) {
          rune_msg.a = gns_state(0) * sign;
          rune_msg.w = gns_state(1);
          rune_msg.c = gns_state(2);
          rune_msg.b = (2.09 - gns_state(0)) * sign;
          int T = 2 * PI / rune_msg.w * 1000;
          rune_msg.t_offset = int((now_sec - obs_time + rune_msg.c / rune_msg.w) * 1000) % T;
        } else {
          rune_msg.tracking = tracker_->solver_status == Tracker::SolverStatus::INVALID;
        }
      }
      rune_info_msg.predicted_speed =
        rune_msg.a * sin(1.0 * rune_msg.t_offset / 1000.0 * rune_msg.w) + rune_msg.b;
      rune_msg.offset_id = tracker_->blade_id;

      // Publish visualization
      center_marker_.header.stamp = time;
      center_marker_.pose.position.x = predict_blade.center_position.x;
      center_marker_.pose.position.y = predict_blade.center_position.y;
      center_marker_.pose.position.z = predict_blade.center_position.z;
      center_marker_pub_->publish(center_marker_);

      blade_marker_.header.stamp = time;
      blade_marker_.pose.position.x = predict_blade.blade_position.x;
      blade_marker_.pose.position.y = predict_blade.blade_position.y;
      blade_marker_.pose.position.z = predict_blade.blade_position.z;
      auto q = tf2::Quaternion();
      q.setRPY(atan2(predict_blade.center_position.y, predict_blade.center_position.x), -PI / 2, 0);
      blade_marker_.pose.orientation = tf2::toMsg(q);
      blade_marker_pub_->publish(blade_marker_);
    }
    rune_info_publisher_->publish(rune_info_msg);
  }

  last_time_ = time;
  rune_publisher_->publish(rune_msg);
}
}  // namespace rm_buff

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_buff::BuffTrackerNode)

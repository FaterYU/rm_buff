#include "buff_tracker/tracker_node.hpp"

namespace rm_buff
{

BuffTrackerNode::BuffTrackerNode(const rclcpp::NodeOptions & options)
: Node("tracker_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Tracker node initialized");

  // Parameters
  lost_time_threshold_ = this->declare_parameter("tracker.lost_time_threshold", 0.5);
  double max_match_theta = this->declare_parameter("tracker.max_match_theta", 0.628);
  double max_match_center_xoy = this->declare_parameter("tracker.max_match_center_xoy", 10.0);

  tracker_ = std::make_unique<Tracker>(max_match_theta, max_match_center_xoy);
  tracker_->tracking_threshold = this->declare_parameter("tracker.tracking_threshold", 4);

  // EKF
  // xc = x_rune_center, xb = x_blade_center
  // state: x, y, z, vx, vy, vz, r, theta, omega
  // measurement: xb, yb, zb, xc, yc, zc
  // f - Process function
  auto f = [this](const Eigen::VectorXd & x) {
    Eigen::VectorXd x_new = x;
    x_new(0) += x(3) * dt_;
    x_new(1) += x(4) * dt_;
    // x_new(2) += x(6) * dt_;
    x_new(7) += x(8) * dt_;
    std::cout << "x(7): " << x(7) << " x_new(7): " << x_new(7) << std::endl;
    return x_new;
  };
  // J_f - Jacobian of process function
  auto j_f = [this](const Eigen::VectorXd &) {
    Eigen::MatrixXd f(9, 9);
    // clang-format off
    f <<  1,   0,   0,   dt_, 0,   0,   0,   0,   0,
          0,   1,   0,   0,   dt_, 0,   0,   0,   0,
          0,   0,   1,   0,   0,   0,   0,   0,   0, 
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
    Eigen::VectorXd z(6);
    double xc = x(0), yc = x(1), zc = x(2), r = x(6), theta = -x(7);
    double st = sin(theta), ct = cos(theta);
    double dn_1_2 = pow(xc * xc + yc * yc, -0.5);

    z(0) = xc + r * (st * yc * dn_1_2);   // xb
    z(1) = yc + r * (-st * xc * dn_1_2);  // yb
    z(2) = zc + r * ct;                   // zb
    z(3) = xc;                            // xc
    z(4) = yc;                            // yc
    z(5) = zc;                            // zc
    return z;
  };
  // J_h - Jacobian of observation function
  auto j_h = [](const Eigen::VectorXd & x) {
    Eigen::MatrixXd h(6, 9);
    double xc = x(0), yc = x(1), r = x(6), theta = -x(7);
    double st = sin(theta), ct = cos(theta);
    double dn_1_2 = pow(xc * xc + yc * yc, -0.5);
    double dn_3_2 = pow(xc * xc + yc * yc, -1.5);
    // clang-format off
    //    x    y    z    v_x  v_y  v_z  r           theta            omega
    h <<  1-r*xc*yc*st*dn_3_2,   r*xc*xc*st*dn_3_2,   0,   0,   0,   0,   yc*st*dn_1_2,   yc*ct*dn_1_2,   0,

          -r*yc*yc*st*dn_3_2,   1+r*xc*yc*st*dn_3_2,   0,   0,   0,   0,   xc*st*dn_1_2,   -xc*ct*dn_1_2,   0,

          1,   0,   0,   0,   0,   0,   ct,   -r*st,   0,

          1,   0,   0,   0,   0,   0,   0,   0,   0,
          0,   1,   0,   0,   0,   0,   0,   0,   0,
          0,   0,   1,   0,   0,   0,   0,   0,   0;
    // clang-format on
    return h;
  };
  // update_Q - process noise covariance matrix
  // s2qxyz_ = declare_parameter("ekf.sigma2_q_xyz", 20.0);
  // s2qyaw_ = declare_parameter("ekf.sigma2_q_yaw", 100.0);
  // s2qr_ = declare_parameter("ekf.sigma2_q_r", 800.0);
  auto u_q = [this]() {
    Eigen::MatrixXd q(9, 9);
    // double t = dt_, x = s2qxyz_, y = s2qyaw_, r = s2qr_;
    // double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
    // double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
    // double q_r = pow(t, 4) / 4 * r;
    double t = dt_;
    double x = 20.0, y = 100.0, z = 20.0, theta = 100.0, r = 800.0;
    double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
    double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
    double q_z_z = pow(t, 4) / 4 * z, q_z_vz = pow(t, 3) / 2 * x, q_vz_vz = pow(t, 2) * z;
    double q_r = pow(t, 4) / 4 * r;
    double q_theta = pow(t, 4) / 4 * theta;
    // clang-format off
    //    x       y       z       v_x     v_y     v_z     r       theta   omega
    q <<  q_x_x,  0,      0,      q_x_vx, 0,      0,      0,      0,      0,
          0,      q_y_y,  0,      0,      q_y_vy, 0,      0,      0,      0,
          0,      0,      q_z_z,  0,      0,      0,      q_z_vz, 0,      0,
          q_x_vx, 0,      0,      q_vx_vx,0,      0,      0,      0,      0,
          0,      q_y_vy, 0,      0,      q_vy_vy,0,      0,      0,      0,
          0,      0,      q_z_vz, 0,      0,      q_vz_vz,0,      0,      0,
          0,      0,      0,      0,      0,      0,      q_r,    0,      0,
          0,      0,      0,      0,      0,      0,      0,      q_theta,0,
          0,      0,      0,      0,      0,      0,      0,      0,      0;
    // clang-format on
    return q;
  };
  // update_R - measurement noise covariance matrix
  // r_blade = declare_parameter("ekf.r_blade", 0.05);
  // r_center = declare_parameter("ekf.r_center", 0.02);
  auto u_r = [this](const Eigen::VectorXd & z) {
    Eigen::DiagonalMatrix<double, 6> r;
    double x = 0.05;
    r.diagonal() << abs(x * z(0)), abs(x * z(1)), abs(x * z(2)), abs(x * z(3)), abs(x * z(4)),
      abs(x * z(5));
    return r;
  };
  // P - error estimate covariance matrix
  Eigen::DiagonalMatrix<double, 9> p0;
  p0.setIdentity();
  tracker_->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};

  // Create publishers
  rune_publisher_ = this->create_publisher<buff_interfaces::msg::Rune>("/tracker/rune", 10);
  rune_info_publisher_ =
    this->create_publisher<buff_interfaces::msg::RuneInfo>("tracker/rune_info", 10);

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
  // target_frame_ = this->declare_parameter("target_frame", "odom");
  target_frame_ = "odom";
  tf2_filter_ = std::make_shared<tf2_filter>(
    blades_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(),
    this->get_node_clock_interface(), std::chrono::duration<int>(1));
  // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
  tf2_filter_->registerCallback(&BuffTrackerNode::bladesCallback, this);
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

  // Update tracker
  if (tracker_->tracker_state == Tracker::State::LOST) {
    tracker_->init(blades_msg);
  } else {
    dt_ = (time - last_time_).seconds();
    tracker_->lost_threshold = static_cast<int>(lost_time_threshold_ / dt_);
    tracker_->update(blades_msg);

    // Publish rune
    rune_info_msg.blade.x = tracker_->measurement(0);
    rune_info_msg.blade.y = tracker_->measurement(1);
    rune_info_msg.blade.z = tracker_->measurement(2);
    rune_info_msg.center.x = tracker_->measurement(3);
    rune_info_msg.center.y = tracker_->measurement(4);
    rune_info_msg.center.z = tracker_->measurement(5);
    rune_info_publisher_->publish(rune_info_msg);

    if (tracker_->tracker_state == Tracker::State::DETECTING) {
      rune_msg.tracking = false;
    } else if (
      tracker_->tracker_state == Tracker::State::TRACKING ||
      tracker_->tracker_state == Tracker::State::TEMP_LOST) {
      rune_msg.tracking = true;
      const auto & state = tracker_->target_state;
      rune_msg.position.x = state(0);
      rune_msg.position.y = state(1);
      rune_msg.position.z = state(2);
      rune_msg.velocity.x = state(3);
      rune_msg.velocity.y = state(4);
      rune_msg.velocity.z = state(5);
      rune_msg.r = state(6);
      rune_msg.theta = state(7);
      rune_msg.omega = state(8);
      rune_msg.offset_id = tracker_->blade_id;
    }
  }

  last_time_ = time;
  rune_publisher_->publish(rune_msg);
}
}  // namespace rm_buff

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_buff::BuffTrackerNode)
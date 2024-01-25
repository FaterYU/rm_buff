#include "buff_detector/detector_node.hpp"

namespace rm_buff {
BuffDetectorNode::BuffDetectorNode(const rclcpp::NodeOptions& options)
    : Node("detector_node", options) {
  blades_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/detector/blade_array", 10);
  debug_blades_publisher_ =
      this->create_publisher<buff_interfaces::msg::DebugBladeArray>(
          "/debug/blade_array", 10);
  latency_publisher_ =
      this->create_publisher<std_msgs::msg::String>("/latency/inference", 10);
  result_img_pub_ =
      image_transport::create_publisher(this, "/detector/result_img");

  auto pkg_path = ament_index_cpp::get_package_share_directory("buff_detector");
  detector_ =
      std::make_unique<Detector>(pkg_path + "/models/best_quantized.xml");
  RCLCPP_INFO(this->get_logger(), "Model loaded");

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
        cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
        cam_info_ =
            std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
        pnp_solver_ =
            std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
        cam_info_sub_.reset();
      });

  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&BuffDetectorNode::imageCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Detector node initialized");
}

void BuffDetectorNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  if (pnp_solver_ != nullptr) {
    auto blades = DetectBlades(msg);

    geometry_msgs::msg::PoseArray blade_array;
    blade_array.header = msg->header;
    blade_array.header.frame_id = "camera_optical_frame";
    buff_interfaces::msg::DebugBladeArray debug_blade_array;

    // publish blade array
    for (auto& blade : blades) {
      buff_interfaces::msg::DebugBlade debug_blade_msg;
      debug_blade_msg.x = blade.rect.x;
      debug_blade_msg.y = blade.rect.y;
      debug_blade_msg.width = blade.rect.width;
      debug_blade_msg.height = blade.rect.height;
      debug_blade_msg.label = blade.label;
      debug_blade_msg.prob = blade.prob;
      for (auto& kpt : blade.kpt) {
        geometry_msgs::msg::Point point;
        point.x = kpt.x;
        point.y = kpt.y;
        debug_blade_msg.kpt.emplace_back(point);
      }
      // solve pnp
      cv::Mat rvec, tvec;
      if (pnp_solver_->solvePnP(blade, rvec, tvec)) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = tvec.at<double>(0);
        pose.position.y = tvec.at<double>(1);
        pose.position.z = tvec.at<double>(2);

        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);

        tf2::Matrix3x3 tf_rotation_matrix(
            rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
            rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
            rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
            rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
            rotation_matrix.at<double>(2, 2));
        tf2::Quaternion tf_quaternion;
        tf_rotation_matrix.getRotation(tf_quaternion);
        pose.orientation = tf2::toMsg(tf_quaternion);
        debug_blade_msg.pose = pose;
        blade_array.poses.emplace_back(pose);

        geometry_msgs::msg::Point center;
        center.x = blade.kpt[2].x;
        center.y = blade.kpt[2].y;
        debug_blade_msg.center = center;
      } else {
        RCLCPP_WARN(this->get_logger(), "PnP failed");
      }
      debug_blade_array.blades.emplace_back(debug_blade_msg);
    }
    blades_publisher_->publish(blade_array);
    debug_blades_publisher_->publish(debug_blade_array);
  }
}

std::vector<Blade> BuffDetectorNode::DetectBlades(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg) {
  auto img = cv_bridge::toCvShare(image_msg, "rgb8")->image;

  // start time
  auto start = std::chrono::steady_clock::now();
  std::vector<Blade> result = detector_->Detect(img);

  // end time
  auto end = std::chrono::steady_clock::now();

  // publish
  auto time =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  std_msgs::msg::String latency_msg;
  latency_msg.data = std::to_string(time.count());
  latency_publisher_->publish(latency_msg);
  if (result.size() == 0) {
    RCLCPP_DEBUG(this->get_logger(), "No blade detected");
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Blade detected");
    // draw blade
    detector_->draw_blade(img);
  }
  result_img_pub_.publish(
      cv_bridge::CvImage(image_msg->header, "rgb8", img).toImageMsg());

  return result;
}

}  // namespace rm_buff

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_buff::BuffDetectorNode)
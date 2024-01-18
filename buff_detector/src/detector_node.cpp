#include "buff_detector/detector_node.hpp"

namespace rm_buff {
BuffDetectorNode::BuffDetectorNode(const rclcpp::NodeOptions& options)
    : Node("detector_node", options) {
  publisher_ =
      this->create_publisher<buff_interfaces::msg::BladeArray>("result", 10);
  auto pkg_path = ament_index_cpp::get_package_share_directory("buff_detector");
  //   detector_ = std::make_unique<Detector>(pkg_path +
  //   "/models/best_100b.xml");
  //   detector_ =
  //       std::make_unique<Detector>(pkg_path + "/models/half/best_100b.xml");
  detector_ =
      std::make_unique<Detector>(pkg_path + "/models/best_100b_quantized.xml");

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
        cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
        cam_info_ =
            std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
        // pnp_solver_ =
        //     std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
        cam_info_sub_.reset();
      });

  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&BuffDetectorNode::imageCallback, this, std::placeholders::_1));
}

void BuffDetectorNode::imageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Image received");
  RCLCPP_INFO(this->get_logger(), "Image size: %dx%d", msg->width, msg->height);

  // Convert to cv::Mat
  auto img = cv_bridge::toCvShare(msg, "rgb8")->image;

  RCLCPP_INFO(this->get_logger(), "Image converted");

  // start time
  auto start = std::chrono::steady_clock::now();
  buff_interfaces::msg::BladeArray result = detector_->Detect(img);

  // end time
  auto end = std::chrono::steady_clock::now();
  // log time
  auto time =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  RCLCPP_INFO(this->get_logger(), "Frame time:  %ldms", time.count());

  publisher_->publish(result);
}

}  // namespace rm_buff

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_buff::BuffDetectorNode)
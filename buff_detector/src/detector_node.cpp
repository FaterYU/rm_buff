#include "buff_detector/detector_node.hpp"

namespace rm_buff {
BuffDetectorNode::BuffDetectorNode(const rclcpp::NodeOptions& options)
    : Node("detector_node", options) {
  publisher_ = this->create_publisher<buff_interfaces::msg::BladeArray>(
      "/detector/result", 10);
  latency_publisher_ =
      this->create_publisher<std_msgs::msg::String>("/latency/inference", 10);
  result_img_pub_ =
      image_transport::create_publisher(this, "/detector/result_img");

  auto pkg_path = ament_index_cpp::get_package_share_directory("buff_detector");
  detector_ =
      std::make_unique<Detector>(pkg_path + "/models/best_100b_quantized.xml");
  RCLCPP_INFO(this->get_logger(), "Model loaded");

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

  RCLCPP_INFO(this->get_logger(), "Detector node initialized");
}

void BuffDetectorNode::imageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  auto img = cv_bridge::toCvShare(msg, "rgb8")->image;

  // start time
  auto start = std::chrono::steady_clock::now();
  buff_interfaces::msg::BladeArray result = detector_->Detect(img);

  // end time
  auto end = std::chrono::steady_clock::now();

  // publish
  auto time =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  std_msgs::msg::String latency_msg;
  latency_msg.data = std::to_string(time.count());
  latency_publisher_->publish(latency_msg);
  if (result.blades.size() == 0) {
    RCLCPP_DEBUG(this->get_logger(), "No blade detected");
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Blade detected");
    // draw blade
    detector_->draw_blade(img);
  }
  result_img_pub_.publish(
      cv_bridge::CvImage(msg->header, "rgb8", img).toImageMsg());

  publisher_->publish(result);
}

}  // namespace rm_buff

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_buff::BuffDetectorNode)
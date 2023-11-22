#include "buff_detector/detector_node.hpp"

namespace rm_buff {
BuffDetectorNode::BuffDetectorNode(const rclcpp::NodeOptions& options)
    : Node("detector_node", options) {
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&BuffDetectorNode::timer_callback, this));
  count_ = 0;
  auto pkg_path = ament_index_cpp::get_package_share_directory("buff_detector");
  detector_ = Detector(pkg_path + "/models/best_100b.xml");
  std::string img_path = pkg_path + "/images/sample.jpg";
  img_ = cv::imread(img_path);
}

void BuffDetectorNode::timer_callback() {
  std::vector<float> result = detector_.Detect(img_);
  auto message = std_msgs::msg::String();
  // log result[0]
  message.data = "Frame result:  " + std::to_string(result[0]);
  RCLCPP_INFO(this->get_logger(), message.data.c_str());
  publisher_->publish(message);
}

}  // namespace rm_buff

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_buff::BuffDetectorNode)
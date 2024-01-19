#ifndef BUFF_DETECTOR__DETECTOR_NODE_HPP_
#define BUFF_DETECTOR__DETECTOR_NODE_HPP_

#include <cv_bridge/cv_bridge.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <buff_interfaces/msg/blade.hpp>
#include <buff_interfaces/msg/blade_array.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "buff_detector/detector.hpp"

namespace rm_buff {
class BuffDetectorNode : public rclcpp::Node {
 public:
  explicit BuffDetectorNode(const rclcpp::NodeOptions& options);

 private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  // Camera info part
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  cv::Point2f cam_center_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;

  rclcpp::Publisher<buff_interfaces::msg::BladeArray>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr latency_publisher_;
  image_transport::Publisher result_img_pub_;

  //   Detetor
  std::unique_ptr<Detector> detector_;

  //   Image Subscrpition
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
};
}  // namespace rm_buff

#endif  // BUFF_DETECTOR__DETECTOR_NODE_HPP_

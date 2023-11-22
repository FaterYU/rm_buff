#ifndef BUFF_DETECTOR__DETECTOR_NODE_HPP_
#define BUFF_DETECTOR__DETECTOR_NODE_HPP_

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include "buff_detector/detector.hpp"

namespace rm_buff {
class BuffDetectorNode : public rclcpp::Node {
 public:
  explicit BuffDetectorNode(const rclcpp::NodeOptions& options);

 private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  void timer_callback();

  //   Detetor
  Detector detector_;

  //   Image Subscrpition
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  cv::Mat img_;
};
}  // namespace rm_buff

#endif  // BUFF_DETECTOR__DETECTOR_NODE_HPP_

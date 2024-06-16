// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef BUFF_DETECTOR__DETECTOR_NODE_HPP_
#define BUFF_DETECTOR__DETECTOR_NODE_HPP_

#include <cv_bridge/cv_bridge.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <buff_detector/blade.hpp>
#include <buff_detector/pnp_solver.hpp>
#include <buff_interfaces/msg/blade.hpp>
#include <buff_interfaces/msg/blade_array.hpp>
#include <buff_interfaces/msg/debug_blade.hpp>
#include <buff_interfaces/msg/debug_blade_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "buff_detector/detector.hpp"

namespace rm_buff
{
class BuffDetectorNode : public rclcpp::Node
{
public:
  explicit BuffDetectorNode(const rclcpp::NodeOptions & options);

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  std::vector<Blade> DetectBlades(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);

  //  task subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
  bool is_buff_task_;
  void taskCallback(const std_msgs::msg::String::SharedPtr task_msg);

  // Camera info part
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  cv::Point2f cam_center_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;

  // PnP Solver
  std::unique_ptr<PnPSolver> pnp_solver_;

  rclcpp::Publisher<buff_interfaces::msg::BladeArray>::SharedPtr blades_publisher_;
  rclcpp::Publisher<buff_interfaces::msg::DebugBladeArray>::SharedPtr debug_blades_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr latency_publisher_;
  image_transport::Publisher result_img_pub_;
  image_transport::Publisher bin_img_pub_;
  image_transport::Publisher debug_img_pub_;

  //   Detetor
  std::unique_ptr<Detector> detector_;

  //   Image Subscrpition
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
};
}  // namespace rm_buff

#endif  // BUFF_DETECTOR__DETECTOR_NODE_HPP_

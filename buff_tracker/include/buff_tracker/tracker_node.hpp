// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef BUFF_TRACKER__TRACKER_NODE_HPP_
#define BUFF_TRACKER__TRACKER_NODE_HPP_

// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <buff_interfaces/msg/blade.hpp>
#include <buff_interfaces/msg/blade_array.hpp>
#include <buff_interfaces/msg/rune.hpp>
#include <buff_interfaces/msg/rune_info.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "buff_tracker/extended_kalman_filter.hpp"
#include "buff_tracker/gauss_newton_solver.hpp"
#include "buff_tracker/tracker.hpp"

namespace rm_buff
{

using tf2_filter = tf2_ros::MessageFilter<buff_interfaces::msg::BladeArray>;

class BuffTrackerNode : public rclcpp::Node
{
public:
  explicit BuffTrackerNode(const rclcpp::NodeOptions & options);

private:
  void bladesCallback(const buff_interfaces::msg::BladeArray::SharedPtr blades_msg);
  double dt_;

  rclcpp::Time last_time_;

  double lost_time_threshold_;

  std::unique_ptr<Tracker> tracker_;

  // param
  double s2qxyz_;
  double s2qtheta_;
  double s2qr_;
  double r_blade_;
  double r_center_;

  int max_iter_;
  double min_step_;
  int obs_max_size_;
  double max_a_;
  double min_a_;
  double max_w_;
  double min_w_;

  double s2q_a_;
  double s2q_w_;
  double s2q_c_;
  double r_a_;
  double r_w_;
  double r_c_;

  //  task subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
  std::string task_mode_;
  void taskCallback(const std_msgs::msg::String::SharedPtr task_msg);

  // Subscriber with tf2 message_filter
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<buff_interfaces::msg::BladeArray> blades_sub_;
  std::shared_ptr<tf2_filter> tf2_filter_;

  // Publisher
  rclcpp::Publisher<buff_interfaces::msg::Rune>::SharedPtr rune_publisher_;
  rclcpp::Publisher<buff_interfaces::msg::RuneInfo>::SharedPtr rune_info_publisher_;

  // visualization
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr blade_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr center_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr measure_marker_pub_;
  visualization_msgs::msg::Marker blade_marker_;
  visualization_msgs::msg::Marker center_marker_;
  visualization_msgs::msg::Marker measure_marker_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pnp_result_pub_;
};
}  // namespace rm_buff

#endif  // BUFF_TRACKER__TRACKER_NODE_HPP_
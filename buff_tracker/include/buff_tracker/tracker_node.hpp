#ifndef BUFF_TRACKER__TRACKER_NODE_HPP_
#define BUFF_TRACKER__TRACKER_NODE_HPP_

#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>

namespace rm_buff {

class BuffTrackerNode : public rclcpp::Node {
 public:
  explicit BuffTrackerNode(const rclcpp::NodeOptions &options);
};
}  // namespace rm_buff

#endif  // BUFF_TRACKER__TRACKER_NODE_HPP_
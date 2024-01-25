#include "buff_tracker/tracker_node.hpp"

namespace rm_buff {

BuffTrackerNode::BuffTrackerNode(const rclcpp::NodeOptions &options)
    : Node("tracker_node", options) {
  RCLCPP_INFO(this->get_logger(), "Tracker node initialized");
}
}  // namespace rm_buff

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_buff::BuffTrackerNode)
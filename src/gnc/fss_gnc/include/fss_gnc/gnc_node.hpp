#pragma once

#include <rclcpp/rclcpp.hpp>

namespace fss::gnc {

rclcpp::Node::SharedPtr make_gnc_node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

}  // namespace fss::gnc

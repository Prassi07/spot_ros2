// Copyright (c) 2024 Boston Dynamics, Inc.  All rights reserved.
//
// Downloading, reproducing, distributing or otherwise using the SDK Software
// is subject to the terms andconditions of the Boston Dynamics Software
// Development Kit License (20191101-BDSDK-SL).

#include <spot_driver/api/middleware_handle_base.hpp>
#include <spot_driver/local_grid/local_grid_middleware_handle.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/node.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace {
constexpr auto kPublisherHistoryDepth = 10;

}  

namespace spot_ros2 {

    LocalGridMiddlewareHandle::LocalGridMiddlewareHandle(const std::shared_ptr<rclcpp::Node>& node) : node_{node} {}

    LocalGridMiddlewareHandle::LocalGridMiddlewareHandle(const rclcpp::NodeOptions& node_options)
     : LocalGridMiddlewareHandle(std::make_shared<rclcpp::Node>("local_grid_publisher", node_options)) {}

    void LocalGridMiddlewareHandle::createPublishers(const std::vector<std::string>& grid_names){
        
        for (const auto& name : grid_names) {
            // For each name, create a publisher with a namespaced topic, e.g., "local_grid/terrain"
            grid_publishers_[name] = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("local_grid/" + name, 10);

            grid_publishers_.try_emplace(
                name, node_->create_publisher<nav_msgs::msg::OccupancyGrid>("local_grid/" + name, 
                                                                             makePublisherQoS(kPublisherHistoryDepth)));
        }
    }


    tl::expected<void, std::string> LocalGridMiddlewareHandle::publishSpecificOccupancyGrid(
         const std::string& grid_name, 
         nav_msgs::msg::OccupancyGrid::UniquePtr message) {

        try {
            grid_publishers_.at(grid_name)->publish(std::move(message));
        } catch (const std::out_of_range& e) {
            return tl::make_unexpected("No grid publisher exists for grid name `" + grid_name + "`.");
        }

        return {};
    } 

}  // namespace spot_ros2

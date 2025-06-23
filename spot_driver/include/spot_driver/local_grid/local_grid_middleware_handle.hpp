#pragma once

#include <rclcpp/node.hpp>
#include <spot_driver/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver/interfaces/rclcpp_parameter_interface.hpp>
#include <spot_driver/interfaces/rclcpp_wall_timer_interface.hpp>
#include <spot_driver/local_grid/local_grid_publisher.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <string>
#include <tl_expected/expected.hpp>
#include <unordered_map>

#include <memory>
#include <string>

namespace spot_ros2 {
/**
 * @brief An abstract interface for middleware-specific publishing of local grid data.
 *
 * This version is designed to be highly flexible, allowing for the creation and management
 * of multiple OccupancyGrid publishers that are configured at runtime.
 */
class LocalGridMiddlewareHandle : public LocalGridPublisher::MiddlewareHandle {
 public:
    /**
   * @brief Constructor for ImagesMiddlewareHandle
   *
   * @param node  A shared_ptr to an instance of a rclcpp::Node
   */
  explicit LocalGridMiddlewareHandle(const std::shared_ptr<rclcpp::Node>& node);

  /**
   * @brief Constructor for ImagesMiddlewareHandle which creates an instance of an rclcpp::node
   *
   * @param node_options Options for rclcpp::node initialization
   */
  explicit LocalGridMiddlewareHandle(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions{});

  
  /**
   * @brief Populates the grid_publishers_ with occupancy grid publishers.
   * @param image_sources Set of ImageSources. A publisher will be created for each ImageSource.
   */
  void createPublishers(const std::vector<std::string>& grid_names) override;

  /**
   * @brief Publish an OccupancyGrid message to a specific topic.
   *
   * The implementation will map the abstract `grid_name` to a concrete middleware topic.
   * For example, a `grid_name` of "terrain" might be published to the "/local_grid/terrain" topic.
   *
   * @param grid_name The abstract name of the grid to publish (e.g., "terrain", "DownsampledScanDots").
   * @param message A unique pointer to the message to be published.
   * @return True if a publisher existed for the given name and the message was published, false otherwise.
   */
  tl::expected<void, std::string> publishSpecificOccupancyGrid(
    const std::string& grid_name, 
    nav_msgs::msg::OccupancyGrid::UniquePtr message
  ) override;

  private:
  /** @brief Shared instance of an rclcpp node to create publishers */
  std::shared_ptr<rclcpp::Node> node_;

  /** @brief Map between grid topic names and image publishers. */
  std::unordered_map<std::string, std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>>> grid_publishers_;

};
}  // namespace spot_ros2
#pragma once

#include <spot_driver/api/spot_api.hpp>
#include <spot_driver/interfaces/logger_interface_base.hpp>
#include <spot_driver/interfaces/parameter_interface_base.hpp>
#include <spot_driver/local_grid/local_grid_middleware_handle.hpp>
#include <spot_driver/api/middleware_handle_base.hpp>

#include <memory>
#include <string>
#include <vector>
#include <rclcpp_wall_timer_interface.hpp>

namespace spot_ros2 {

/**
 * @brief Handles the core logic of requesting, processing, and publishing local grid data.
 *
 * This class is configured at runtime to fetch and publish a user-defined set of local grids,
 * plus an optional custom-downsampled grid.
 */
class LocalGridPublisher {
 public:
  
 /**
   * @brief A handle class around rclcpp::Node operations for SpotImagePublisher
   */
  class MiddlewareHandle : public MiddlewareHandleBase {
   public:
    virtual ~MiddlewareHandle() = default;

    virtual void createPublishers(const std::vector<std::string>& grid_names) = 0;

    virtual tl::expected<void, std::string> publishSpecificOccupancyGrid(
        const std::string& grid_name, 
        nav_msgs::msg::OccupancyGrid::UniquePtr message) = 0;
  };
 
 
 /**
   * @brief Constructor for LocalGridPublisher.
   * @param logger An interface for logging messages.
   * @param param_interface An interface for getting parameters from the ROS node.
   * @param api An interface for communicating with the Spot API.
   * @param middleware_handle An interface for publishing messages to the middleware.
   */
  LocalGridPublisher(const std::shared_ptr<LocalGridClientInterface>& local_grid_client_interface,
                     const std::shared_ptr<StateClientInterface>& state_client_interface,
                     std::unique_ptr<MiddlewareHandle> middleware_handle,
                     std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                     std::unique_ptr<LoggerInterfaceBase> logger_interface,
                     std::unique_ptr<TimerInterfaceBase> timer_interface);

  [[nodiscard]] bool initialize();

 private:
  /**
   * @brief Fetches data from the robot and publishes all configured grids.
   */
  void localGridTimerCallback();

  /**
   * @brief Computes the downsampled grid from last received high-resolution terrain grid.
   */
  void downsampledGridTimerCallback();

  /**
   * @brief Unpacks a LocalGrid's raw data into a more usable format based on its encoding.
   * @param local_grid_proto The protobuf message received from Spot.
   * @return A vector of bytes representing the decoded grid data.
   */
  std::vector<uint8_t> unpackGridData(const bosdyn::api::LocalGrid& local_grid_proto) const;

  /**
   * @brief Processes a single grid type into a ROS OccupancyGrid message.
   * @param grid_response The LocalGridResponse protobuf for a single grid type.
   * @return A unique_ptr to the generated nav_msgs::msg::OccupancyGrid. Returns nullptr on failure.
   */
  nav_msgs::msg::OccupancyGrid::UniquePtr processSingleGrid(const bosdyn::api::LocalGridResponse& grid_response) const;

  /**
   * @brief Processes the special 'terrain' grid, which requires combining it with the 'terrain_valid' grid.
   * @param terrain_grid The LocalGrid protobuf for the 'terrain' data.
   * @param valid_grid The LocalGrid protobuf for the 'terrain_valid' data.
   * @return A unique_ptr to the generated nav_msgs::msg::OccupancyGrid. Returns nullptr on failure.
   */
  nav_msgs::msg::OccupancyGrid::UniquePtr processTerrainGrid(const bosdyn::api::LocalGrid& terrain_grid,
                                                             const bosdyn::api::LocalGrid& valid_grid) const;

  std::unique_ptr<LoggerInterfaceBase> logger_;
  std::unique_ptr<ParameterInterfaceBase> param_interface_;
  std::unique_ptr<MiddlewareHandle> middleware_handle_;
  std::unique_ptr<TimerInterfaceBase> timer_interface_;

  std::shared_ptr<StateClientInterface> state_client_interface_;
  std::shared_ptr<LocalGridClientInterface> local_grid_client_interface_;

  // Parameters read from the ROS parameter server
  std::vector<std::string> standard_grids_to_publish_;
  std::set<SpotLocalGrid> grids_requested_;

  bool publish_scandots_;

  std::shared_ptr<nav_msgs::msg::OccupancyGrid> terrain_grid_data_;
};

}  // namespace spot_ros2

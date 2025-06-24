#pragma once

#include <spot_driver/api/spot_api.hpp>
#include <spot_driver/interfaces/logger_interface_base.hpp>
#include <spot_driver/interfaces/parameter_interface_base.hpp>
#include <spot_driver/api/middleware_handle_base.hpp>

#include <bosdyn/math/frame_helpers.h>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <memory>
#include <string>
#include <vector>
#include <optional>
#include <spot_driver/interfaces/rclcpp_wall_timer_interface.hpp>

namespace spot_ros2 {

struct ProcessedGridResult {
    nav_msgs::msg::OccupancyGrid::SharedPtr main_grid;
    std::optional<nav_msgs::msg::OccupancyGrid::UniquePtr> secondary_grid;
};

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
    
    virtual tl::expected<void, std::string> publishSpecificOccupancyGrid(
        const std::string& grid_name, 
        nav_msgs::msg::OccupancyGrid::SharedPtr message) = 0;
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
   * @param unpacked_data The unpacked data cast to float from the protobuf message based on encoding (Using float makes it resuable for different local grid types)
   */
  void unpackKnownGridData(const bosdyn::api::LocalGrid& local_grid_proto, std::vector<float>& unpacked_known_data) const;
  
  void unpackUnknownGridData(const bosdyn::api::LocalGrid& local_grid_proto, std::vector<uint8_t>& unpacked_unknown_data) const;


  /**
   * @brief Processes a single grid type into a ROS OccupancyGrid message.
   * @param grid_response The LocalGridResponse protobuf for a single grid type.
   * @return A unique_ptr to the generated nav_msgs::msg::OccupancyGrid.
   */
  nav_msgs::msg::OccupancyGrid::UniquePtr processNonTerrainGrid(const ::bosdyn::api::LocalGridResponse& grid_response) const;

  /**
   * @brief Processes the special 'terrain' grid, which requires combining it with the 'terrain_valid' grid.
   * @param terrain_grid The LocalGrid protobuf for the 'terrain' data.
   * @param valid_grid The LocalGrid protobuf for the 'terrain_valid' data.
   * @param processed_grids Updated ProcessedGridResult object that contains the terrain grid and the optional terrain_valid grid
  */
  void processTerrainGrid(const ::bosdyn::api::LocalGridResponse& terrain_grid,
                          const ::bosdyn::api::LocalGridResponse& valid_grid,
                          ProcessedGridResult& processed_grids) const;
  
  std::unique_ptr<LoggerInterfaceBase> logger_;
  std::unique_ptr<ParameterInterfaceBase> param_interface_;
  std::unique_ptr<MiddlewareHandle> middleware_handle_;
  std::shared_ptr<LocalGridClientInterface> local_grid_client_interface_;
  std::shared_ptr<StateClientInterface> state_client_interface_;
  std::unique_ptr<TimerInterfaceBase> timer_interface_;

  // Parameters read from the ROS parameter server
  std::vector<std::string> standard_grids_to_publish_, standard_grids_to_request_;
  std::set<SpotLocalGrid> grids_requested_;

  bool publish_scandots_, terrain_grid_initialized_, is_using_vision_;
  ::bosdyn::api::FrameTreeSnapshot tf_snapshot_;
  ::bosdyn::api::RobotState robot_state_;

  ::bosdyn::api::SE3Pose tf_body_pose_;

  std::string tf_root_, frame_prefix_, full_tf_root_id_;
  std::shared_ptr<nav_msgs::msg::OccupancyGrid> terrain_grid_data_;
};

}  // namespace spot_ros2

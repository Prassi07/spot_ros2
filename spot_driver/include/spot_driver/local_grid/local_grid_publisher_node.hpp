#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_options.hpp>

#include <spot_driver/api/default_spot_api.hpp>
#include <spot_driver/interfaces/rclcpp_node_interface.hpp>
#include <spot_driver/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver/interfaces/rclcpp_parameter_interface.hpp>
#include <spot_driver/interfaces/rclcpp_wall_timer_interface.hpp>
#include <spot_driver/local_grid/local_grid_publisher.hpp>


#include <memory>

namespace spot_ros2 {
/**
 * @brief The ROS2 node wrapper for the LocalGridPublisher.
 *
 * This class is a subclass of rclcpp::Node and is responsible for setting up the
 * ROS2-specific components (like parameters, timers, and publishers) and instantiating
 * the core LocalGridPublisher class to do the actual work.
 */
class LocalGridPublisherNode{
 public:
  
  LocalGridPublisherNode(std::unique_ptr<SpotApi> spot_api,
                         std::unique_ptr<LocalGridPublisher::MiddlewareHandle> mw_handle,
                         std::unique_ptr<ParameterInterfaceBase> parameters,
                         std::unique_ptr<LoggerInterfaceBase> logger,
                         std::unique_ptr<TimerInterfaceBase> timer,
                         std::unique_ptr<NodeInterfaceBase> node_base_interface);

  /**
   * @brief Constructor for StatePublisherNode.
   * @details This constructor creates an rclcpp::Node and rclcpp-specialized implementations of the middleware
   * interfaces.
   *
   * @param node_options node configuration options used when creating a rclcpp::Node
   */
  explicit LocalGridPublisherNode(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions{});


  /**
   * @brief Returns the NodeBaseInterface of this class's node.
   * @details This function exists to allow spinning the class's node as if it were derived from rclcpp::Node.
   * This allows loading this class as a component node in a composable node container.
   *
   * @return A shared_ptr to the NodeBaseInterface of the node stored as a private member of this class.
   */
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> get_node_base_interface();

 private:

  /**
   * @brief Connect to and authenticate with Spot, and then create the StatePublisher class member.
   *
   * @param node_base_interface Exposes the NodeBaseInterface of this class's rclcpp::Node so this class can be spun by
   * an rclcpp executor.
   * @param spot_api Connects to Spot and exposes interfaces to request data from it.
   * @param middleware_handle Publishes robot state info to the middleware.
   * @param parameter_interface Retrieves runtime configuration settings needed to connect to and communicate with Spot.
   * @param logger_interface Logs error messages if requesting, processing, and publishing the robot state info does not
   * succeed.
   * @param timer_interface Repeatedly triggers timerCallback() using the middleware's clock.
   *
   * @throw std::runtime_error if the Spot API fails to create a connection to Spot or fails to authenticate with Spot.
   */
  void initialize(std::unique_ptr<SpotApi> spot_api,
                    std::unique_ptr<LocalGridPublisher::MiddlewareHandle> middleware_handle,
                    std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                    std::unique_ptr<LoggerInterfaceBase> logger_interface,
                    std::unique_ptr<TimerInterfaceBase> timer_interface);

  std::unique_ptr<NodeInterfaceBase> node_base_interface_;
  std::unique_ptr<SpotApi> spot_api_;
  std::unique_ptr<LocalGridPublisher> internal_;

};

}  // namespace spot_ros2

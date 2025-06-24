#include <spot_driver/local_grid/local_grid_publisher_node.hpp>

#include <spot_driver/api/default_spot_api.hpp>

#include <spot_driver/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver/interfaces/rclcpp_node_interface.hpp>
#include <spot_driver/interfaces/rclcpp_parameter_interface.hpp>
#include <spot_driver/interfaces/rclcpp_wall_timer_interface.hpp>
#include <spot_driver/local_grid/local_grid_middleware_handle.hpp>

#include <memory>
#include <string>
#include <vector>


namespace {
constexpr auto kDefaultSDKName{"robot_local_grid_publisher_node"};
}

namespace spot_ros2 {

LocalGridPublisherNode::LocalGridPublisherNode(std::unique_ptr<SpotApi> spot_api,
                         std::unique_ptr<LocalGridPublisher::MiddlewareHandle> mw_handle,
                         std::unique_ptr<ParameterInterfaceBase> parameters,
                         std::unique_ptr<LoggerInterfaceBase> logger,
                         std::unique_ptr<TimerInterfaceBase> timer,
                         std::unique_ptr<NodeInterfaceBase> node_base_interface)
    : node_base_interface_{std::move(node_base_interface)} {
  initialize(std::move(spot_api), std::move(mw_handle), std::move(parameters),
             std::move(logger), std::move(timer));
}
LocalGridPublisherNode::LocalGridPublisherNode(const rclcpp::NodeOptions& node_options) {

const auto node = std::make_shared<rclcpp::Node>("state_publisher", node_options);
  node_base_interface_ = std::make_unique<RclcppNodeInterface>(node->get_node_base_interface());

  auto mw_handle = std::make_unique<LocalGridMiddlewareHandle>(node);
  auto parameter_interface = std::make_unique<RclcppParameterInterface>(node);
  auto logger_interface = std::make_unique<RclcppLoggerInterface>(node->get_logger());
  auto timer_interface = std::make_unique<RclcppWallTimerInterface>(node);

  const auto timesync_timeout = parameter_interface->getTimeSyncTimeout();

  auto spot_api = std::make_unique<DefaultSpotApi>(kDefaultSDKName, timesync_timeout, parameter_interface->getCertificate());

  initialize(std::move(spot_api), std::move(mw_handle), std::move(parameter_interface),
             std::move(logger_interface), std::move(timer_interface));

}

void LocalGridPublisherNode::initialize(std::unique_ptr<SpotApi> spot_api,
                                        std::unique_ptr<LocalGridPublisher::MiddlewareHandle> mw_handle,
                                        std::unique_ptr<ParameterInterfaceBase> parameters,
                                        std::unique_ptr<LoggerInterfaceBase> logger,
                                        std::unique_ptr<TimerInterfaceBase> timer) {
  spot_api_ = std::move(spot_api);

  const auto hostname = parameters->getHostname();
  const auto port = parameters->getPort();
  const auto username = parameters->getUsername();
  const auto password = parameters->getPassword();
  const std::string frame_prefix = parameters->getFramePrefixWithDefaultFallback();

  // create and authenticate robot
  if (const auto create_robot_result = spot_api_->createRobot(hostname, port, frame_prefix); !create_robot_result) {
    const auto error_msg{std::string{"Failed to create interface to robot: "}.append(create_robot_result.error())};
    logger->logError(error_msg);
    throw std::runtime_error(error_msg);
  }

  if (const auto authentication_result = spot_api_->authenticate(username, password); !authentication_result) {
    const auto error_msg{std::string{"Failed to authenticate robot: "}.append(authentication_result.error())};
    logger->logError(error_msg);
    throw std::runtime_error(error_msg);
  }

  internal_ = std::make_unique<LocalGridPublisher>(spot_api_->localGridClientInterface(), spot_api_->stateClientInterface(),
                                                   std::move(mw_handle), std::move(parameters),
                                                   std::move(logger), std::move(timer));
                                                   
  if (!internal_->initialize()) {
    constexpr auto error_msg{"Failed to initialize Local Grid publisher."};
    logger->logError(error_msg);
    throw std::runtime_error(error_msg);
  }
}

std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> LocalGridPublisherNode::get_node_base_interface() {
  return node_base_interface_->getNodeBaseInterface();
}


}  // namespace spot_ros2

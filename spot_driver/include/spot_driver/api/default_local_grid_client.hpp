#pragma once

#include <bosdyn/client/local_grid/local_grid_client.h>
#include <spot_driver/api/local_grid_client_interface.hpp>
#include <memory>

namespace spot_ros2 {
/**
 * @brief The default implementation of the LocalGridClientInterface.
 *
 * This class wraps the Boston Dynamics C++ SDK's LocalGridClient, translating simple
 * C++ types into the required protobuf formats and handling the result.
 */
class DefaultLocalGridClient : public LocalGridClientInterface {
 public:
  /**
   * @brief Constructor for DefaultLocalGridClient.
   * @param client A pointer to the Boston Dynamics SDK's LocalGridClient.
   */
  explicit DefaultLocalGridClient(::bosdyn::client::LocalGridClient* client);

  /**
   * @brief Override of the interface function to get local grids.
   * This now correctly returns a tl::expected object to match the interface.
   */
  tl::expected<bosdyn::api::GetLocalGridsResponse, std::string> getLocalGrids(
      const std::vector<std::string>& grid_type_names) override;

 private:
  // A pointer to the underlying Boston Dynamics SDK client.
  ::bosdyn::client::LocalGridClient* client_;
};
}  // namespace spot_ros2

#pragma once

#include <bosdyn/api/local_grid.pb.h>
#include <tl_expected/expected.hpp> 
#include <string>
#include <vector>

namespace spot_ros2 {

/**
 * @brief Abstract interface defining how to get local grid data from the Spot API.
 *
 * This class defines a clean, high-level contract for a client that fetches local grid data,
 * hiding the underlying gRPC and protobuf complexity. It is non-copyable and non-movable
 * to prevent misuse of the underlying API connection.
 */
class LocalGridClientInterface {
 public:
  /**
   * @brief Fetches local grids from the robot.
   * @param grid_type_names A list of strings identifying which grid types to request.
   * @return A tl::expected object containing either the GetLocalGridsResponse protobuf message on success,
   * or a std::string with an error message on failure.
   */
  virtual tl::expected<bosdyn::api::GetLocalGridsResponse, std::string> getLocalGrids(
      const std::vector<std::string>& grid_type_names) = 0;

  // Forbid copying and moving to ensure a single, unambiguous owner for the API connection.
  LocalGridClientInterface(const LocalGridClientInterface&) = delete;
  LocalGridClientInterface& operator=(const LocalGridClientInterface&) = delete;
  LocalGridClientInterface(LocalGridClientInterface&&) = delete;
  LocalGridClientInterface& operator=(LocalGridClientInterface&&) = delete;

 protected:
  // Default constructor for use by derived classes.
  LocalGridClientInterface() = default;
  // Default virtual destructor.
  virtual ~LocalGridClientInterface() = default;
};

}  // namespace spot_ros2

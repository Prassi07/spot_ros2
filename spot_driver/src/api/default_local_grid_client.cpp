#include <spot_driver/api/default_local_grid_client.hpp>
#include <bosdyn/api/local_grid.pb.h>

namespace spot_ros2 {

    DefaultLocalGridClient::DefaultLocalGridClient(::bosdyn::client::LocalGridClient* client) : client_(client) {}

    tl::expected<bosdyn::api::GetLocalGridsResponse, std::string> DefaultLocalGridClient::getLocalGrids(const std::vector<std::string>& grid_type_names)
    {

        // Now, call the SDK client's method with the fully-formed request object.
        // The SDK handles all the underlying gRPC details.
        auto result = client_->GetLocalGrids(grid_type_names);

        if (!result) {
            // If the call failed, extract the error message from the SDK's result object
            // and return it inside our own error-state tl::expected object.
            return tl::make_unexpected("The GetLocalGrids service returned with error code " +
                               std::to_string(result.status.code().value()) + ": " + result.status.message());
        }

        // If the call succeeded, return a success-state tl::expected object containing the protobuf response data.
        return result.response;
    }

}  // namespace spot_ros2

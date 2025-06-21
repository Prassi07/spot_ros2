// Copyright (c) 2024 Boston Dynamics, Inc.  All rights reserved.
//
// Downloading, reproducing, distributing or otherwise using the SDK Software
// is subject to the terms and conditions of the Boston Dynamics Software
// Development Kit License (20191101-BDSDK-SL).

#include <spot_driver/local_grid/local_grid_publisher.hpp>
#include <spot_driver/conversions/common_conversions.hpp>
#include <spot_driver/conversions/geometry.hpp>
#include <spot_driver/conversions/time.hpp>

#include <algorithm>
#include <cmath>

namespace {
constexpr auto kLocalGridCallbackPeriod = std::chrono::duration<double>{1.0 / 30.0};  // 30 Hz
}

namespace spot_ros2 {

// LocalGridPublisher Implementation
LocalGridPublisher::LocalGridPublisher(
    const std::shared_ptr<LocalGridClientInterface>& local_grid_client_interface,
    const std::shared_ptr<StateClientInterface>& state_client_interface,
    std::unique_ptr<MiddlewareHandle> middleware_handle,
    std::unique_ptr<ParameterInterfaceBase> parameter_interface,
    std::unique_ptr<LoggerInterfaceBase> logger_interface,
    std::unique_ptr<TimerInterfaceBase> timer_interface)
    : logger_{std::move(logger_interface)},
      param_interface_{std::move(parameter_interface)},
      middleware_handle_{std::move(middleware_handle)},
      local_grid_client_interface_{local_grid_client_interface},
      state_client_interface_{state_client_interface},
      timer_interface_{std::move(timer_interface)} {}

bool LocalGridPublisher::initialize() {
  // Read configuration from the parameter interface.
  // standard_grids_to_publish_ = param_interface_->getGridNames();
  // publish_downsampled_grid_ = param_interface_->getPublishDownsampledGrid();
  logger_->logInfo("LocalGridPublisher initialized.");

  // Create publishers for all the grid topics we need.
  auto all_grids = standard_grids_to_publish_;
  
  if (publish_downsampled_grid_) {
    all_grids.push_back("DownsampledScanDots");
  }

  middleware_handle_->createPublishers(all_grids);

  // Create a timer to publish local grids
  timer_interface_->setTimer(kLocalGridCallbackPeriod, [this]() {
    timerCallback();
  });

  return true;
}

// void LocalGridPublisher::timerCallback() {
//   // Build the list of all grids we need to request from the robot.
//   auto grids_to_request = standard_grids_to_publish_;
//   // The 'terrain' grid requires 'terrain_valid' for processing, so we must request it as well.
//   if (std::find(grids_to_request.begin(), grids_to_request.end(), "terrain") != grids_to_request.end()) {
//     if (std::find(grids_to_request.begin(), grids_to_request.end(), "terrain_valid") == grids_to_request.end()) {
//         grids_to_request.push_back("terrain_valid");
//     }
//   }

//   if (grids_to_request.empty()) {
//     return;
//   }

//   // Make a single API call to get all requested grids.
//   const auto result = local_grid_client_interface_->getLocalGrids(grids_to_request);
//   if (!result) {
//     logger_->logError("Failed to get local grids: " + result.error());
//     return;
//   }

//   const auto& responses = result.value().local_grid_responses();
//   nav_msgs::msg::OccupancyGrid::SharedPtr terrain_msg_for_downsample = nullptr;

//   // Handle the special case of the 'terrain' grid first.
//   if (std::find(standard_grids_to_publish_.begin(), standard_grids_to_publish_.end(), "terrain") !=
//       standard_grids_to_publish_.end()) {
//     const auto* terrain_response =
//         std::find_if(responses.cbegin(), responses.cend(), [](const auto& r) { return r.local_grid_type_name() == "terrain"; });
//     const auto* valid_response = std::find_if(
//         responses.cbegin(), responses.cend(), [](const auto& r) { return r.local_grid_type_name() == "terrain_valid"; });

//     if (terrain_response != responses.cend() && valid_response != responses.cend()) {
//       auto terrain_occ_msg = processTerrainGrid(terrain_response->local_grid(), valid_response->local_grid());
//       if (terrain_occ_msg) {
//         // Keep a shared_ptr to this message in case we need it for downsampling.
//         terrain_msg_for_downsample = terrain_occ_msg;
//         middleware_handle_->publishSpecificOccupancyGrid("terrain", std::move(terrain_occ_msg));
//       }
//     } else {
//       logger_->logWarn("Requested 'terrain' grid but did not receive both 'terrain' and 'terrain_valid' from robot.");
//     }
//   }

//   // Process all other standard grid types.
//   for (const auto& response : responses) {
//     const auto& name = response.local_grid_type_name();
//     // Skip 'terrain' and 'terrain_valid' since we've already handled them.
//     if (name == "terrain" || name == "terrain_valid") {
//       continue;
//     }

//     auto occ_msg = processSingleGrid(response);
//     if (occ_msg) {
//       middleware_handle_->publishSpecificOccupancyGrid(name, std::move(occ_msg));
//     }
//   }

//   // Handle the custom downsampled grid if enabled and we have a valid source grid.
//   if (publish_downsampled_grid_ && terrain_msg_for_downsample) {
//     publishDownsampledGrid(*terrain_msg_for_downsample);
//   }
// }

// std::vector<uint8_t> LocalGridPublisher::unpackGridData(const bosdyn::api::LocalGrid& local_grid_proto) const {
//   // This function can be expanded to handle RLE, but for now we assume RAW encoding
//   // as it is most common for the int16 and uint8 grids.
//   if (local_grid_proto.encoding() == bosdyn::api::LocalGrid_Encoding_ENCODING_RAW) {
//     const auto& raw_data = local_grid_proto.data();
//     return {raw_data.begin(), raw_data.end()};
//   }
//   logger_->logWarn("Grid unpacking for RLE encoding is not yet implemented.");
//   return {};
// }

// nav_msgs::msg::OccupancyGrid::UniquePtr LocalGridPublisher::processSingleGrid(
//     const bosdyn::api::LocalGridResponse& grid_response) const {
//   // This is a simplified processor for non-terrain grids.
//   // The exact conversion from raw data to 0-100 occupancy values depends on the grid type.
//   // This placeholder assumes a direct mapping, which may need adjustment.
//   auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
//   const auto& grid_proto = grid_response.local_grid();

//   // Populate header and info (same as terrain grid)
//   msg->header.stamp = toRosTime(grid_proto.acquisition_time());
//   msg->header.frame_id = grid_proto.frame_name_local_grid_data();
//   msg->info.map_load_time = msg->header.stamp;
//   msg->info.resolution = grid_proto.extent().cell_size();
//   msg->info.width = grid_proto.extent().num_cells_x();
//   msg->info.height = grid_proto.extent().num_cells_y();
//   const auto& extent = grid_proto.extent();
//   msg->info.origin.position.x = extent.cell_size() * (0.5 - extent.num_cells_x() / 2.0);
//   msg->info.origin.position.y = extent.cell_size() * (0.5 - extent.num_cells_y() / 2.0);

//   auto unpacked_data = unpackGridData(grid_proto);
//   if (unpacked_data.empty()) {
//     return nullptr;
//   }

//   // Convert to int8_t for the final message. This is a naive conversion.
//   // Real implementation would need to know the meaning of the data for each grid type.
//   msg->data.assign(unpacked_data.begin(), unpacked_data.end());

//   return msg;
// }

// nav_msgs::msg::OccupancyGrid::UniquePtr LocalGridPublisher::processTerrainGrid(
//     const bosdyn::api::LocalGrid& terrain_grid, const bosdyn::api::LocalGrid& valid_grid) const {
//   if (terrain_grid.extent().num_cells_x() != valid_grid.extent().num_cells_x() ||
//       terrain_grid.extent().num_cells_y() != valid_grid.extent().num_cells_y()) {
//     logger_->logError("Mismatch in dimensions between 'terrain' and 'terrain_valid' grids.");
//     return nullptr;
//   }

//   auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
//   msg->header.stamp = toRosTime(terrain_grid.acquisition_time());
//   msg->header.frame_id = terrain_grid.frame_name_local_grid_data();
//   msg->info.map_load_time = msg->header.stamp;
//   msg->info.resolution = terrain_grid.extent().cell_size();
//   msg->info.width = terrain_grid.extent().num_cells_x();
//   msg->info.height = terrain_grid.extent().num_cells_y();
//   const auto& extent = terrain_grid.extent();
//   msg->info.origin.position.x = extent.cell_size() * (0.5 - extent.num_cells_x() / 2.0);
//   msg->info.origin.position.y = extent.cell_size() * (0.5 - extent.num_cells_y() / 2.0);

//   auto terrain_data_unpacked = unpackGridData(terrain_grid);
//   auto valid_data_unpacked = unpackGridData(valid_grid);

//   if (terrain_data_unpacked.size() / 2 != valid_data_unpacked.size()) {
//     logger_->logError("Mismatch in unpacked data size between 'terrain' (int16) and 'terrain_valid' (uint8).");
//     return nullptr;
//   }

//   std::vector<int16_t> terrain_data(terrain_data_unpacked.size() / 2);
//   memcpy(terrain_data.data(), terrain_data_unpacked.data(), terrain_data_unpacked.size());

//   msg->data.reserve(valid_data_unpacked.size());
//   for (size_t i = 0; i < valid_data_unpacked.size(); ++i) {
//     if (valid_data_unpacked[i] == 0) {
//       msg->data.push_back(-1);  // Unknown
//     } else {
//       // Simple logic: if terrain is not flat (value != 0), it's occupied.
//       if (terrain_data[i] != 0) {
//         msg->data.push_back(100);  // Occupied
//       } else {
//         msg->data.push_back(0);  // Free
//       }
//     }
//   }
//   return msg;
// }

void LocalGridPublisher::publishDownsampledGrid(const nav_msgs::msg::OccupancyGrid& full_grid) {
  logger_->logDebug("Downsampled grid publishing is not yet implemented.");
  // TODO:
  // 1. Create a nav_msgs::msg::OccupancyGrid::UniquePtr.
  auto downsampled_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
  // 2. Set its header and info fields.
  downsampled_msg->header = full_grid.header;
  //     Example: resolution is doubled, width/height are halved.
  // 3. Implement your downsampling logic.
  //    (e.g., iterate through the full_grid.data in 2x2 blocks and find max value)
  // 4. Set downsampled_msg->data with the result.
  // 5. Publish: middleware_handle_->publishSpecificOccupancyGrid("DownsampledScanDots", std::move(downsampled_msg));
}

}  // namespace spot_ros2

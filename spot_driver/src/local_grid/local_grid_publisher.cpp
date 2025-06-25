// Copyright (c) 2024 Boston Dynamics, Inc.  All rights reserved.
//
// Downloading, reproducing, distributing or otherwise using the SDK Software
// is subject to the terms and conditions of the Boston Dynamics Software
// Development Kit License (20191101-BDSDK-SL).

#include <spot_driver/local_grid/local_grid_publisher.hpp>
#include <spot_driver/conversions/common_conversions.hpp>
#include <spot_driver/conversions/geometry.hpp>
#include <spot_driver/conversions/time.hpp>
#include "bosdyn/math/proto_math.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>

namespace {
constexpr auto kLocalGridCallbackPeriod = std::chrono::duration<double>{1.0 / 30.0};  // 30 Hz
constexpr auto kDownSampledGridCallbackPeriod = std::chrono::duration<double>{1.0 / 50.0}; // 50 Hz
struct HeightModeTracker {
    int modal_height = 0;
    int max_count = 0;
    std::map<int, int> histogram;
};
}

namespace spot_ros2 {

// LocalGridPublisher Implementation
LocalGridPublisher::LocalGridPublisher(
    const std::shared_ptr<LocalGridClientInterface>& local_grid_client_interface,
    const std::shared_ptr<StateClientInterface>& state_client_interface,
    std::unique_ptr<MiddlewareHandle> middleware_handle,
    std::unique_ptr<ParameterInterfaceBase> parameter_interface,
    std::unique_ptr<LoggerInterfaceBase> logger_interface,
    std::unique_ptr<TimerInterfaceBase> timer1_interface,
    std::unique_ptr<TimerInterfaceBase> timer2_interface)
    : logger_{std::move(logger_interface)},
      param_interface_{std::move(parameter_interface)},
      middleware_handle_{std::move(middleware_handle)},
      local_grid_client_interface_{local_grid_client_interface},
      state_client_interface_{state_client_interface},
      timer_interface_main_{std::move(timer1_interface)},
      timer_interface_second_{std::move(timer2_interface)} {}

bool LocalGridPublisher::initialize() {
  
  // Read configuration from the parameter interface.
  const auto grids_parameter = param_interface_->getLocalGridsUsed();
  
  if(grids_parameter.has_value()){
    grids_requested_ = grids_parameter.value();
  }
  else{
    logger_->logWarn("Invalid local_grid_names parameter! Got error:" + grids_parameter.error() + 
                      " Defaulting to obstacle_distance" );
    grids_requested_.insert(SpotLocalGrid::OBSTACLE_DISTANCE);                
  }

  publish_scandots_ = param_interface_->getPublishScanDots();

  tf_root_ = param_interface_->getTFRoot();
  frame_prefix_ = param_interface_->getFramePrefixWithDefaultFallback();
  is_using_vision_ = param_interface_->getPreferredOdomFrame() == "vision";

  full_tf_root_id_ = frame_prefix_ + tf_root_;

  logger_->logInfo("LocalGridPublisher initialized.");

  for(const auto& grid: grids_requested_){
    if (grid == SpotLocalGrid::TERRAIN){
      standard_grids_to_publish_.push_back("terrain");
      standard_grids_to_request_.push_back("terrain");

      if(std::find(standard_grids_to_request_.begin(), standard_grids_to_request_.end(), "terrain_valid") == standard_grids_to_request_.end()){
        standard_grids_to_request_.push_back("terrain_valid");
      }
    }
    else if(grid == SpotLocalGrid::TERRAIN_INTENSITY){
      standard_grids_to_publish_.push_back("intensity");
      standard_grids_to_request_.push_back("intensity");
    }
    else if(grid == SpotLocalGrid::TERRAIN_VALID){
      standard_grids_to_publish_.push_back("terrain_valid");
      if(std::find(standard_grids_to_request_.begin(), standard_grids_to_request_.end(), "terrain_valid") == standard_grids_to_request_.end()){
        standard_grids_to_request_.push_back("terrain_valid");
      }
    }
    else if(grid == SpotLocalGrid::NO_STEP){
      standard_grids_to_publish_.push_back("no_step");
      standard_grids_to_request_.push_back("no_step");
    }
    else if(grid == SpotLocalGrid::OBSTACLE_DISTANCE){
      standard_grids_to_publish_.push_back("obstacle_distance");
      standard_grids_to_request_.push_back("obstacle_distance");
    }
  }
  // Create publishers for all the grid topics we need.
  if (publish_scandots_) {
    standard_grids_to_publish_.push_back("scandots");
  }

  middleware_handle_->createPublishers(standard_grids_to_publish_);

  terrain_grid_initialized_ = false;
  terrain_grid_data_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();

  // Create a timer to publish local grids
  timer_interface_main_->setTimer(kLocalGridCallbackPeriod, [this]() {
    localGridTimerCallback();
  });

  if (publish_scandots_) {
    timer_interface_second_->setTimer(kDownSampledGridCallbackPeriod, [this]() {
      downsampledGridTimerCallback();
    });
  }

  return true;
}

void LocalGridPublisher::localGridTimerCallback() {
  // Step 1: Build the list of required grids for this cycle.
  auto grids_to_request = standard_grids_to_request_;

  if (std::find(grids_to_request.begin(), grids_to_request.end(), "terrain") != grids_to_request.end() &&
      std::find(grids_to_request.begin(), grids_to_request.end(), "terrain_valid") == grids_to_request.end()) {
      grids_to_request.push_back("terrain_valid");
  }
  if (grids_to_request.empty()) {
      return;
  }

  // Step 2: Make API calls to get grids and the latest robot state.
  const auto result = local_grid_client_interface_->getLocalGrids(grids_to_request);
  const auto state_result = state_client_interface_->getRobotState();

  if (!result) {
      logger_->logError("Failed to get local grids: " + result.error());
      return;
  }
  if (!state_result) {
      logger_->logError("Failed to get robot state for local grids: " + state_result.error());
      return;
  }

  const auto& responses = result.value().local_grid_responses();
  if (result.value().num_local_grid_errors() != 0) {
      logger_->logWarn("Failed to get some of the local grids!");
  }

  // Step 3: Get the latest robot pose from the new state.
  if(state_result.value().has_kinematic_state()){
    tf_snapshot_ = state_result.value().kinematic_state().transforms_snapshot();

    if(is_using_vision_){
      if (!::bosdyn::api::GetWorldTformBody(tf_snapshot_, &tf_body_pose_)) {
        logger_->logWarn("Failed to get robot odometry info, incorrect downsampled grid");
      }
    }
    else {
      if (!::bosdyn::api::GetOdomTformBody(tf_snapshot_, &tf_body_pose_)) {
        logger_->logWarn("Failed to get robot odometry info, incorrect downsampled grid");
      }
    }
  }

  
  // Step 4: Check if the 'terrain' grid was requested and process it first.
    // This is a special case because it needs two source grids ('terrain' and 'terrain_valid').
  bool terrain_was_requested = std::find(standard_grids_to_request_.begin(), standard_grids_to_request_.end(), "terrain") != standard_grids_to_request_.end();
  if (terrain_was_requested) {

      auto terrain_it = std::find_if(responses.cbegin(), responses.cend(), 
          [](const auto& r) { return r.local_grid_type_name() == "terrain"; });

      auto valid_it = std::find_if(responses.cbegin(), responses.cend(), 
          [](const auto& r) { return r.local_grid_type_name() == "terrain_valid"; });

      if (terrain_it != responses.cend() && valid_it != responses.cend()) {
          ProcessedGridResult terrain_grids;
          processTerrainGrid(*terrain_it, *valid_it, terrain_grids);
          
          // Store the main grid for the downsampler and publish it.
          if (terrain_grids.main_grid) {
              terrain_grid_data_ = terrain_grids.main_grid;    
              middleware_handle_->publishSpecificOccupancyGrid("terrain", terrain_grids.main_grid);
              terrain_grid_initialized_ = true;
          }

          // Publish the secondary grid (terrain_valid) if it was created.
          if (terrain_grids.secondary_grid.has_value()) {
              middleware_handle_->publishSpecificOccupancyGrid("terrain_valid", std::move(terrain_grids.secondary_grid.value()));
          }
      } else {
          logger_->logWarn("Requested 'terrain' grid but did not receive both 'terrain' and 'terrain_valid' from robot.");
      }
    }

  // Step 5: Loop through all responses and process any remaining (non-terrain) grids.
  for (const ::bosdyn::api::LocalGridResponse& response : responses) {
    const auto& name = response.local_grid_type_name();
    // Skip grids that were already handled in the special terrain-processing block.
    if (name == "terrain" || (name == "terrain_valid" && terrain_was_requested)) {
        continue;
    }

    if (response.status() != ::bosdyn::api::LocalGridResponse_Status::LocalGridResponse_Status_STATUS_OK) {
        logger_->logError("No data received for local_grid with name: " + name);
        continue;
    }

    auto occ_msg = processNonTerrainGrid(response);
    if (occ_msg) {
        middleware_handle_->publishSpecificOccupancyGrid(name, std::move(occ_msg));
    }
  }
}

void LocalGridPublisher::unpackKnownGridData(const bosdyn::api::LocalGrid& local_grid_proto, std::vector<float>& unpacked_known_data) const {

  std::string bytes_to_process = local_grid_proto.data();
  std::vector<float> initial_unpacked_data;
  
  // Convert bytes to different data types based on format, and then finally cast it to float.
  const auto cell_format = local_grid_proto.cell_format();

  switch(cell_format) {
      case bosdyn::api::LocalGrid::CELL_FORMAT_FLOAT32: {
        if (bytes_to_process.size() % sizeof(float) != 0) 
          return;
        initial_unpacked_data.reserve(bytes_to_process.size() / sizeof(float));
        for(size_t i=0; i < bytes_to_process.size(); i += sizeof(float)) {
            float val;
            memcpy(&val, &bytes_to_process[i], sizeof(float));
            initial_unpacked_data.push_back(static_cast<double>(val));
        }
        break;
      }
      case bosdyn::api::LocalGrid::CELL_FORMAT_FLOAT64: {
        if (bytes_to_process.size() % sizeof(double) != 0) 
          return;
        initial_unpacked_data.reserve(bytes_to_process.size() / sizeof(double));
        for(size_t i=0; i < bytes_to_process.size(); i += sizeof(double)) {
            double val;
            memcpy(&val, &bytes_to_process[i], sizeof(double));
            initial_unpacked_data.push_back(val);
        }
        break;
      }
      case bosdyn::api::LocalGrid::CELL_FORMAT_INT16: {
        if (bytes_to_process.size() % sizeof(int16_t) != 0) 
          return;
        initial_unpacked_data.reserve(bytes_to_process.size() / sizeof(int16_t));
        for(size_t i=0; i < bytes_to_process.size(); i += sizeof(int16_t)) {
            int16_t val;
            memcpy(&val, &bytes_to_process[i], sizeof(int16_t));
            initial_unpacked_data.push_back((static_cast<double>(val)));
        }
        break;
      }
      case bosdyn::api::LocalGrid::CELL_FORMAT_UINT16: {
        if (bytes_to_process.size() % sizeof(uint16_t) != 0) 
          return;
        initial_unpacked_data.reserve(bytes_to_process.size() / sizeof(uint16_t));
        for(size_t i=0; i < bytes_to_process.size(); i += sizeof(uint16_t)) {
            uint16_t val;
            memcpy(&val, &bytes_to_process[i], sizeof(uint16_t));
            initial_unpacked_data.push_back(static_cast<double>(val));
        }
        break;
      }
      case bosdyn::api::LocalGrid::CELL_FORMAT_INT8: {
        initial_unpacked_data.reserve(bytes_to_process.size());
        for(char val : bytes_to_process) {
            initial_unpacked_data.push_back(static_cast<double>(static_cast<int8_t>(val)));
        }
        break;
      }
      case bosdyn::api::LocalGrid::CELL_FORMAT_UINT8: {
        initial_unpacked_data.reserve(bytes_to_process.size());
        for(unsigned char val : bytes_to_process) {
            initial_unpacked_data.push_back(static_cast<double>(val));
        }
        break;
      }
      default: {
          logger_->logWarn("Unhandled cell format in unpackGridData.");
          break;
      }
  }
  
  const double scale_from_proto = local_grid_proto.cell_value_scale();
  const double scale_to_use = (std::abs(scale_from_proto) > 1e-9) ? scale_from_proto : 1.0;
  const double offset_to_use = local_grid_proto.cell_value_offset();

  unpacked_known_data.clear();

  if (local_grid_proto.encoding() == bosdyn::api::LocalGrid_Encoding_ENCODING_RLE) {
        const auto& rle_counts = local_grid_proto.rle_counts();
        if (initial_unpacked_data.size() != static_cast<size_t>(rle_counts.size())) {
            logger_->logError("RLE data and counts have mismatched sizes.");
            unpacked_known_data.clear();
            return;
        }

        // Pre-calculate total size to reserve memory once, preventing multiple reallocations.
        size_t total_cells = 0;
        for (const auto& count : rle_counts) {
            total_cells += count;
        }
        
        unpacked_known_data.reserve(total_cells);

        // Decode the RLE data using the parallel counts array.
        for (size_t i = 0; i < initial_unpacked_data.size(); ++i) {
          float val = (initial_unpacked_data[i] * scale_to_use) + offset_to_use;
          unpacked_known_data.insert(unpacked_known_data.end(), rle_counts.Get(i), val);
        }

    } else { // RAW encoding
        unpacked_known_data.reserve(initial_unpacked_data.size());
        // Decode the RLE data using the parallel counts array.
        for (size_t i = 0; i < initial_unpacked_data.size(); ++i) {
          float val = (initial_unpacked_data[i] * scale_to_use) + offset_to_use;
          unpacked_known_data.push_back(val);
        }
    }

}

void LocalGridPublisher::unpackUnknownGridData(const bosdyn::api::LocalGrid& local_grid_proto, std::vector<uint8_t>& unpacked_unknown_data) const {

  std::string unknown_cells_mask =  local_grid_proto.unknown_cells();
  unpacked_unknown_data.clear();
  int unknown_count = 0;
  for (size_t i = 0; i < unknown_cells_mask.size(); ++i) {
    // The mask uses a uint8_t for each cell. A value of 1 means the cell is unknown.
    unpacked_unknown_data.push_back(static_cast<uint8_t>(unknown_cells_mask[i]));
    if (unpacked_unknown_data[i] == 1) {
      ++unknown_count;
    }
  }
  
  // if (unknown_count > 0){
  //   logger_->logWarn("Unknown Found for grid: " + local_grid_proto.local_grid_type_name() + " Count: " + std::to_string(unknown_count));
  // }
}

nav_msgs::msg::OccupancyGrid::UniquePtr LocalGridPublisher::processNonTerrainGrid(
    const ::bosdyn::api::LocalGridResponse& grid_response) const {
 
  auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
  const ::bosdyn::api::LocalGrid& grid_proto = grid_response.local_grid();

  // Populate header and info (same as terrain grid)
  msg->header.stamp.sec =  grid_proto.acquisition_time().seconds();
  msg->header.stamp.nanosec = grid_proto.acquisition_time().nanos();

  msg->info.map_load_time = msg->header.stamp;
  msg->info.resolution = grid_proto.extent().cell_size();
  msg->info.width = grid_proto.extent().num_cells_x();
  msg->info.height = grid_proto.extent().num_cells_y();
  msg->header.frame_id = full_tf_root_id_;

  std::string grid_frame_id = grid_proto.frame_name_local_grid_data();

  ::bosdyn::api::SE3Pose transform, ground_plane_tf; 
  if(::bosdyn::api::get_a_tform_b(grid_proto.transforms_snapshot(), tf_root_, grid_frame_id, &transform)){
    if(::bosdyn::api::get_a_tform_b(tf_snapshot_, tf_root_, ::bosdyn::api::kGroundPlaneEstimateFrame, &ground_plane_tf)){
      transform.mutable_position()->set_z(ground_plane_tf.position().z());
    }
    else{
      logger_->logWarn("Error while getting transform to ground plane to tf_root");
    }
  }
  else{
    logger_->logError("Failed while getting transform to tf_root");
  }

  geometry_msgs::msg::Pose rosPose;
  convertToRos(transform, rosPose);

  msg->info.origin = rosPose;

  std::vector<float> unpacked_known_data;
  std::vector<uint8_t> unpacked_unknown_data;
  unpackKnownGridData(grid_proto, unpacked_known_data); 
  unpackUnknownGridData(grid_proto, unpacked_unknown_data);

  // Different scaling based on terrain grid type
  if(grid_proto.local_grid_type_name() == "obstacle_distance"){
    for(size_t i = 0; i < unpacked_known_data.size(); ++i){
      // The scales distance range of -2.0 to 2.0 -> -100 to 100
      // Negative disatnce means inside obstacle, which is set to 0
      // This is a linear scaling: (val / 2.0) * 127.0
      // We also clamp the value to ensure it's within the valid range.
      // Unknown is set to -128.
      bool known = unpacked_unknown_data[i] == 0;
      if(known){
        float val = unpacked_known_data[i];
        const double scaled_value = (val / 2.0) * 100.0;
        msg->data.push_back(static_cast<int8_t>(std::max(-100.0, std::min(100.0, scaled_value))));
      }else{
        msg->data.push_back(-128);
      }
    }
  }
  else if(grid_proto.local_grid_type_name() == "no_step"){
    for (const float val : unpacked_known_data) {
          // The transforms no_step 0 to 0, non-zero to 100
          // Positive means can step 
          // Negative means cannot step (obstacle)
          // Using ROS convention as 100 is obstacle, and free-space is 0 
          if (val > 1e-6) {
             msg->data.push_back(0);
          } else {
             msg->data.push_back(100);
          }
    }
  }
  else if(grid_proto.local_grid_type_name() == "intensity"){
    // Intensity values range from 0 to 255, we move this to -128 to 127 as ros using int8
    for (const float val : unpacked_known_data) {
      msg->data.push_back(static_cast<int8_t>(std::max(-128.0, std::min(127.0, val - 128.0))));
    }
  }
  else if(grid_proto.local_grid_type_name() == "terrain_valid"){
    // Terrain Validity 0 and 1 are scaled to 0 and 100
    for (const float val : unpacked_known_data) {
      msg->data.push_back(static_cast<int8_t>(val * 100));
    }
  }
  return msg;
}

void LocalGridPublisher::processTerrainGrid(const ::bosdyn::api::LocalGridResponse& terrain_grid,
                                             const ::bosdyn::api::LocalGridResponse& valid_grid,
                                             ProcessedGridResult& processed_grids) const {

  processed_grids.main_grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();

  const ::bosdyn::api::LocalGrid& terrain_grid_proto = terrain_grid.local_grid();
  const ::bosdyn::api::LocalGrid& terrain_valid_proto = valid_grid.local_grid();

  // Populate header and info (same as terrain grid)
  processed_grids.main_grid->header.stamp.sec =  terrain_grid_proto.acquisition_time().seconds();
  processed_grids.main_grid->header.stamp.nanosec = terrain_grid_proto.acquisition_time().nanos();

  processed_grids.main_grid->info.map_load_time = processed_grids.main_grid->header.stamp;
  processed_grids.main_grid->info.resolution = terrain_grid_proto.extent().cell_size();
  processed_grids.main_grid->info.width = terrain_grid_proto.extent().num_cells_x();
  processed_grids.main_grid->info.height = terrain_grid_proto.extent().num_cells_y();
  processed_grids.main_grid->header.frame_id = full_tf_root_id_;

  std::string grid_frame_id = terrain_grid_proto.frame_name_local_grid_data();

  ::bosdyn::api::SE3Pose transform, ground_plane_tf; 
  double ground_plane_z = 0.0;
  
  if(::bosdyn::api::get_a_tform_b(terrain_grid_proto.transforms_snapshot(), tf_root_, grid_frame_id, &transform)){
    if(::bosdyn::api::get_a_tform_b(tf_snapshot_, tf_root_, ::bosdyn::api::kGroundPlaneEstimateFrame, &ground_plane_tf)){
      ground_plane_z = ground_plane_tf.position().z();
      transform.mutable_position()->set_z(ground_plane_z);
    }
    else{
      logger_->logWarn("Error while getting transform to ground plane to tf_root");
    }
  }
  else{
    logger_->logError("Failed while getting transform to tf_root");
  }

  geometry_msgs::msg::Pose rosPose;
  convertToRos(transform, rosPose);

  processed_grids.main_grid->info.origin = rosPose;

  std::vector<float> unpacked_known_terrain_data, unpacked_terrain_valid_data;

  // Unpack Terrain Grid 
  unpackKnownGridData(terrain_grid_proto, unpacked_known_terrain_data); 

  // Unpack Terrain Valid
  unpackKnownGridData(terrain_valid_proto, unpacked_terrain_valid_data); 

  // Scale Terrain Data
  for(size_t i = 0; i < unpacked_known_terrain_data.size(); ++i){

    if(unpacked_known_terrain_data.size() != unpacked_terrain_valid_data.size()){
      logger_->logWarn("Terrain and Terrain Valid Sizes are different. Not publishing!");
      return;
    }
    // Original Terrain height is in world frame. We transform it to -1 to 1 wrt ground plane z. No change in orienatation
    // Terrain height range of -1.0 to 1.0 is scaled to -100 to 100
    // We also clamp the value to ensure it's within the valid range.
    // Unknown terrain height is set to 0 (assumes free space)
    bool valid_terrain = unpacked_terrain_valid_data[i] == 1;

    if(valid_terrain){
        float val = unpacked_known_terrain_data[i]; 
        val = val - ground_plane_z;
        processed_grids.main_grid->data.push_back(static_cast<int8_t>(std::max(-100.0, std::min(100.0, val * 100.0))));
    } else{
      processed_grids.main_grid->data.push_back(0);
    }
  }

  if(std::find(standard_grids_to_publish_.begin(), standard_grids_to_publish_.end(), "terrain_valid") != standard_grids_to_publish_.end()){
    auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    msg->header = processed_grids.main_grid->header;
    msg->info = processed_grids.main_grid->info;
    msg->header.stamp.sec =  terrain_valid_proto.acquisition_time().seconds();
    msg->header.stamp.nanosec =  terrain_valid_proto.acquisition_time().nanos();
    msg->info.map_load_time = msg->header.stamp;

    for(float val: unpacked_terrain_valid_data){
      // 0 and 1 are scaled to 0 and 100
      msg->data.push_back(static_cast<int8_t>(val * 100));
    }

    processed_grids.secondary_grid = std::move(msg);
  }
}

void LocalGridPublisher::downsampledGridTimerCallback() {
  if(!terrain_grid_initialized_){
    return;
  }

  nav_msgs::msg::MapMetaData& local_metadata = terrain_grid_data_->info;
  std::vector<int8_t>& local_terrain = terrain_grid_data_->data;
  ::bosdyn::api::SE3Pose& local_body_pose = tf_body_pose_;
  
  double ground_plane_z = local_metadata.origin.position.z;
  ::bosdyn::api::Quaternion yaw_only_orientation = ::bosdyn::api::ClosestYawOnly(local_body_pose.rotation());

  Eigen::Affine3d world_to_body_transform = Eigen::Translation3d(local_body_pose.position().x(), local_body_pose.position().y(), 0) *
                                           Eigen::Quaterniond(yaw_only_orientation.w(), yaw_only_orientation.x(), yaw_only_orientation.y(), yaw_only_orientation.z());

  auto downsampled_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  downsampled_msg->header.stamp = local_metadata.map_load_time;
  downsampled_msg->header.frame_id = full_tf_root_id_;
  downsampled_msg->info.resolution = policy_grid_resolution_;
  downsampled_msg->info.width = policy_grid_width_;
  downsampled_msg->info.height = policy_grid_height_;

  ::bosdyn::api::SE3Pose pose_yaw_only;
  pose_yaw_only.mutable_position()->set_x(local_body_pose.position().x());
  pose_yaw_only.mutable_position()->set_y(local_body_pose.position().y());
  pose_yaw_only.mutable_position()->set_z(local_body_pose.position().z());
  pose_yaw_only.mutable_rotation()->CopyFrom(yaw_only_orientation);

  // Define the target grid for the policy
  geometry_msgs::msg::Pose ds_grid_global_origin;
  geometry_msgs::msg::TransformStamped tf_stm;
  convertToRos(pose_yaw_only, tf_stm.transform);
  tf2::doTransform(ds_grid_local_origin_, ds_grid_global_origin, tf_stm);

  downsampled_msg->info.origin = ds_grid_global_origin;
  downsampled_msg->data.resize(policy_grid_width_ * policy_grid_height_);
  const double robot_z_in_world = local_body_pose.position().z();

  downsampled_msg->data.resize(num_policy_cells_);
  
  const int kernel_radius = 1;

  for (int iy = 0; iy < policy_grid_height_; ++iy) {
    for (int ix = 0; ix < policy_grid_width_; ++ix) {

      Eigen::Vector3d p_body(-0.8 + (ix + 0.5) * policy_grid_resolution_, -0.5 + (iy + 0.5) * policy_grid_resolution_, 0.0);

      Eigen::Vector3d p_world = world_to_body_transform * p_body;

      int center_hr_ix = static_cast<int>((p_world.x() - local_metadata.origin.position.x) / local_metadata.resolution);
      int center_hr_iy = static_cast<int>((p_world.y() - local_metadata.origin.position.y) / local_metadata.resolution);
      
      HeightModeTracker tracker;
      
      for (int dy = -kernel_radius; dy <= kernel_radius; ++dy) {
        for (int dx = -kernel_radius; dx <= kernel_radius; ++dx) {
          int current_hr_ix = center_hr_ix + dx;
          int current_hr_iy = center_hr_iy + dy;
          
          if (current_hr_ix >= 0 && current_hr_ix < local_metadata.width && 
              current_hr_iy >= 0 && current_hr_iy < local_metadata.height) {
            
            size_t hr_index = current_hr_iy * local_metadata.width + current_hr_ix;
            int new_count = ++tracker.histogram[local_terrain.at(hr_index)];
            if (new_count > tracker.max_count) {
              tracker.max_count = new_count;
              tracker.modal_height = local_terrain.at(hr_index);
            }

          }
        }
      }

      int8_t final_value = 0;
      if (tracker.max_count > 0) {
        const double height_in_world_m = (tracker.modal_height * 0.01) + ground_plane_z;
        const double policy_value = robot_z_in_world - height_in_world_m - 0.5;
        
        const double scaled_value = policy_value * 100.0;
        final_value = static_cast<int8_t>(std::max(-100.0, std::min(100.0, scaled_value)));
      }
      
      downsampled_msg->data[iy * policy_grid_width_ + ix] = final_value;
    }
  }

  middleware_handle_->publishSpecificOccupancyGrid("scandots", std::move(downsampled_msg));
}

}  // namespace spot_ros2

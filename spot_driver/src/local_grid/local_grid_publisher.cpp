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
constexpr auto kDownSampledGridCallbackPeriod = std::chrono::duration<double>{1.0 / 50.0}; // 50 Hz
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
    standard_grids_to_publish_.push_back("DownsampledScanDots");
  }

  middleware_handle_->createPublishers(standard_grids_to_publish_);

  terrain_grid_initialized_ = false;
  terrain_grid_data_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();

  // Create a timer to publish local grids
  timer_interface_->setTimer(kLocalGridCallbackPeriod, [this]() {
    localGridTimerCallback();
  });

  if (publish_scandots_) {
    timer_interface_->setTimer(kDownSampledGridCallbackPeriod, [this]() {
      downsampledGridTimerCallback();
    });
  }

  return true;
}

void LocalGridPublisher::localGridTimerCallback() {
  
  // Build the list of all grids we need to request from the robot.
  auto grids_to_request = standard_grids_to_request_;

  // The 'terrain' grid requires 'terrain_valid' for processing, so we must request it as well.
  if (std::find(grids_to_request.begin(), grids_to_request.end(), "terrain") != grids_to_request.end()) {
    if (std::find(grids_to_request.begin(), grids_to_request.end(), "terrain_valid") == grids_to_request.end()) {
        grids_to_request.push_back("terrain_valid");
    }
  }

  if (grids_to_request.empty()) {
    return;
  }

  // Make a single API call to get all requested grids.
  const auto result = local_grid_client_interface_->getLocalGrids(grids_to_request);
  if (!result) {
    logger_->logError("Failed to get local grids: " + result.error());
    return;
  }

  const auto& responses = result.value().local_grid_responses();
  const auto& num_errors = result.value().num_local_grid_errors();

  if(num_errors != 0){
    logger_->logWarn("Failed to get some of the local grids!");
  }
  
  const auto state_result = state_client_interface_->getRobotState();
  if(!result){
    logger_->logError("Failed to get robot state for local grids: " + result.error());
    return;
  }

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
  
  // Process NON-TERRAIN GRID first
  const ::bosdyn::api::LocalGridResponse *terrain_response, *terrain_valid_response;
  int publish_terrain_grids = 0;

  for (const ::bosdyn::api::LocalGridResponse& response : responses) {
    const auto& name = response.local_grid_type_name();
    // Skip 'terrain' and 'terrain_valid' since we've already handled them.
    if (name == "terrain"){
      terrain_response = &response;
      publish_terrain_grids++;
      continue;
    }
    if (name == "terrain_valid") {
      terrain_valid_response = &response;
      publish_terrain_grids++;
      continue;
    }

    if(response.status() != ::bosdyn::api::LocalGridResponse_Status::LocalGridResponse_Status_STATUS_OK){
      logger_->logError("No data received for local_grid with name: " + name);
      continue;
    }

    auto occ_msg = processNonTerrainGrid(response);
    if (occ_msg) {
      middleware_handle_->publishSpecificOccupancyGrid(name, std::move(occ_msg));
    }
  }

  if(publish_terrain_grids >= 2){
    ProcessedGridResult terrain_grids;
    processTerrainGrid(*terrain_response, *terrain_valid_response, terrain_grids);

    terrain_grid_data_ = terrain_grids.main_grid;    
    middleware_handle_->publishSpecificOccupancyGrid(terrain_response->local_grid_type_name(), terrain_grids.main_grid);
    
    if(terrain_grids.secondary_grid.has_value()){
      middleware_handle_->publishSpecificOccupancyGrid(terrain_valid_response->local_grid_type_name(), std::move(terrain_grids.secondary_grid.value()));
    }
  
    terrain_grid_initialized_ = true;
  }
  else if(publish_terrain_grids == 1){
    auto occ_msg = processNonTerrainGrid(*terrain_valid_response);
    if (occ_msg) {
      middleware_handle_->publishSpecificOccupancyGrid(terrain_valid_response->local_grid_type_name(), std::move(occ_msg));
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
      // This is a linear scaling: (val / 3.0) * 127.0
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

  logger_->logDebug("Downsampled grid publishing is not yet implemented.");
  // TODO:
  // 1. Create a nav_msgs::msg::OccupancyGrid::UniquePtr.
  auto downsampled_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
  //     Example: resolution is doubled, width/height are halved.
  // 3. Implement your downsampling logic.
  //    (e.g., iterate through the full_grid.data in 2x2 blocks and find max value)
  // 4. Set downsampled_msg->data with the result.
  // 5. Publish: middleware_handle_->publishSpecificOccupancyGrid("DownsampledScanDots", std::move(downsampled_msg));
}

}  // namespace spot_ros2

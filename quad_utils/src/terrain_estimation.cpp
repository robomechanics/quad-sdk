#include "quad_utils/terrain_estimation.h"

TerrainEstimation::TerrainEstimation(ros::NodeHandle nh)
    : filterChain_("grid_map::GridMap") {
  nh_ = nh;

  quad_utils::loadROSParam(nh_, "terrain_estimation/update_rate", update_rate_);

  std::string grf_topic, robot_state_topic, terrain_estimation_topic;
  quad_utils::loadROSParam(nh_, "/topics/state/ground_truth",
                           robot_state_topic);
  quad_utils::loadROSParam(nh_, "/topics/state/grfs", grf_topic);
  quad_utils::loadROSParam(nh_, "/topics/terrain_estimation",
                           terrain_estimation_topic);

  robot_state_sub_ = nh_.subscribe(robot_state_topic, 1,
                                   &TerrainEstimation::robotStateCallback, this,
                                   ros::TransportHints().tcpNoDelay(true));
  grf_sub_ = nh_.subscribe(grf_topic, 1, &TerrainEstimation::grfCallback, this,
                           ros::TransportHints().tcpNoDelay(true));

  // Terrain estimation related initialization
  // Initialize footstep history publisher
  terrain_estimation_pub_ =
      nh_.advertise<grid_map_msgs::GridMap>(terrain_estimation_topic, 1, true);

  // Initialize foot position arrays
  current_foot_positions_world_ = Eigen::VectorXd::Zero(4 * 3);

  // Initialize footstep history map
  terrain_estimation_ = grid_map::GridMap({"z"});
  terrain_estimation_.setFrameId("map");
  terrain_estimation_.setGeometry(grid_map::Length(10.0, 8.0), 0.1,
                                  grid_map::Position(3.0, 0.0));
  terrain_estimation_.clear("z");

  // Initialize footstep history filter
  filterChainParametersName_ =
      std::string("/terrain_estimation/grid_map_filters");
  if (!filterChain_.configure(filterChainParametersName_, nh)) {
    ROS_ERROR("Could not configure the filter chain!");
    return;
  }

  // Initialize the temporary foothold index vector
  tmp_foot_hist_idx_.resize(4);
}

void TerrainEstimation::robotStateCallback(
    const quad_msgs::RobotState::ConstPtr &msg) {
  // Make sure the data is actually populated
  if (msg->feet.feet.empty() || msg->joints.position.empty()) return;

  robot_state_msg_ = msg;
}

void TerrainEstimation::grfCallback(const quad_msgs::GRFArray::ConstPtr &msg) {
  // GRF callback
  grf_msg_ = msg;
}

void TerrainEstimation::publishFootStepHist() {
  // Publish foot step history
  if (robot_state_msg_ == NULL || grf_msg_ == NULL) return;

  // Get the current body and foot positions into Eigen
  current_state_ = quad_utils::bodyStateMsgToEigen(robot_state_msg_->body);
  quad_utils::multiFootStateMsgToEigen(robot_state_msg_->feet,
                                       current_foot_positions_world_);

  for (size_t i = 0; i < 4; i++) {
    // Check if it's on the ground
    if ((grf_msg_->contact_states.at(i) &&
         abs(grf_msg_->vectors.at(i).z) > 5)) {
      if (!tmp_foot_hist_idx_.at(i).empty()) {
        // If there's temporary index, replace them all with the actual contact
        // value
        for (size_t j = 0; j < tmp_foot_hist_idx_.at(i).size(); j++) {
          terrain_estimation_.at("z", tmp_foot_hist_idx_.at(i).at(j)) =
              current_foot_positions_world_(3 * i + 2) - toe_radius;
        }

        // Clear the temporary index vector
        tmp_foot_hist_idx_.at(i).clear();
      }

      // Add the foot position to the history
      terrain_estimation_.atPosition(
          "z", grid_map::Position(current_foot_positions_world_(3 * i + 0),
                                  current_foot_positions_world_(3 * i + 1))) =
          current_foot_positions_world_(3 * i + 2) - toe_radius;
    } else if (terrain_grid_.exists("z_inpainted")) {
      // Check if the current foot is lower than the records
      if (current_foot_positions_world_(3 * i + 2) - toe_radius <
          terrain_grid_.atPosition(
              "z_inpainted",
              grid_map::Position(current_foot_positions_world_(3 * i + 0),
                                 current_foot_positions_world_(3 * i + 1)),
              grid_map::InterpolationMethods::INTER_LINEAR)) {
        // Temporary record the position
        grid_map::Index tmp_idx;
        terrain_estimation_.getIndex(
            grid_map::Position(current_foot_positions_world_(3 * i + 0),
                               current_foot_positions_world_(3 * i + 1)),
            tmp_idx);
        tmp_foot_hist_idx_.at(i).push_back(tmp_idx);

        // Replace all the previous record with the latest position
        for (size_t j = 0; j < tmp_foot_hist_idx_.at(i).size(); j++) {
          terrain_estimation_.at("z", tmp_foot_hist_idx_.at(i).at(j)) =
              current_foot_positions_world_(3 * i + 2) - toe_radius;
        }
      }
    }
  }

  // Apply filter
  if (!filterChain_.update(terrain_estimation_, terrain_grid_)) {
    ROS_ERROR("Could not update the grid map filter chain!");
    return;
  }

  // Publish footstep history
  grid_map_msgs::GridMap terrain_estimation_message;
  grid_map::GridMapRosConverter::toMessage(terrain_grid_,
                                           terrain_estimation_message);
  terrain_estimation_pub_.publish(terrain_estimation_message);
}

void TerrainEstimation::spin() {
  ros::Rate r(update_rate_);

  // Continue publishing the map at the update rate
  while (ros::ok()) {
    ros::spinOnce();

    publishFootStepHist();

    r.sleep();
  }
}

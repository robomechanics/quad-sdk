#include "global_body_planner/global_body_plan.h"

GlobalBodyPlan::GlobalBodyPlan() {
  length_plan_.push_back(0);
  plan_status_ = UNSOLVED;
}

void GlobalBodyPlan::clear() {

  // Clear all data and reset the length of the plan
  t_plan_.clear();
  body_plan_.clear();
  grf_plan_.clear();
  primitive_id_plan_.clear();
  length_plan_.clear();
  length_plan_.push_back(0);
  plan_status_ = UNSOLVED;
}

void GlobalBodyPlan::invalidate() {
  length_plan_.back() = std::numeric_limits<double>::max();
  plan_status_ = UNSOLVED;
}

void GlobalBodyPlan::eraseAfterIndex(int start_index) {

  // Clear out old plan and interpolate to get full body plan
  t_plan_.erase(t_plan_.begin()+start_index, t_plan_.end());
  body_plan_.erase(body_plan_.begin()+start_index, body_plan_.end());
  grf_plan_.erase(grf_plan_.begin()+start_index, grf_plan_.end());
  primitive_id_plan_.erase(primitive_id_plan_.begin()+start_index, 
    primitive_id_plan_.end());
  length_plan_.erase(length_plan_.begin()+start_index+1, length_plan_.end());

}

void GlobalBodyPlan::loadPlanData(int plan_status, FullState &start_state, double dist_to_goal, 
  std::vector<State> &state_sequence, std::vector<Action> &action_sequence, double dt, double t0,
  const PlannerConfig &planner_config) {

  plan_status_ = plan_status;
  state_sequence_ = state_sequence;
  action_sequence_ = action_sequence;

  std::vector<State> interp_reduced_plan;

  // Loop through state action pairs, interp each and add to the path
  for (int i=0; i < action_sequence.size();i++)
  {
    interpStateActionPair(state_sequence[i], action_sequence[i], t0, dt, interp_reduced_plan,
      grf_plan_, t_plan_, primitive_id_plan_, length_plan_, planner_config);

    t0 += (action_sequence[i].t_s_leap + action_sequence[i].t_f + action_sequence[i].t_s_land);
  }

  // Add the final state in case it was missed by interp (GRF is undefined 
  // here so just copy the last element)
  t_plan_.push_back(t0);
  length_plan_.push_back(length_plan_.back() + poseDistance(state_sequence.back(),
    interp_reduced_plan.back()));
  interp_reduced_plan.push_back(state_sequence.back());
  grf_plan_.push_back(grf_plan_.back());
  primitive_id_plan_.push_back(LEAP_STANCE);

  // Lift from reduced into full body plan
  addFullStates(start_state, interp_reduced_plan, dt, body_plan_, planner_config);

  goal_distance_ = dist_to_goal;

}

void GlobalBodyPlan::addStateAndGRFToMsg(double t, int plan_index, const FullState &body_state, 
        const GRF &grf, int primitive_id, quad_msgs::RobotPlan& msg) {

  // Represent each state as an Odometry message
  quad_msgs::RobotState state;
  quad_utils::updateStateHeaders(state, msg.header.stamp+ros::Duration(t),
    msg.header.frame_id, plan_index);

  // Load the data into the message
  state.body = quad_utils::eigenToBodyStateMsg(fullStateToEigen(body_state));

  quad_msgs::GRFArray grf_msg;
  geometry_msgs::Vector3 vector_msg;
  vector_msg.x = grf[0];
  vector_msg.y = grf[1];
  vector_msg.z = grf[2];
  geometry_msgs::Point point_msg;
  quad_utils::Eigen3ToPointMsg(body_state.pos, point_msg);

  grf_msg.header = state.header;
  grf_msg.vectors.push_back(vector_msg);
  grf_msg.points.push_back(point_msg);

  bool contact_state = (primitive_id != FLIGHT);
  grf_msg.contact_states.push_back(contact_state);

  msg.states.push_back(state);
  msg.grfs.push_back(grf_msg);
  msg.plan_indices.push_back(plan_index);
  msg.primitive_ids.push_back(primitive_id);
}

void GlobalBodyPlan::convertToMsg(quad_msgs::RobotPlan &robot_plan_msg,
  quad_msgs::RobotPlan &discrete_robot_plan_msg) {

  if (getSize() <= 0)
    return;

  // Loop through the interpolated body plan and add to message
  for (int i=0;i<body_plan_.size(); ++i) {
    addStateAndGRFToMsg(t_plan_[i], i, body_plan_[i], grf_plan_[i],
      primitive_id_plan_[i], robot_plan_msg);
  }

  // Loop through the discrete states and add to message
  for (int i = 0; i<state_sequence_.size(); i++)
  {
    // Discrete states don't need roll, yaw, or timing data, set to zero
    FullState full_discrete_state = stateToFullState(state_sequence_[i],0,0,0,0,0,0);
    addStateAndGRFToMsg(0.0, 0, full_discrete_state, grf_plan_[i], 
      primitive_id_plan_[i], discrete_robot_plan_msg);
  }
  
  if (robot_plan_msg.states.size() != robot_plan_msg.grfs.size()) {
    throw std::runtime_error("Mismatch between number of states and wrenches, something is wrong");
  }

}
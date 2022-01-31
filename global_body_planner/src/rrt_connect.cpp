#include "global_body_planner/rrt_connect.h"
//constructor
RRTConnectClass::RRTConnectClass(){}
//destructor
RRTConnectClass::~RRTConnectClass() {}

using namespace planning_utils;

int RRTConnectClass::connect(PlannerClass &T, State s, const PlannerConfig &planner_config,
  int direction, ros::Publisher &tree_pub)
{
  // Find nearest neighbor
  flipDirection(s);
  int s_near_index = T.getNearestNeighbor(s);
  State s_near = T.getVertex(s_near_index);
  StateActionResult result;

  // Try to connect to nearest neighbor, add to graph if REACHED or ADVANCED
  int connect_result = attemptConnect(s_near, s, result, planner_config, direction);
  if (connect_result != TRAPPED)
  {
    int s_new_index = T.getNumVertices();
    T.addVertex(s_new_index, result.s_new);
    T.addEdge(s_near_index, s_new_index, result.length);
    T.addAction(s_new_index, result.a_new);

    #ifdef VISUALIZE_TREE
      publishStateActionPair(s_near,result.a_new, s,planner_config, tree_viz_msg_, tree_pub);
    #endif
    // T.updateGValue(s_new_index, T.getGValue(s_near_index) + result.length);
  }

  return connect_result;
}

std::vector<Action> RRTConnectClass::getActionSequenceReverse(PlannerClass &T, std::vector<int> path)
{
  // Assumes that actions are synched with the states at which they are executed (opposite of the definition in RRTClass)
  std::vector<Action> action_sequence;
  for (int i = 0; i < path.size()-1; ++i)
  {
    action_sequence.push_back(T.getAction(path.at(i)));
  }
  return action_sequence;
}

void RRTConnectClass::postProcessPath(std::vector<State> &state_sequence, std::vector<Action> &action_sequence, const PlannerConfig &planner_config)
{
  auto t_start = std::chrono::steady_clock::now();

  // Initialize first and last states
  State s_goal = state_sequence.back();
  State s = state_sequence.front();
  State s_next;
  State dummy;
  Action a_new;
  Action a_next;

  // Initialize state and action sequences
  std::vector<State> new_state_sequence;
  new_state_sequence.push_back(s);
  std::vector<Action> new_action_sequence;
  path_length_ = 0;

  // Iterate until the goal has been added to the state sequence
  while (s != s_goal)
  {
    // Make a copy of the original state and action sequences 
    std::vector<State> state_sequence_copy = state_sequence;
    std::vector<Action> action_sequence_copy = action_sequence;
    
    // Start at the back of the sequence
    s_next = state_sequence_copy.back();
    a_next = action_sequence_copy.back();
    State old_state;
    Action old_action;
    StateActionResult result;

    // Try to connect to the last state in the sequence
    // if unsuccesful remove the back and try again until successful or no states left
    while ((attemptConnect(s,s_next, result, planner_config, FORWARD) != REACHED) && (s != s_next))
    {
      old_state = s_next;
      old_action = a_next;
      state_sequence_copy.pop_back();
      action_sequence_copy.pop_back();
      s_next = state_sequence_copy.back(); 
      a_next = action_sequence_copy.back();
    }

    // If a new state was found add it to the sequence, otherwise add the next state in the original sequence
    if (s != s_next)
    {
      new_state_sequence.push_back(s_next);
      new_action_sequence.push_back(result.a_new);
      path_length_ += result.length;
      s = s_next;

    } else {
      new_state_sequence.push_back(old_state);
      new_action_sequence.push_back(old_action);

      // Recompute path length
      // flipDirection(old_state);
      isValidStateActionPair(old_state,old_action,result,planner_config);
      path_length_ += result.length;
      s = old_state;
    }
  }

  // Replace the old state and action sequences with the new ones
  state_sequence = new_state_sequence;
  action_sequence = new_action_sequence;

  auto t_end = std::chrono::steady_clock::now();
  std::chrono::duration<double> processing_time = t_end - t_start;
}

void RRTConnectClass::extractPath(PlannerClass &Ta, PlannerClass &Tb,
  std::vector<State> &state_sequence, std::vector<Action> &action_sequence,
  const PlannerConfig &planner_config)
{
  // Get both paths, remove the back of path_b and reverse it to align with path a
  std::vector<int> path_a = pathFromStart(Ta, Ta.getNumVertices()-1);
  std::vector<int> path_b = pathFromStart(Tb, Tb.getNumVertices()-1);

  std::reverse(path_b.begin(), path_b.end());
  std::vector<Action> action_sequence_b = getActionSequenceReverse(Tb, path_b);
  for (int i = 0; i < action_sequence_b.size(); i++) {
    flipDirection(action_sequence_b[i]);
  }
  path_b.erase(path_b.begin());

  state_sequence = getStateSequence(Ta, path_a);
  std::vector<State> state_sequence_b = getStateSequence(Tb, path_b);
  for (int i = 0; i < state_sequence_b.size(); i++) {
    flipDirection(state_sequence_b[i]);
  }
  state_sequence.insert(state_sequence.end(), state_sequence_b.begin(), state_sequence_b.end());

  action_sequence = getActionSequence(Ta, path_a);
  action_sequence.insert(action_sequence.end(), action_sequence_b.begin(), action_sequence_b.end());

  // Post process to reduce the path length
  postProcessPath(state_sequence, action_sequence, planner_config);

  // // Override to specify particular state action sequence
  // State s_start = state_sequence.front();
  // state_sequence.clear();
  // action_sequence.clear();
  // Action a = {10.0,0,0,-5,0,20.0,0.3,0.3};
  // State s_end = applyAction(s_start,a,planner_config);
  // state_sequence.push_back(s_start);
  // state_sequence.push_back(s_end);
  // action_sequence.push_back(a);
}

void RRTConnectClass::extractClosestPath(PlannerClass &Ta, const State &s_goal,
  std::vector<State> &state_sequence, std::vector<Action> &action_sequence,
  const PlannerConfig &planner_config) {

  std::vector<int> path_a = pathFromStart(Ta, Ta.getNearestNeighbor(s_goal));
  state_sequence = getStateSequence(Ta, path_a);
  action_sequence = getActionSequence(Ta, path_a);
  postProcessPath(state_sequence, action_sequence, planner_config);
}


int RRTConnectClass::runRRTConnect(const PlannerConfig &planner_config, State s_start,
  State s_goal, std::vector<State> &state_sequence, std::vector<Action> &action_sequence,
  ros::Publisher &tree_pub)
{

  // Perform validity checking on start and goal states
  if (!isValidState(s_start, planner_config, LEAP_STANCE)) {
    return INVALID_START_STATE;
  }
  // Set goal height to nominal distance above terrain
  s_goal.pos[2] = planner_config.terrain.getGroundHeight(s_goal.pos[0], s_goal.pos[1]) + 
    planner_config.H_NOM;
  if (!isValidState(s_goal, planner_config, LEAP_STANCE)) {
    return INVALID_GOAL_STATE;
  }
  if (stateDistance(s_start, s_goal) <= 1e-2) {
    return INVALID_START_GOAL_EQUAL;
  }

  // Initialize timing information
  auto t_start_total_solve = std::chrono::steady_clock::now();
  auto t_start_current_solve = std::chrono::steady_clock::now();
  int result;

  PlannerClass Ta(FORWARD);
  PlannerClass Tb(REVERSE);
  Ta.init(s_start);
  flipDirection(s_goal);
  Tb.init(s_goal);

  #ifdef VISUALIZE_TREE
    tree_viz_msg_.markers.clear();
    tree_viz_msg_.markers.resize(1);
  #endif

  anytime_horizon = poseDistance(s_start, s_goal)/planning_rate_estimate;


  while(true && ros::ok())
  {
    auto t_current = std::chrono::steady_clock::now();
    std::chrono::duration<double> total_elapsed = t_current - t_start_total_solve;
    std::chrono::duration<double> current_elapsed = t_current - t_start_current_solve;
    
    if(current_elapsed.count() >= anytime_horizon)
    {
      if(total_elapsed.count() >= planner_config.MAX_TIME)
      {
        std::cout << "Failed, exiting" << std::endl;
        elapsed_to_first_ = total_elapsed;
        num_vertices_ = (Ta.getNumVertices() + Tb.getNumVertices());
        break;
      }

      #ifndef VISUALIZE_TREE
        auto t_start_current_solve = std::chrono::steady_clock::now();
        anytime_horizon = anytime_horizon*horizon_expansion_factor;
        std::cout << "Failed, retrying with horizon of " << anytime_horizon << "s" << std::endl;
        Ta = PlannerClass(FORWARD);
        Tb = PlannerClass(REVERSE);
        tree_viz_msg_.markers.clear();
        Ta.init(s_start);
        Tb.init(s_goal);
        continue;
      #endif
    }

    // Generate random s
    State s_rand = Ta.randomState(planner_config);

    if (isValidState(s_rand, planner_config, LEAP_STANCE))
    {
      if (extend(Ta, s_rand, planner_config, FORWARD, tree_pub) != TRAPPED)
      {
        State s_new = Ta.getVertex(Ta.getNumVertices()-1);

        #ifdef VISUALIZE_TREE
          Action a_new = Ta.getAction(Ta.getNumVertices()-1);
          State s_parent = Ta.getVertex(Ta.getPredecessor(Ta.getNumVertices()-1));
          publishStateActionPair(s_parent,a_new, s_rand, planner_config, tree_viz_msg_, tree_pub);
        #endif

        if(connect(Tb, s_new, planner_config, FORWARD, tree_pub) == REACHED)
        {
          goal_found = true;

          auto t_end = std::chrono::steady_clock::now();
          elapsed_to_first_ = t_end - t_start_total_solve;
          path_length_ = Ta.getGValue(Ta.getNumVertices()-1) + Tb.getGValue(Tb.getNumVertices()-1);
          break;
        }
      }
    }

    s_rand = Tb.randomState(planner_config);

    if (isValidState(s_rand, planner_config, LEAP_STANCE))
    {
      if (extend(Tb, s_rand, planner_config, FORWARD, tree_pub) != TRAPPED)
      {
        State s_new = Tb.getVertex(Tb.getNumVertices()-1);

        #ifdef VISUALIZE_TREE
          Action a_new = Tb.getAction(Tb.getNumVertices()-1);
          State s_parent = Tb.getVertex(Tb.getPredecessor(Tb.getNumVertices()-1));
          publishStateActionPair(s_parent,a_new, s_rand, planner_config, tree_viz_msg_, tree_pub);
        #endif

        if(connect(Ta, s_new, planner_config, FORWARD, tree_pub) == REACHED)
        {
          goal_found = true;

          auto t_end = std::chrono::steady_clock::now();
          elapsed_to_first_ = t_end - t_start_total_solve;
          path_length_ = Ta.getGValue(Ta.getNumVertices()-1) + Tb.getGValue(Tb.getNumVertices()-1);
          break;
        }
      }
    }
  }

  num_vertices_ = (Ta.getNumVertices() + Tb.getNumVertices());

  if (goal_found == true)
  {
    extractPath(Ta, Tb, state_sequence, action_sequence, planner_config);
    result = VALID;
  } else {
    extractClosestPath(Ta, s_goal, state_sequence, action_sequence, planner_config);
    result = VALID_PARTIAL;
    std::cout << "Extracting partial path" << std::endl;
    printStateSequence(state_sequence);
    printActionSequence(action_sequence);
  }

  auto t_end = std::chrono::steady_clock::now();
  elapsed_total_ = t_end - t_start_total_solve;

  path_duration_ = 0.0;
  for (Action a : action_sequence)
  {
    path_duration_ += (a.t_s_leap + a.t_f + a.t_s_land);
  }

  return result;
}

#include "global_body_planner/rrt.h"
#include <ctime>
//constructor
RRTClass::RRTClass(){}
//destructor
RRTClass::~RRTClass() {}

using namespace planning_utils;

bool RRTClass::newConfig(State s, State s_near, StateActionResult &result,
  const PlannerConfig &planner_config, int direction, ros::Publisher &tree_pub)
{
  double best_so_far = stateDistance(s_near, s);

  Eigen::Vector3d surf_norm =
    planner_config.terrain.getSurfaceNormalFilteredEigen(s.pos[0], s.pos[1]);

  int tree_size = tree_viz_msg_.markers.size();

  // if (direction == REVERSE) {
  // 	flipDirection(s_near);
  // }
  // direction = FORWARD;

  std::cout << "Entering new config" << std::endl;
  int num_total_actions = 0;
  int num_valid_actions = 0;

  bool any_valid_actions = false;
  for (int i = 0; i < planner_config.NUM_GEN_STATES; ++i)
  {
    bool valid_state_found = false;
    StateActionResult current_result;

    // Action a_test = getRandomAction(surf_norm,planner_config);
    Action a_test;
    bool is_valid_initial = getRandomLeapAction(s_near,surf_norm,a_test,planner_config);
    num_total_actions++;
    for (int j = 0; j < planner_config.NUM_GEN_STATES; ++j)
    {
    
      bool is_valid = isValidStateActionPair(s_near, a_test, current_result, planner_config);

      #ifdef VISUALIZE_ALL_CANDIDATE_ACTIONS
        if (direction == FORWARD) {
          // publishStateActionPair(s_near,current_result.a_new, s,planner_config, tree_viz_msg_, tree_pub);
          publishStateActionPair(s_near,a_test, s,planner_config, tree_viz_msg_, tree_pub);
        } else if (direction == REVERSE) {
        //   State s_reverse = applyActionReverse(s_near,a_test,planner_config);
        //   publishStateActionPair(s_reverse, a_test, s,planner_config, tree_viz_msg_,
        //     tree_pub);
        }
      #endif

      if (is_valid == true)
      {
        num_valid_actions++;
        valid_state_found = true;
        break;
      } else {
        is_valid_initial = getRandomLeapAction(s_near,surf_norm,a_test,planner_config);
        num_total_actions++;
      }
    }	

    if (valid_state_found)
    {
      double current_dist = stateDistance(current_result.s_new, s);
      if (current_dist < best_so_far)
      {
        any_valid_actions = true;
        best_so_far = current_dist;
        result.s_new = current_result.s_new;
        result.a_new = current_result.a_new;
        result.length = current_result.length;
      }
    }
  }

  std::cout << "Fraction valid action = " << (double)num_valid_actions/num_total_actions << std::endl;
  throw std::runtime_error("Stop here, viz?");

  std::cout << "Attempting connect,  state = ";
  printStateNewline(s);
  printStateNewline(s_near);

  // Try connecting directly
  StateActionResult current_result;
  if (attemptConnect(s_near, s, current_result, planner_config, direction) != TRAPPED) {
    double current_dist = stateDistance(current_result.s_new, s);

    if (current_dist < best_so_far)
    {
      best_so_far = current_dist;
      result.s_new = current_result.s_new;
      result.a_new = current_result.a_new;
      result.length = current_result.length;
    }
  }


  #ifdef VISUALIZE_ALL_CANDIDATE_ACTIONS
    tree_viz_msg_.markers.resize(tree_size);
  #endif

  if (best_so_far == stateDistance(s_near, s))
  {
    return false;
  } else
  {
    return true;
  }
}

int RRTClass::attemptConnect(const State &s_existing, const State &s, double t_s,
  StateActionResult &result, const PlannerConfig &planner_config, int direction)
{
  // Enforce stance time greater than the kinematic check resolution to ensure that the action is useful
  if (t_s <= planner_config.KINEMATICS_RES)
    return TRAPPED;

  // Initialize the start and goal states depending on the direction, as well as the stance and flight times
  State s_start = (direction == FORWARD) ? s_existing : s;
  State s_goal = (direction == FORWARD) ? s : s_existing;
  double t_f = 0;

  // Update the vertical component of velocities to match the terrain
  setDz(s_start, planner_config);
  setDz(s_goal, planner_config);
  
  // Compute accelerations at start and end of the behavior
  Eigen::Vector3d acc_0 = -(2.0*(3.0*s_start.pos - 3.0*s_goal.pos + 2.0*s_start.vel*t_s + 
    s_goal.vel*t_s))/(t_s*t_s);
  Eigen::Vector3d acc_f = (2.0*(3.0*s_start.pos - 3.0*s_goal.pos + s_start.vel*t_s +
    2.0*s_goal.vel*t_s))/(t_s*t_s);

  // Transform from accelerations to body weight grfs
  result.a_new.grf_0 = (acc_0-planner_config.G_VEC)/planner_config.G_CONST;
  result.a_new.grf_f = (acc_f-planner_config.G_VEC)/planner_config.G_CONST;

  // Set the vertical component of accel to contain height above terrain
  result.a_new.grf_0[2] = s_start.pos[2] - getZFromState(s_start, planner_config);
  result.a_new.grf_f[2] = s_goal.pos[2] - getZFromState(s_goal, planner_config);
  
  result.a_new.t_s_leap = t_s;
  result.a_new.t_f = 0;
  result.a_new.t_s_land = 0;
  result.a_new.dz_0 = getDzFromState(s_start, planner_config);
  result.a_new.dz_f = getDzFromState(s_goal, planner_config);

  // If the connection results in an infeasible action, abort and return trapped
  if (isValidAction(result.a_new,planner_config))
  {
    // If valid, great, return REACHED, otherwise try again to the valid state returned by isValidStateActionPair
    if (isValidStateActionPair(s_start, result.a_new, result,planner_config)){
      return REACHED;
    } else {
      if (attemptConnect(s_existing, result.s_new, result.t_new, result, planner_config, direction) == TRAPPED)
        return TRAPPED;
      else
        return ADVANCED;
    }
  }

  return attemptConnect(s_existing, s, t_s*2, result, planner_config, direction);
}

int RRTClass::attemptConnect(const State &s_existing, const State &s, StateActionResult &result,
  const PlannerConfig &planner_config, int direction)
{
  // select desired stance time to enforce a nominal stance velocity
  double t_s = poseDistance(s, s_existing)/planner_config.V_NOM;
  return attemptConnect(s_existing, s, t_s, result, planner_config, direction);
}

int RRTClass::extend(PlannerClass &T, const State &s, const PlannerConfig &planner_config,
  int direction, ros::Publisher &tree_pub)
{
  int s_near_index = T.getNearestNeighbor(s);
  State s_near = T.getVertex(s_near_index);
  StateActionResult result;

  if (newConfig(s,s_near,result, planner_config, direction, tree_pub) == true)
  {
    int s_new_index = T.getNumVertices();
    T.addVertex(s_new_index, result.s_new);
    T.addEdge(s_near_index, s_new_index, result.length);
    T.addAction(s_new_index, result.a_new);

    // T.updateGValue(s_new_index, T.getGValue(s_near_index) + result.length);

    // if (s_new == s)
    if (isWithinBounds(result.s_new, s,planner_config) == true)
    {
      return REACHED;
    } else {
      return ADVANCED;
    }
  } else {
    return TRAPPED;
  }
}

std::vector<int> RRTClass::pathFromStart(PlannerClass &T, int s)
{
  std::vector<int> path;
  path.push_back(s);
  while(s != 0)
  {
    int s_pred = T.getPredecessor(s);
    path.push_back(s_pred);
    s = s_pred;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<State> RRTClass::getStateSequence(PlannerClass &T, std::vector<int> path)
{
  std::vector<State> state_sequence;
  for (int i = 0; i < path.size(); ++i)
  {
    state_sequence.push_back(T.getVertex(path.at(i)));
  }
  return state_sequence;
}

std::vector<Action> RRTClass::getActionSequence(PlannerClass &T, std::vector<int> path)
{
  // Assumes that actions are synched with the states to which they lead
  std::vector<Action> action_sequence;
  for (int i = 1; i < path.size(); ++i)
  {
    action_sequence.push_back(T.getAction(path.at(i)));
  }
  return action_sequence;
}

void RRTClass::printPath(PlannerClass &T, std::vector<int> path)
{
  std::cout << "Printing path:";
  for (int i = 0; i < path.size(); i++)
  {
    std::cout << std::endl;
    std::cout << path.at(i) << " or ";
    printState(T.getVertex(path.at(i)));
    std::cout << " ->";
  }
  std::cout << "\b\b  " << std::endl;
}

void RRTClass::getStatistics(double &plan_time, int &vertices_generated, double &plan_length, double& path_duration)
{
  plan_time = elapsed_total_.count();
  vertices_generated = num_vertices_;
  plan_length = path_length_;
  path_duration = path_duration_;
}
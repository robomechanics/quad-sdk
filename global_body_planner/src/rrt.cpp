#include "global_body_planner/rrt.h"
#include <ctime>
//constructor
RRTClass::RRTClass(){}
//destructor
RRTClass::~RRTClass() {}

using namespace planning_utils;

bool RRTClass::newConfig(State s, State s_near, StateActionResult &result,
	const PlannerConfig &planner_config, int direction)
{
	double best_so_far = stateDistance(s_near, s);
	std::array<double, 3> surf_norm = planner_config.terrain.getSurfaceNormal(s[0], s[1]);

	for (int i = 0; i < planner_config.NUM_GEN_STATES; ++i)
	{
		bool valid_state_found = false;
		StateActionResult current_result;

		Action a_test = getRandomAction(surf_norm,planner_config);
		for (int j = 0; j < planner_config.NUM_GEN_STATES; ++j)
		{
			bool is_valid;
			if (direction == FORWARD)
			{
				is_valid = isValidStateActionPair(s_near, a_test, current_result, planner_config);
			} else if (direction == REVERSE)
			{
				is_valid = isValidStateActionPairReverse(s_near, a_test, current_result, planner_config);
			}

			if (is_valid == true)
			{
				valid_state_found = true;
				break;
			} else {
				a_test = getRandomAction(surf_norm,planner_config);
			}
		}			

		if (valid_state_found == true)
		{
			double current_dist = stateDistance(current_result.s_new, s);
			if (current_dist < best_so_far)
			{
				best_so_far = current_dist;
				result.s_new = current_result.s_new;
				result.a_new = a_test;
				result.length = current_result.length;
			}
		}
	}

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

	if (best_so_far == stateDistance(s_near, s))
	{
		return false;
	} else
	{
		return true;
	}
}

int RRTClass::attemptConnect(State s_existing, State s, double t_s, StateActionResult &result,
  const PlannerConfig &planner_config, int direction)
{
  // Enforce stance time greater than the kinematic check resolution to ensure that the action is useful
  if (t_s <= planner_config.KINEMATICS_RES)
    return TRAPPED;

  // Initialize the start and goal states depending on the direction, as well as the stance and flight times
  State s_start = (direction == FORWARD) ? s_existing : s;
  State s_goal = (direction == FORWARD) ? s : s_existing;
  double t_f = 0;
  
  // Calculate the action to connect the desired states
  double x_td = s_start[0];
  double y_td = s_start[1];
  double z_td = s_start[2];
  double dx_td = s_start[3];
  double dy_td = s_start[4];
  double dz_td = s_start[5];

  double x_to = s_goal[0];
  double y_to = s_goal[1];
  double z_to = s_goal[2];
  double dx_to = s_goal[3];
  double dy_to = s_goal[4];
  double dz_to = s_goal[5];

  double p_td = s_start[6];
  double dp_td = s_start[7];
  double p_to = s_goal[6];
  double dp_to = s_goal[7];

  // result.a_new[0] = -(2.0*(3.0*x_td - 3.0*x_to + 2.0*dx_td*t_s + dx_to*t_s))/(t_s*t_s);
  // result.a_new[1] = -(2.0*(3.0*y_td - 3.0*y_to + 2.0*dy_td*t_s + dy_to*t_s))/(t_s*t_s);
  // result.a_new[2] = -(2.0*(3.0*z_td - 3.0*z_to + 2.0*dz_td*t_s + dz_to*t_s))/(t_s*t_s);
  // result.a_new[3] = (2.0*(3.0*x_td - 3.0*x_to + dx_td*t_s + 2.0*dx_to*t_s))/(t_s*t_s);
  // result.a_new[4] = (2.0*(3.0*y_td - 3.0*y_to + dy_td*t_s + 2.0*dy_to*t_s))/(t_s*t_s);
  // result.a_new[5] = (2.0*(3.0*z_td - 3.0*z_to + dz_td*t_s + 2.0*dz_to*t_s))/(t_s*t_s);
  // result.a_new[6] = t_s;
  // result.a_new[7] = t_f;

  result.a_new[0] = -(2.0*(3.0*x_td - 3.0*x_to + 2.0*dx_td*t_s + dx_to*t_s))/(t_s*t_s);
  result.a_new[1] = -(2.0*(3.0*y_td - 3.0*y_to + 2.0*dy_td*t_s + dy_to*t_s))/(t_s*t_s);
  result.a_new[2] = z_td - planner_config.terrain.getGroundHeight(x_td,y_td);
  result.a_new[3] = (2.0*(3.0*x_td - 3.0*x_to + dx_td*t_s + 2.0*dx_to*t_s))/(t_s*t_s);
  result.a_new[4] = (2.0*(3.0*y_td - 3.0*y_to + dy_td*t_s + 2.0*dy_to*t_s))/(t_s*t_s);
  result.a_new[5] = z_to - planner_config.terrain.getGroundHeight(x_to,y_to);;
  result.a_new[6] = t_s;
  result.a_new[7] = t_f;

  // If the connection results in an infeasible action, abort and return trapped
  if (isValidAction(result.a_new,planner_config) == true)
  {
    // Check if the resulting state action pair is kinematically valid
    bool isValid = (direction == FORWARD) ? (isValidStateActionPair(s_start, result.a_new, result, 
      planner_config)) : (isValidStateActionPairReverse(s_goal,result.a_new, result, planner_config));

    // If valid, great, return REACHED, otherwise try again to the valid state returned by isValidStateActionPair
    if (isValid == true)
      return REACHED;
    else
    {
      if (attemptConnect(s_existing, result.s_new, result.t_new, result, planner_config, direction) == TRAPPED)
        return TRAPPED;
      else
        return ADVANCED;
    }
  }
  return TRAPPED;
}

int RRTClass::attemptConnect(State s_existing, State s, StateActionResult &result,
  const PlannerConfig &planner_config, int direction)
{
  // select desired stance time to enforce a nominal stance velocity
  double t_s = poseDistance(s, s_existing)/planner_config.V_NOM;
  return attemptConnect(s_existing, s, t_s, result, planner_config, direction);
}

int RRTClass::extend(PlannerClass &T, State s, const PlannerConfig &planner_config, int direction)
{
	int s_near_index = T.getNearestNeighbor(s);
	State s_near = T.getVertex(s_near_index);
	StateActionResult result;

	if (newConfig(s,s_near,result, planner_config, direction) == true)
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
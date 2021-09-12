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

	if (best_so_far == stateDistance(s_near, s))
	{
		return false;
	} else
	{
		return true;
	}
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
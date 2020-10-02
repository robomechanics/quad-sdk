#include "global_body_planner/rrt.h"
#include <ctime>
//constructor
RRTClass::RRTClass(){}
//destructor
RRTClass::~RRTClass() {}

using namespace planning_utils;

bool RRTClass::newConfig(State s, State s_near, State &s_new, Action &a_new, FastTerrainMap& terrain, int direction)
{
	double best_so_far = stateDistance(s_near, s);
	std::array<double, 3> surf_norm = terrain.getSurfaceNormal(s[0], s[1]);

	// std::array<double,3> 
	for (int i = 0; i < NUM_GEN_STATES; ++i)
	{
		bool valid_state_found = false;
		State s_test;
		double t_new;


		Action a_test = getRandomAction(surf_norm);
		for (int j = 0; j < NUM_GEN_STATES; ++j)
		{
			bool is_valid;
			if (direction == FORWARD)
			{
				is_valid = isValidStateActionPair(s_near, a_test, terrain, s_test, t_new);
			} else if (direction == REVERSE)
			{
				is_valid = isValidStateActionPairReverse(s_near, a_test, terrain, s_test, t_new);
			}

			if (is_valid == true)
			{
				valid_state_found = true;
				break;
			} else {
				a_test = getRandomAction(surf_norm);
			}
		}			

		if (valid_state_found == true)
		{
			double current_dist = stateDistance(s_test, s);
			if (current_dist < best_so_far)
			{
				best_so_far = current_dist;
				s_new = s_test;
				a_new = a_test;
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

int RRTClass::extend(PlannerClass &T, State s, FastTerrainMap& terrain, int direction)
{
	int s_near_index = T.getNearestNeighbor(s);
	State s_near = T.getVertex(s_near_index);
	State s_new;
	Action a_new;

	if (newConfig(s,s_near,s_new, a_new, terrain, direction) == true)
	{
		int s_new_index = T.getNumVertices();
		T.addVertex(s_new_index, s_new);
		T.addEdge(s_near_index, s_new_index);
		T.addAction(s_new_index, a_new);
		T.updateGValue(s_new_index, T.getGValue(s_near_index) + poseDistance(s_near, s_new));


		// if (s_new == s)
		if (isWithinBounds(s_new, s) == true)
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

void RRTClass::getStatistics(double &plan_time, int &success, int &vertices_generated, double &time_to_first_solve,
		std::vector<double> &cost_vector, std::vector<double> &cost_vector_times, double& path_duration)
{
	plan_time = elapsed_total.count();
	success = success_;
	vertices_generated = num_vertices;
	time_to_first_solve = elapsed_to_first.count();
	cost_vector = cost_vector_;
	cost_vector_times = cost_vector_times_;
	path_duration = path_duration_;
	std::cout << "path_duration: " << path_duration << std::endl;
}

void RRTClass::buildRRT(
		   FastTerrainMap& terrain,
           State s_start,
           State s_goal,
           std::vector<State> &state_sequence,
           std::vector<Action> &action_sequence)
{
	auto t_start = std::chrono::high_resolution_clock::now();
	success_ = 0;

	cost_vector_.clear();
	cost_vector_times_.clear();

	std::cout << "RRT" << std::endl;

	PlannerClass T;
    T.init(s_start);

    int s_goal_idx;
    goal_found = false;
	while(true)
	{
		auto t_current = std::chrono::high_resolution_clock::now();
    	std::chrono::duration<double> current_elapsed = t_current - t_start;
		if(current_elapsed.count() >= 30)
		{
			num_vertices = T.getNumVertices();
			auto t_end = std::chrono::high_resolution_clock::now();
    		elapsed_total = t_end - t_start;
			return;
		}

		// Generate new s to be either random or the goal
		double prob_goal = (double)rand()/RAND_MAX;
		State s = (prob_goal <= prob_goal_thresh) ? s_goal : T.randomState(terrain);

		if (isValidState(s, terrain, STANCE))
		{
			int result = extend(T, s, terrain, FORWARD);

			if ((isWithinBounds(s, s_goal) == true) && (result == REACHED))
			{
				std::cout << "goal found!" << std::endl;
				goal_found = true;
				s_goal_idx = T.getNumVertices()-1;


				auto t_end = std::chrono::high_resolution_clock::now();
	    		elapsed_to_first = t_end - t_start;
				break;
			}	
		}
		
	}

	num_vertices = T.getNumVertices();

	if (goal_found == true)
	{
		std::vector<int> path = pathFromStart(T, T.getNumVertices()-1);
		state_sequence = getStateSequence(T, path);
		action_sequence = getActionSequence(T, path);
	} else {
		std::cout << "Path not found" << std::endl;
	}

	auto t_end = std::chrono::high_resolution_clock::now();
    elapsed_total = t_end - t_start;

    if (elapsed_total.count() <= 5.0)
    	success_ = 1;

    path_duration_ = 0.0;
    for (Action a : action_sequence)
    {
    	path_duration_ += a[6] + a[7];
    }

    path_quality_ = T.getGValue(s_goal_idx);
    cost_vector_.push_back(path_quality_);
    cost_vector_times_.push_back(elapsed_total.count());
    std::cout << "Path quality = " << path_quality_ << std::endl;
}
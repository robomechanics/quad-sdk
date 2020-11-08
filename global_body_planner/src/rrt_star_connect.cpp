#include "global_body_planner/rrt_star_connect.h"
#include <ctime>
//constructor
RRTStarConnectClass::RRTStarConnectClass(){}
//destructor
RRTStarConnectClass::~RRTStarConnectClass() {}

using namespace planning_utils;

int RRTStarConnectClass::extend(PlannerClass &T, State s, FastTerrainMap& terrain, int direction)
{
	int s_nearest_index = T.getNearestNeighbor(s);
	State s_nearest = T.getVertex(s_nearest_index);
	State s_new;
	Action a_new;
	Action a_connect;
	State dummy;

	if (newConfig(s,s_nearest,s_new, a_new, terrain, direction) == true)
	{
		int s_new_idx = T.getNumVertices();
		T.addVertex(s_new_idx, s_new);
		
		int s_min_idx = s_nearest_index;
		std::vector<int> neighbors = T.neighborhoodDist(s_new, delta);

		// Initialize g_s_new
		double g_s_new = T.getGValue(s_nearest_index) + poseDistance(s_new, s_nearest);
		for (int i = 0; i < neighbors.size(); i++)
		{
			int s_near_idx = neighbors[i];
			State s_near = T.getVertex(s_near_idx);			

			if (attemptConnect(s_near, s_new, dummy, a_connect, terrain, direction) == REACHED)
			{
				double g_s_near = T.getGValue(s_near_idx) + poseDistance(s_near, s_new);
				if (g_s_near < g_s_new)
				{
					a_new = a_connect;
					s_min_idx = s_near_idx;
					g_s_new = g_s_near;
				}
			}
		}

		T.addEdge(s_min_idx, s_new_idx);
		T.updateGValue(s_new_idx, g_s_new);
		T.addAction(s_new_idx, a_new);
		
		for (int s_near_idx : neighbors)
		{
			if (s_near_idx != s_min_idx)
			{
				State s_near = T.getVertex(s_near_idx);
				if ((attemptConnect(s_new, s_near, dummy, a_connect, terrain, direction) == REACHED) && 
						( T.getGValue(s_near_idx) > (T.getGValue(s_new_idx) + poseDistance(s_near, s_new)) ))
				{
					int s_parent = T.getPredecessor(s_near_idx);
					T.removeEdge(s_parent, s_near_idx);
					T.addEdge(s_new_idx, s_near_idx);
					T.updateGValue(s_near_idx, (T.getGValue(s_new_idx) + poseDistance(s_near, s_new)));
					T.addAction(s_near_idx, a_connect);
				}
			}
		}

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

void RRTStarConnectClass::getStateAndActionSequences(PlannerClass &Ta, PlannerClass &Tb, 
	int shared_a_idx, int shared_b_idx, 
	std::vector<State> &state_sequence, std::vector<Action> &action_sequence)
{
	state_sequence.clear(); action_sequence.clear();

	// Get both paths, remove the back of path_b and reverse it to align with path a
	std::vector<int> path_a = pathFromStart(Ta, shared_a_idx);
	std::vector<int> path_b = pathFromStart(Tb, shared_b_idx);

	std::reverse(path_b.begin(), path_b.end());
	std::vector<Action> action_sequence_b = getActionSequenceReverse(Tb, path_b);
	path_b.erase(path_b.begin());

	state_sequence = getStateSequence(Ta, path_a);
	std::vector<State> state_sequence_b = getStateSequence(Tb, path_b);
	state_sequence.insert(state_sequence.end(), state_sequence_b.begin(), state_sequence_b.end());

	action_sequence = getActionSequence(Ta, path_a);
	action_sequence.insert(action_sequence.end(), action_sequence_b.begin(), action_sequence_b.end());
}

void RRTStarConnectClass::buildRRTStarConnect(FastTerrainMap& terrain, State s_start, State s_goal,
	std::vector<State> &state_sequence, std::vector<Action> &action_sequence, double max_time)
{
	double elapsed_in_processing = 0.0;
	auto t_start = std::chrono::high_resolution_clock::now();
	success_ = 0;

	std::cout << "RRT Star Connect" << std::endl;

	cost_vector_.clear();
	cost_vector_times_.clear();

	PlannerClass Ta;
	Ta.init(s_start);

	PlannerClass Tb;
	Tb.init(s_goal);

	int shared_a_idx, shared_b_idx;
	std::vector<int> shared_a, shared_b;

	goal_found = false;
	double cost_so_far = INFTY;
	while(true)
	{
		// Generate random s
		State s_rand = Ta.randomState(terrain);

		if (isValidState(s_rand, terrain, STANCE))
		{
			if (extend(Ta, s_rand, terrain, FORWARD) != TRAPPED)
			{
				State s_new = Ta.getVertex(Ta.getNumVertices()-1);

				if(connect(Tb, s_new, terrain, REVERSE) == REACHED)
				{
					if (goal_found == false)
					{
						auto t_end = std::chrono::high_resolution_clock::now();
						elapsed_to_first = t_end - t_start;
					}
					
					goal_found = true;
					shared_a.push_back(Ta.getNumVertices()-1);
					shared_b.push_back(Tb.getNumVertices()-1);
				}
			}
		}

		s_rand = Tb.randomState(terrain);

		if (isValidState(s_rand, terrain, STANCE))
		{
			if (extend(Tb, s_rand, terrain, REVERSE) != TRAPPED)
			{
				State s_new = Tb.getVertex(Tb.getNumVertices()-1);

				if(connect(Ta, s_new, terrain, FORWARD) == REACHED)
				{
					if (goal_found == false)
					{
						auto t_end = std::chrono::high_resolution_clock::now();
						elapsed_to_first = t_end - t_start;
					}

					goal_found = true;
					shared_a.push_back(Ta.getNumVertices()-1);
					shared_b.push_back(Tb.getNumVertices()-1);
				}
			}
		}

		auto t_current = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> current_elapsed = t_current - t_start;

		if ((goal_found == true) && (current_elapsed.count() >= max_time))
			break;

		// Find the shared state that has the lowest cost summed over both trees
		for (int i = 0; i < shared_a.size(); ++i)
		{
			int a_idx = shared_a[i]; int b_idx = shared_b[i];
			double cost = Ta.getGValue(a_idx) + Tb.getGValue(b_idx);
			if (cost < cost_so_far)
			{
				cost_so_far = cost;
				shared_a_idx = a_idx; shared_b_idx = b_idx;

				auto t_current = std::chrono::high_resolution_clock::now();
				std::chrono::duration<double> current_elapsed = t_current - t_start;

				cost_vector_.push_back(cost_so_far);
				cost_vector_times_.push_back(current_elapsed.count());
			}
		}
	}
	
	num_vertices = Ta.getNumVertices() + Tb.getNumVertices();
	
	if (goal_found == true)
	{
		getStateAndActionSequences(Ta, Tb, shared_a_idx, shared_b_idx, state_sequence, action_sequence);
	} else {
		std::cout << "Path not found" << std::endl;
	}

	auto t_end = std::chrono::high_resolution_clock::now();
	elapsed_total = t_end - t_start;

	cost_vector_.push_back(cost_so_far);
	cost_vector_times_.push_back(elapsed_total.count());

	postProcessPath(state_sequence, action_sequence, terrain);


	if (elapsed_total.count() <= 5.0)
		success_ = 1;

	path_duration_ = 0.0;
    for (Action a : action_sequence)
    {
    	path_duration_ += a[6] + a[7];
    }

	std::cout << "Path quality = " << path_quality_ << std::endl;
}
#include "global_body_planner/rrt_connect.h"
//constructor
RRTConnectClass::RRTConnectClass(){}
//destructor
RRTConnectClass::~RRTConnectClass() {}

int RRTConnectClass::attemptConnect(State s_existing, State s, double t_s, State &s_new, Action &a_new, FastTerrainMap& terrain, int direction)
{
	// Enforce stance time greater than the kinematic check resolution to ensure that the action is useful
	if (t_s <= KINEMATICS_RES)
		return TRAPPED;

	// Initialize the start and goal states depending on the direction, as well as the stance and flight times
	State s_start = (direction == FORWARD) ? s_existing : s;
	State s_goal = (direction == FORWARD) ? s : s_existing;
	double t_new;
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

	a_new[0] = -(2.0*(3.0*x_td - 3.0*x_to + 2.0*dx_td*t_s + dx_to*t_s))/(t_s*t_s);
	a_new[1] = -(2.0*(3.0*y_td - 3.0*y_to + 2.0*dy_td*t_s + dy_to*t_s))/(t_s*t_s);
	a_new[2] = -(2.0*(3.0*z_td - 3.0*z_to + 2.0*dz_td*t_s + dz_to*t_s))/(t_s*t_s);
	a_new[3] = (2.0*(3.0*x_td - 3.0*x_to + dx_td*t_s + 2.0*dx_to*t_s))/(t_s*t_s);
	a_new[4] = (2.0*(3.0*y_td - 3.0*y_to + dy_td*t_s + 2.0*dy_to*t_s))/(t_s*t_s);
	a_new[5] = (2.0*(3.0*z_td - 3.0*z_to + dz_td*t_s + 2.0*dz_to*t_s))/(t_s*t_s);
	a_new[6] = t_s;
	a_new[7] = t_f;

	a_new[8] = -(2.0*(3.0*p_td - 3.0*p_to + 2.0*dp_td*t_s + dp_to*t_s))/(t_s*t_s);
	a_new[9] = (2.0*(3.0*p_td - 3.0*p_to + dp_td*t_s + 2.0*dp_to*t_s))/(t_s*t_s);

	// If the connection results in an infeasible action, abort and return trapped
	if (isValidAction(a_new) == true)
	{
		// Check if the resulting state action pair is kinematically valid
		bool isValid = (direction == FORWARD) ? (isValidStateActionPair(s_start, a_new, terrain, s_new, t_new)) :
											   (isValidStateActionPairReverse(s_goal,a_new, terrain, s_new, t_new));

		// If valid, great, return REACHED, otherwise try again to the valid state returned by isValidStateActionPair
		if (isValid == true)
			return REACHED;
		else
		{
			if (attemptConnect(s_existing, s_new, t_new, s_new, a_new, terrain, direction) == TRAPPED)
				return TRAPPED;
			else
				return ADVANCED;
		}
	}
	return TRAPPED;
}

int RRTConnectClass::attemptConnect(State s_existing, State s, State &s_new, Action &a_new, FastTerrainMap& terrain, int direction)
{
	// select desired stance time to enforce a nominal stance velocity
	double t_s = poseDistance(s, s_existing)/V_NOM;
	return attemptConnect(s_existing, s, t_s, s_new, a_new, terrain, direction);
}

int RRTConnectClass::connect(PlannerClass &T, State s, FastTerrainMap& terrain, int direction)
{
	// Find nearest neighbor
	int s_near_index = T.getNearestNeighbor(s);
	State s_near = T.getVertex(s_near_index);
	State s_new;
	Action a_new;

	// Try to connect to nearest neighbor, add to graph if REACHED or ADVANCED
	int result = attemptConnect(s_near, s, s_new, a_new, terrain, direction);
	if (result != TRAPPED)
	{
		int s_new_index = T.getNumVertices();
		T.addVertex(s_new_index, s_new);
		T.addEdge(s_near_index, s_new_index);
		T.addAction(s_new_index, a_new);
		T.updateGValue(s_new_index, T.getGValue(s_near_index) + poseDistance(s_near, s_new));
	}

	return result;
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

void RRTConnectClass::postProcessPath(std::vector<State> &state_sequence, std::vector<Action> &action_sequence, FastTerrainMap& terrain)
{
	auto t_start = std::chrono::high_resolution_clock::now();

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
	path_quality_ = 0;

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

		// Try to connect to the last state in the sequence
		// if unsuccesful remove the back and try again until successful or no states left
		while ((attemptConnect(s,s_next, dummy, a_new, terrain, FORWARD) != REACHED) && (s != s_next))
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
			new_action_sequence.push_back(a_new);
			path_quality_ += poseDistance(s,s_next);
			s = s_next;
		} else {
			new_state_sequence.push_back(old_state);
			new_action_sequence.push_back(old_action);
			path_quality_ += poseDistance(s,old_state);
			s = old_state;
		}
	}

	// Replace the old state and action sequences with the new ones
	state_sequence = new_state_sequence;
	action_sequence = new_action_sequence;

	auto t_end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> processing_time = t_end - t_start;

	// std::cout << "Post processing took " << processing_time.count() << std::endl;

}

void RRTConnectClass::runRRTConnect(PlannerClass &Ta, PlannerClass &Tb, FastTerrainMap& terrain)
{
	auto t_start = std::chrono::high_resolution_clock::now();

	while(true)
	{
		auto t_current = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> current_elapsed = t_current - t_start;
		if(current_elapsed.count() >= anytime_horizon) // 0.3 for plinth
		{
			anytime_horizon = anytime_horizon*horizon_expansion_factor;
			std::cout << "Failed, retrying with horizon of " << anytime_horizon << "s" << std::endl;
			return;
		}
		// Generate random s
		State s_rand = Ta.randomState(terrain);

		static int i = 0;

		if (isValidState(s_rand, terrain, STANCE))
		{
			if (extend(Ta, s_rand, terrain, FORWARD) != TRAPPED)
			{
				State s_new = Ta.getVertex(Ta.getNumVertices()-1);

				if(connect(Tb, s_new, terrain, REVERSE) == REACHED)
				{
					goal_found = true;

					auto t_end = std::chrono::high_resolution_clock::now();
					elapsed_to_first = t_end - t_start;
					path_quality_ = Ta.getGValue(Ta.getNumVertices()-1) + Tb.getGValue(Tb.getNumVertices()-1);
					break;
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
					goal_found = true;

					auto t_end = std::chrono::high_resolution_clock::now();
					elapsed_to_first = t_end - t_start;
					path_quality_ = Ta.getGValue(Ta.getNumVertices()-1) + Tb.getGValue(Tb.getNumVertices()-1);
					break;
				}
			}
		}
	}
}


void RRTConnectClass::buildRRTConnect(FastTerrainMap& terrain, State s_start, State s_goal, std::vector<State> &state_sequence, std::vector<Action> &action_sequence, double max_time_opt)
{
	auto t_start = std::chrono::high_resolution_clock::now();
	success_ = 0;

	cost_vector_.clear();
	cost_vector_times_.clear();

	std::cout << "RRT Connect" << std::endl;

	goal_found = false;
	PlannerClass Ta;
	PlannerClass Tb;
	PlannerClass Ta_best;
	PlannerClass Tb_best;

	// Initialize anytime horizon
	// anytime_horizon = 0.3;
	anytime_horizon = poseDistance(s_start, s_goal)/planning_rate_estimate;
	num_vertices = 0;

	double cost_so_far = INFTY;
	while (true)
	{
		Ta = PlannerClass();
		Tb = PlannerClass();
		Ta.init(s_start);
		Tb.init(s_goal);

		goal_found = false;
		runRRTConnect(Ta, Tb, terrain);	

		num_vertices += (Ta.getNumVertices() + Tb.getNumVertices());

		auto t_current = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> current_elapsed = t_current - t_start;
		if(current_elapsed.count() >= max_time_solve)
		{
			std::cout << "Failed, exiting" << std::endl;
			elapsed_total = current_elapsed;
			elapsed_to_first = current_elapsed;
			success_ = 0;
			num_vertices += (Ta.getNumVertices() + Tb.getNumVertices());
			return;
		}

		if (goal_found == true)
		{
			// Get both paths, remove the back of path_b and reverse it to align with path a
			std::vector<int> path_a = pathFromStart(Ta, Ta.getNumVertices()-1);
			std::vector<int> path_b = pathFromStart(Tb, Tb.getNumVertices()-1);

			std::reverse(path_b.begin(), path_b.end());
			std::vector<Action> action_sequence_b = getActionSequenceReverse(Tb, path_b);
			path_b.erase(path_b.begin());

			state_sequence = getStateSequence(Ta, path_a);
			std::vector<State> state_sequence_b = getStateSequence(Tb, path_b);
			state_sequence.insert(state_sequence.end(), state_sequence_b.begin(), state_sequence_b.end());

			action_sequence = getActionSequence(Ta, path_a);
			action_sequence.insert(action_sequence.end(), action_sequence_b.begin(), action_sequence_b.end());

			postProcessPath(state_sequence, action_sequence, terrain);

			if (path_quality_ < cost_so_far)
			{
				cost_so_far = path_quality_;
				Ta_best = Ta;
				Tb_best = Tb;

				t_current = std::chrono::high_resolution_clock::now();
				current_elapsed = t_current - t_start;

				cost_vector_.push_back(cost_so_far);
				cost_vector_times_.push_back(current_elapsed.count());

				// std::cout << "New best cost: " << cost_so_far << " at " << current_elapsed.count() << "s" << std::endl;

			}
		}

		if ((goal_found == true) && (current_elapsed.count() >= max_time_opt))
			break;
	}

	Ta = Ta_best;
	Tb = Tb_best;

	// while (goal_found == false)
	// {
	// 	auto t_current = std::chrono::high_resolution_clock::now();
	// 	std::chrono::duration<double> current_elapsed = t_current - t_start;
	// 	if(current_elapsed.count() >= max_time_solve)
	// 	{
	// 		std::cout << "Failed, exiting" << std::endl;
	// 		elapsed_total = current_elapsed;
	// 		elapsed_to_first = current_elapsed;
	// 		success_ = 0;
	// 		num_vertices += (Ta.getNumVertices() + Tb.getNumVertices());
	// 		return;
	// 	}


	// 	Ta = PlannerClass();
	// 	Tb = PlannerClass();
	// 	Ta.init(s_start);
	// 	Tb.init(s_goal);

	// 	runRRTConnect(Ta, Tb, terrain);	

	// 	num_vertices += (Ta.getNumVertices() + Tb.getNumVertices());
	// }
	
	// std::cout << "Number of samples: " << num_vertices << std::endl;
	
	if (goal_found == true)
	{
		// Get both paths, remove the back of path_b and reverse it to align with path a
		std::vector<int> path_a = pathFromStart(Ta, Ta.getNumVertices()-1);
		std::vector<int> path_b = pathFromStart(Tb, Tb.getNumVertices()-1);

		std::reverse(path_b.begin(), path_b.end());
		std::vector<Action> action_sequence_b = getActionSequenceReverse(Tb, path_b);
		path_b.erase(path_b.begin());

		state_sequence = getStateSequence(Ta, path_a);
		std::vector<State> state_sequence_b = getStateSequence(Tb, path_b);
		state_sequence.insert(state_sequence.end(), state_sequence_b.begin(), state_sequence_b.end());

		action_sequence = getActionSequence(Ta, path_a);
		action_sequence.insert(action_sequence.end(), action_sequence_b.begin(), action_sequence_b.end());
	} else {
		std::cout << "Path not found" << std::endl;
	}

	// cost_vector_.push_back(path_quality_);
	// std::cout << "Path quality = " << path_quality_ << std::endl;

	postProcessPath(state_sequence, action_sequence, terrain);

	// cost_vector_.push_back(path_quality_);
	// std::cout << "Path quality = " << path_quality_ << std::endl;

	auto t_end = std::chrono::high_resolution_clock::now();
	elapsed_total = t_end - t_start;

	if (elapsed_total.count() <= 5.0)
		success_ = 1;

	path_duration_ = 0.0;
    for (Action a : action_sequence)
    {
    	path_duration_ += (a[6] + a[7]);
    }

	// cost_vector_times_.push_back(elapsed_total.count());
	// path_quality_ = Ta.getGValue(Ta.getNumVertices()-1) + Tb.getGValue(Tb.getNumVertices()-1);
	
}

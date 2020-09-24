#include "global_body_planner/planner_class.h"
#include <queue>
#include <chrono>

//constructor
PlannerClass::PlannerClass(){}
//destructor
PlannerClass::~PlannerClass(){}

typedef std::pair<double, int> Distance; 

State PlannerClass::randomState(FastTerrainMap& terrain)
{
    // auto t_start = std::chrono::high_resolution_clock::now();
    // static int num_calls = 0;
    // num_calls++;

    double x_min = terrain.getXData().front();
    double x_max = terrain.getXData().back();
    double y_min = terrain.getYData().front();
    double y_max = terrain.getYData().back();
    // double x_length = x_max - x_min;
    // double y_length = y_max - y_min;
    // grid_map::Index max_idx = {0,0};
    // grid_map::Index min_idx = {terrain.getSize()(0)-1,terrain.getSize()(1)-1};
    // grid_map::Position max_pos;
    // terrain.getPosition(max_idx,max_pos);
    // grid_map::Position min_pos;
    // terrain.getPosition(min_idx,min_pos);
    // double x_max = max_pos[0];
    // double y_max = max_pos[1];
    // double x_min = min_pos[0];
    // double y_min = min_pos[1];
    // double y_min = 0;
    // double y_max = 0;
    double z_min_rel = H_MIN + ROBOT_H;   // defined in functions.h
    double z_max_rel = H_MAX + ROBOT_H;   // defined in functions.h

    // double dx_max = DX_MAX;     // defined in functions.h
    // double dx_min = -dx_max;
    // double dy_max = DX_MAX;
    // double dy_min = -dy_max;
    // double dz_max = DX_MAX;
    // double dz_min = -dz_max;

    State q;

    // Normal distribution sampling
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::normal_distribution<double> height_distribution(0.5*(z_max_rel + z_min_rel),(z_max_rel - z_min_rel)*(1.0/(2*3.0))); // std is such that the max and min are 3 std away from mean
    // std::normal_distribution<double> vel_distribution(0.0,(V_MAX/4.0)); // std is such that the max and min are 4 std away from mean
    std::normal_distribution<double> ang_vel_distribution(0.0,(P_MAX/3.0)); // std is such that the max and min are 3 std away from mean

    q[0] = (x_max - x_min)*(double)rand()/RAND_MAX + x_min;
    q[1] = (y_max - y_min)*(double)rand()/RAND_MAX + y_min;

    q[2] = std::max(std::min(height_distribution(generator), z_max_rel), z_min_rel) + terrain.getGroundHeight(q[0], q[1]);

    // while(q[2] < -2.0)
    // {
    //     q[0] = (x_max - x_min)*(double)rand()/RAND_MAX + x_min;
    //     q[1] = (y_max - y_min)*(double)rand()/RAND_MAX + y_min;
    //     q[2] = std::max(std::min(height_distribution(generator), z_max_rel), z_min_rel) + getGroundHeight(q[0], q[1], terrain);
    // }

    double phi = (2.0*MY_PI)*(double)rand()/RAND_MAX;
    double cos_theta = 2.0*(double)rand()/RAND_MAX - 1.0;
    double theta = acos(cos_theta);
    double v = (double)rand()/RAND_MAX*V_MAX;
    q[3] = v*sin(theta)*cos(phi);
    q[4] = v*sin(theta)*sin(phi);
    // q[3] = v*sin(theta);
    // q[4] = 0;
    // q[5] = v*cos(theta);

    q[6] = 2*P_MAX*(double)rand()/RAND_MAX - P_MAX;
    // q[7] = std::max(std::min(ang_vel_distribution(generator), DP_MAX), -DP_MAX);
    q[7] = 0.0;

    // q[3] = std::max(std::min(vel_distribution(generator), V_MAX), -V_MAX);
    // double dy_max = sqrt(V_MAX*V_MAX - q[3]*q[3]);
    // q[4] = std::max(std::min(vel_distribution(generator), dy_max), -dy_max);
    // q[5] = std::max(std::min(vel_distribution(generator), V_MAX), -V_MAX);

    // auto t_end = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> elapsed = t_end - t_start;
    // std::cout << "Random State took: " << elapsed.count() << "s, num calls: " << num_calls << std::endl;

    return q;
    
}

std::vector<int> PlannerClass::neighborhoodN(State q, int N)
{
	std::priority_queue<Distance, std::vector<Distance>, std::greater<Distance>> closest;

	std::unordered_map<int, State >:: iterator itr;
    for (itr = vertices.begin(); itr != vertices.end(); itr++)
    { 
        closest.push(std::make_pair(stateDistance(q,itr->second), itr->first));
        // closest.push(std::make_pair(poseDistance(q,itr->second), itr->first));
    }

    if (N > closest.size()) N = closest.size();
    std::vector<int> neighbors;
    for (int i = 0; i < N; i++)
    {
    	neighbors.push_back(closest.top().second);
    	// std::cout << "Neighbor " << i << ":" << closest.top().second << std::endl;
    	closest.pop();
    }

    return neighbors;
}

std::vector<int> PlannerClass::neighborhoodDist(State q, double dist)
{
    std::vector<int> neighbors;

    std::unordered_map<int, State >:: iterator itr;
    for (itr = vertices.begin(); itr != vertices.end(); itr++)
    { 
        if ((stateDistance(q,itr->second) <= dist) && stateDistance(q,itr->second) > 0) // don't include itself
        // if ((poseDistance(q,itr->second) <= dist) && poseDistance(q,itr->second) > 0) // don't include itself
        {
            neighbors.push_back(itr->first);
        }
    }

    return neighbors;
}

int PlannerClass::getNearestNeighbor(State q)
{
	std::vector<int> closest_q = neighborhoodN(q,1);
	return closest_q.front();
}
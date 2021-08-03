#include "global_body_planner/planner_class.h"
#include <queue>
#include <chrono>

//constructor
PlannerClass::PlannerClass(){}
//destructor
PlannerClass::~PlannerClass(){}

using namespace planning_utils;

typedef std::pair<double, int> Distance; 

State PlannerClass::randomState(const PlannerConfig &planner_config)
{

    double x_min = planner_config.terrain.getXData().front();
    double x_max = planner_config.terrain.getXData().back();
    double y_min = planner_config.terrain.getYData().front();
    double y_max = planner_config.terrain.getYData().back();

    double z_min_rel = planner_config.H_MIN + planner_config.ROBOT_H;
    double z_max_rel = planner_config.H_MAX + planner_config.ROBOT_H;

    State q;

    // Normal distribution sampling
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::normal_distribution<double> height_distribution(0.5*(z_max_rel + z_min_rel),(z_max_rel - z_min_rel)*(1.0/(2*3.0))); // std is such that the max and min are 3 std away from mean

    q[0] = (x_max - x_min)*(double)rand()/RAND_MAX + x_min;
    q[1] = (y_max - y_min)*(double)rand()/RAND_MAX + y_min;
    q[2] = std::max(std::min(height_distribution(generator), z_max_rel), z_min_rel) + planner_config.terrain.getGroundHeight(q[0], q[1]);


    double phi = (2.0*MY_PI)*(double)rand()/RAND_MAX;
    double cos_theta = 2.0*(double)rand()/RAND_MAX - 1.0;
    double theta = acos(cos_theta);
    // double v = (double)rand()/RAND_MAX*planner_config.V_MAX;
    double v = planner_config.V_NOM;
    q[3] = v*sin(theta)*cos(phi);
    q[4] = v*sin(theta)*sin(phi);
    q[5] = v*cos(theta);

    return q;
    
}

std::vector<int> PlannerClass::neighborhoodN(State q, int N)
{
	std::priority_queue<Distance, std::vector<Distance>, std::greater<Distance>> closest;

	std::unordered_map<int, State >:: iterator itr;
    for (itr = vertices.begin(); itr != vertices.end(); itr++)
    { 
        closest.push(std::make_pair(stateDistance(q,itr->second), itr->first));
    }

    if (N > closest.size()) N = closest.size();
    std::vector<int> neighbors;
    for (int i = 0; i < N; i++)
    {
    	neighbors.push_back(closest.top().second);
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
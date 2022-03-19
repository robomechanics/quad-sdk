#include "global_body_planner/planner_class.h"

#include <chrono>
#include <queue>

using namespace planning_utils;

typedef std::pair<double, int> Distance;

// constructor
PlannerClass::PlannerClass(int direction) { direction_ = direction; }

// destructor
PlannerClass::~PlannerClass() {}

State PlannerClass::randomState(const PlannerConfig &planner_config) {
  double eps = 1;
  double x_min, x_max, y_min, y_max;
  getMapBounds(planner_config, x_min, x_max, y_min, y_max);

  double z_min_rel = planner_config.H_MIN + planner_config.ROBOT_H;
  double z_max_rel = planner_config.H_MAX + planner_config.ROBOT_H;

  State q;

  // Normal distribution sampling
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::normal_distribution<double> height_distribution(
      0.5 * (z_max_rel + z_min_rel),
      (z_max_rel - z_min_rel) *
          (1.0 / (2 * 3.0)));  // std is such that the max and min are 3 std
                               // away from mean

  q.pos[0] = (x_max - x_min) * (double)rand() / RAND_MAX + x_min;
  q.pos[1] = (y_max - y_min) * (double)rand() / RAND_MAX + y_min;
  q.pos[2] = planner_config.H_NOM + getTerrainZFromState(q, planner_config);

  double phi = (2.0 * MY_PI) * (double)rand() / RAND_MAX;
  double v = planner_config.V_NOM;
  q.vel[0] = v * cos(phi);
  q.vel[1] = v * sin(phi);
  q.vel[2] = getDzFromState(q, planner_config);

  return q;
}

std::vector<int> PlannerClass::neighborhoodN(State q, int N) const {
  std::priority_queue<Distance, std::vector<Distance>, std::greater<Distance>>
      closest;

  std::unordered_map<int, State>::const_iterator itr;
  for (itr = vertices.begin(); itr != vertices.end(); itr++) {
    closest.push(std::make_pair(stateDistance(q, itr->second), itr->first));
  }

  if (N > closest.size()) N = closest.size();
  std::vector<int> neighbors;
  for (int i = 0; i < N; i++) {
    neighbors.push_back(closest.top().second);
    closest.pop();
  }

  return neighbors;
}

std::vector<int> PlannerClass::neighborhoodDist(State q, double dist) const {
  std::vector<int> neighbors;

  std::unordered_map<int, State>::const_iterator itr;
  for (itr = vertices.begin(); itr != vertices.end(); itr++) {
    if ((stateDistance(q, itr->second) <= dist) &&
        stateDistance(q, itr->second) > 0) {
      neighbors.push_back(itr->first);
    }
  }

  return neighbors;
}

int PlannerClass::getNearestNeighbor(State q) const {
  std::vector<int> closest_q = neighborhoodN(q, 1);
  return closest_q.front();
}

#ifndef PLANNERCLASS_H
#define PLANNERCLASS_H

#include "global_body_planner/graph_class.hpp"

using namespace planning_utils;

typedef std::pair<double, int> Distance;

//! A directed graph class with supplemental methods to aid in sample-based
//! planning.
/*!
   This class inherits GraphClass, and adds method to add random states to the
   graph and search for neighbors. These functions are useful for sample-based
   planners such as RRTs or PRMs.
*/
class PlannerClass : public GraphClass {
 public:
  /**
   * @brief Constructor for PlannerClass
   * @param[in] direction Direction of tree expansion
   * @param[in] planner_config Configuration structure used for sampling params
   * @return Constructed object of type PlannerClass
   */
  PlannerClass(int direction, const PlannerConfig& planner_config);

  /**
   * @brief Generate a random state by sampling from within the bounds of the
   * terrain
   * @param[in] terrain Height map of the terrain
   * @return Newly generated random state
   */
  State randomState(const PlannerConfig& planner_config);

  /**
   * @brief Get the closest N vertices to the specified state by Euclidean
   * distance.
   * @param[in] s State to query the neighborhood
   * @param[in] N Number of vertices to include in the neighborhood
   * @return Vector of indices included in the neighborhood
   */
  std::vector<int> neighborhoodN(State s, int N) const;

  /**
   * @brief Get the vertices within the specified Euclidean distance of the
   * specified state
   * @param[in] s State to query the neighborhood
   * @param[in] dist distance threshold for the neighborhood
   * @return Vector of indices included in the neighborhood
   */
  std::vector<int> neighborhoodDist(State q, double dist) const;

  /**
   * @brief Get the closest verticex to the specified state by Euclidean
   * distance.
   * @param[in] s State to query the neighborhood
   * @return Index of closest vertex
   */
  int getNearestNeighbor(State q) const;

  /**
   * @brief Mark a vertex (and implicitly the actions/edges that emanate from
   * it) as invalid for the purpose of warm-started replans. Invalid vertices
   * are skipped during nearest-neighbor lookups so they cannot be used as
   * extension origins or appear in returned paths, but they remain in the
   * graph so their valid descendants are not lost.
   * @param[in] idx Vertex index to mark invalid.
   */
  void markVertexInvalid(int idx);

  /**
   * @brief Test whether a vertex was previously marked invalid.
   */
  bool isVertexInvalid(int idx) const;

  /**
   * @brief Re-enable every vertex (called when starting a fresh plan).
   */
  void resetInvalidVertices();

  /**
   * @brief Walk every vertex currently in the tree and mark any that violate
   * the dynamic constraints stored on the supplied planner config. Used to
   * warm-start a replan after CBS has added new constraints — vertices that
   * would now sit inside another robot's body are pruned from neighbor
   * queries before RRT-Connect resumes.
   * @param[in] planner_config Configuration carrying dynamic_constraints.
   * @return Number of vertices that were newly marked invalid.
   */
  int pruneByConstraints(const PlannerConfig& planner_config);

  /// Direction
  int direction_ = FORWARD;

  /// Lognormal distribution for velocity sampling
  std::shared_ptr<std::lognormal_distribution<double>> vel_distribution_;

 protected:
  /// Indices of vertices that are not currently usable as extension origins
  /// or path waypoints, kept separate from the tree so warm-starts retain
  /// the valid portion of the graph.
  std::unordered_set<int> invalid_vertices_;
};

#endif

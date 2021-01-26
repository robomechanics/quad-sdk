#ifndef PLANNERCLASS_H
#define PLANNERCLASS_H

#include "global_body_planner/graph_class.h"

using namespace planning_utils;

//! A directed graph class with supplemental methods to aid in sample-based planning.
/*!
   This class inherits GraphClass, and adds method to add random states to the graph and search for neighbors.
   These functions are useful for sample-based planners such as RRTs or PRMs.
*/
class PlannerClass : public GraphClass
{
  public:
    /**
     * @brief Constructor for PlannerClass
     * @return Constructed object of type PlannerClass
     */
    PlannerClass();

    /**
     * @brief Destructor for PlannerClass
     */
    ~PlannerClass();

    /**
     * @brief Generate a random state by sampling from within the bounds of the terrain
     * @param[in] terrain Height map of the terrain
     * @return Newly generated random state
     */
    State randomState(FastTerrainMap& terrain);
    
    /**
     * @brief Get the closest N vertices to the specified state by Euclidean distance.
     * @param[in] s State to query the neighborhood
     * @param[in] N Number of vertices to include in the neighborhood
     * @return Vector of indices included in the neighborhood
     */
    std::vector<int> neighborhoodN(State s, int N);
    
    /**
     * @brief Get the vertices within the specified Euclidean distance of the specified state
     * @param[in] s State to query the neighborhood
     * @param[in] dist distance threshold for the neighborhood
     * @return Vector of indices included in the neighborhood
     */
    std::vector<int> neighborhoodDist(State q, double dist);
    
    /**
     * @brief Get the closest verticex to the specified state by Euclidean distance.
     * @param[in] s State to query the neighborhood
     * @return Index of closest vertex
     */
    int getNearestNeighbor(State q);

};

#endif
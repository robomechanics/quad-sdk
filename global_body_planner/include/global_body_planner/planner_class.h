#ifndef PLANNERCLASS_H
#define PLANNERCLASS_H

#include "global_body_planner/graph_class.h"

class PlannerClass : public GraphClass
{
    public:
        //constructor
        PlannerClass();

        //destructor
        ~PlannerClass();

        State randomState(FastTerrainMap& terrain);
        std::vector<int> neighborhoodN(State q, int N);
        std::vector<int> neighborhoodDist(State q, double dist);
        int getNearestNeighbor(State q);

};

#endif
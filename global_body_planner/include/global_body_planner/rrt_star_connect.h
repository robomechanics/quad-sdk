#ifndef RRTSTARCONNECT_H
#define RRTSTARCONNECT_H

#include "global_body_planner/rrt_connect.h"
// #include "functions.h"

// #define TRAPPED 0
// #define ADVANCED 1
// #define REACHED 2

class RRTStarConnectClass : public RRTConnectClass
{
    public:
        //constructor
        //const reference pass because the values w and h don't change and reference avoid the time it takes to copy large
        //  objects by value (if there were any)
        RRTStarConnectClass();

        //destructor
        ~RRTStarConnectClass();

        int extend(PlannerClass &T, State s, FastTerrainMap& terrain, int direction);
        void getStateAndActionSequences(PlannerClass &Ta, PlannerClass &Tb, 
            int shared_a_idx, int shared_b_idx, 
            std::vector<State> &state_sequence, std::vector<Action> &action_sequence);

        void buildRRTStarConnect(FastTerrainMap& terrain, State s_start, State s_goal,
            std::vector<State> &state_sequence, std::vector<Action> &action_sequence, double max_time);

    protected:
        const double delta_max = 0.5;
        double delta = 3.0;
        double elapsed_so_far_total;
};

#endif
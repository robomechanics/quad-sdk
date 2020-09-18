#ifndef RRTCONNECT_H
#define RRTCONNECT_H

#include "global_body_planner/rrt.h"
// #include "functions.h"

#define TRAPPED 0
#define ADVANCED 1
#define REACHED 2

class RRTConnectClass : public RRTClass
{
    public:
        //constructor
        RRTConnectClass();

        //destructor
        ~RRTConnectClass();

        int attemptConnect(State s_existing, State s, double t_s, State &s_new, Action &a_new, Ground &ground, int direction);
        int attemptConnect(State s_existing, State s, State &s_new, Action &a_new, Ground &ground, int direction);
        int connect(PlannerClass &T, State s, Ground &ground, int direction);
        std::vector<Action> getActionSequenceReverse(PlannerClass &T, std::vector<int> path);
        void postProcessPath(std::vector<State> &state_sequence, std::vector<Action> &action_sequence, Ground& ground);
        void runRRTConnect(PlannerClass &Ta, PlannerClass &Tb, Ground &ground);
        void buildRRTConnect(Ground &ground, State s_start, State s_goal, std::vector<State> &state_sequence, std::vector<Action> &action_sequence, double max_time);

    protected:
        double anytime_horizon;
        const double planning_rate_estimate = 16.0; // m/s (meters planned/computation time)
        double anytime_horizon_init;
        double horizon_expansion_factor = 1.2;
        const int max_time_solve = 120;

};

#endif
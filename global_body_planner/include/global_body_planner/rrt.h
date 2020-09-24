#ifndef RRT_H
#define RRT_H

#include "global_body_planner/planner_class.h"
#include <chrono>

#define TRAPPED 0
#define ADVANCED 1
#define REACHED 2

class RRTClass
{
    public:
        //constructor
        RRTClass();

        //destructor
        ~RRTClass();

        virtual int extend(PlannerClass &T, State s, FastTerrainMap& terrain, int direction);
        std::vector<int> pathFromStart(PlannerClass &T, int s);
        void printPath(PlannerClass &T, std::vector<int> path);
        void buildRRT(FastTerrainMap& terrain, State s_start, State s_goal, std::vector<State> &state_sequence, std::vector<Action> &action_sequence);
        void getStatistics(double &plan_time, int &success_var, int &vertices_generated, double &time_to_first_solve, std::vector<double> &cost_vector, std::vector<double> &cost_vector_times, double& path_duration);

        bool newConfig(State s, State s_near, State &s_new, Action &a_new, FastTerrainMap& terrain, int direction);

        std::vector<State> getStateSequence(PlannerClass &T, std::vector<int> path);
        std::vector<Action> getActionSequence(PlannerClass &T, std::vector<int> path);


    protected:
        const double prob_goal_thresh = 0.05;
        bool goal_found = false;

        // Statistics
        std::chrono::duration<double> elapsed_total;
        std::chrono::duration<double> elapsed_to_first;
        int success_ = 0;
        int num_vertices;
        double path_quality_;
        std::vector<double> cost_vector_;
        std::vector<double> cost_vector_times_;
        double path_duration_;

        
        
};

#endif
#ifndef GRAPHCLASS_H
#define GRAPHCLASS_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include "global_body_planner/functions.h"

class GraphClass
{
    public:
        //constructor
        GraphClass();

        //destructor
        ~GraphClass();

        State getVertex(int index);
        int getNumVertices();
        void addVertex(int index, State q);
        virtual void addEdge(int idx1, int idx2);
        void removeEdge(int idx1, int idx2);
        virtual int getPredecessor(int idx);
        std::vector<int> getSuccessors(int idx);
        void addAction(int idx, Action a);
        Action getAction(int idx);
        
        double getGValue(int idx);
        void updateGValue(int idx, double val);

        void printVertex(State vertex);
        void printVertices();
        void printIncomingEdges(int in_vertex);
        virtual void printEdges();

        virtual void init(State q);
        // double vertex_distance(std::vector<double> q1,std::vector<double> q2);
        // double vertex_distance(int q1,int q2);


    protected:
        std::unordered_map<int, State > vertices;
        std::unordered_map<int, Action > actions;
        std::unordered_map<int, std::vector<int> > edges;
        std::unordered_map<int, double> g_values;

        std::unordered_map<int, std::vector<int> > successors;
};

#endif
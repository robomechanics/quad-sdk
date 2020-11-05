#include "global_body_planner/graph_class.h"

#include <vector>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <math.h>

using namespace planning_utils;

GraphClass::GraphClass(){}
GraphClass::~GraphClass(){}

State GraphClass::getVertex(int idx)
{
    return vertices[idx];
}

int GraphClass::getNumVertices()
{
    return vertices.size();
}

void GraphClass::addVertex(int idx, State q)
{
    vertices[idx] = q;
    return;
}

void GraphClass::addEdge(int idx1, int idx2)
{
    edges[idx2].push_back(idx1);
    successors[idx1].push_back(idx2);
    g_values[idx2] = g_values[idx1] + poseDistance(vertices[idx1], vertices[idx2]);
    return;
}

void GraphClass::removeEdge(int idx1, int idx2)
{
    std::vector<int>::iterator itr;
    for (itr = edges[idx2].begin(); itr != edges[idx2].end(); itr++)
    {
        if (*itr == idx1)
        {
            edges[idx2].erase(itr);
            break;
        }
    }
    for (itr = successors[idx1].begin(); itr != successors[idx1].end(); itr++)
    {
        if (*itr == idx2)
        {
            successors[idx1].erase(itr);
            break;
        }
    }
    return;
}

int GraphClass::getPredecessor(int idx)
{
    if (edges[idx].size() > 1)
    {
        std::cout << "More than one predecessor, fix this!" << std::endl;
        throw("Error");
    }
    return edges[idx].front();
}

std::vector<int> GraphClass::getSuccessors(int idx)
{
    return successors[idx];
}

void GraphClass::addAction(int idx, Action a)
{
    actions[idx] = a;
}

Action GraphClass::getAction(int idx)
{
    return actions[idx];
}

void GraphClass::printVertex(State vertex)
{
    std::cout << "{";
    for (int i= 0; i < vertex.size(); i++)
        std::cout << vertex[i] << ", ";
    std::cout << "\b\b}"; 
}

void GraphClass::printVertices()
{
    std::unordered_map<int, State >:: iterator itr;
    std::cout << "\nAll Vertices : \n";
    for (itr = vertices.begin(); itr != vertices.end(); itr++)
    { 
        std::cout << itr->first << " ";
        printVertex(itr->second);
        std::cout << std::endl;
    } 
}

void GraphClass::printIncomingEdges(int in_vertex)
{
    std::vector<int>::iterator itr;
    for (int i = 0; i < edges[in_vertex].size(); i++)
    {
        int out_vertex = edges[in_vertex][i];
        std::cout << "{" << out_vertex << " -> " << in_vertex << "} or ";
        printState(vertices[out_vertex]);
        std::cout << " -> ";
        printState(vertices[in_vertex]);
        std::cout << std::endl;
    }
}

void GraphClass::printEdges()
{
    std::cout << "All Edges : \n";
    for (int i= 0; i < edges.size(); i++)
    {
        printIncomingEdges(i);
    }
}

double GraphClass::getGValue(int idx)
{
    return g_values[idx];
}

void GraphClass::updateGValue(int idx, double val)
{
    g_values[idx] = val;
    for (int successor : getSuccessors(idx))
    {
        updateGValue(successor, g_values[idx] + poseDistance(getVertex(idx), getVertex(successor)));
    }
}

void GraphClass::init(State q)
{
    int q_init = 0;
    addVertex(q_init, q);
    g_values[q_init] = 0;
    return;
}
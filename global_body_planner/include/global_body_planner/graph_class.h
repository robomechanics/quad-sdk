#ifndef GRAPHCLASS_H
#define GRAPHCLASS_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include "global_body_planner/planning_utils.h"

using namespace planning_utils;

//! A general directed graph class.
/*!
   This class implements a directed graph data structure with methods for adding and deleting vertices and edges,
   as well storing information at each vertex and providing print statements for debugging. Vertices are
   indexed with ints, and edges as unordered maps that map a vertex's index to the indices of its parents. If the
   graph is a tree there should only be one parent per vertex. Other unordered maps store information about each
   vertex, such as the associated state, action, or distance from the root vertex (g-value).
*/
class GraphClass
{
  public:
    /**
     * @brief Constructor for GraphClass
     * @return Constructed object of type GraphClass
     */
    GraphClass();

    /**
     * @brief Destructor for GraphClass
     */
    ~GraphClass();

    /**
     * @brief Add a new vertex to the graph along with its state data
     * @param[in] index Index of the new vertex
     * @param[in] s State information corresponding to the specified index
     */
    void addVertex(int index, State s);

    /**
     * @brief Retrieve the state stored at a particular index in the graph
     * @param[in] index Index of the desired vertex
     * @return State information corresponding to the requested index
     */
    State getVertex(int index);
    
    /**
     * @brief Retrieve the total number of vertices in the graph, computed as the size of the vertices vector
     * @return Number of vertices in the graph
     */
    int getNumVertices();
    
    /**
     * @brief Add a new edge to the graph
     * @param[in] idx1 Index of the outgoing vertex of the edge
     * @param[in] idx2 Index of the incoming vertex of the edge
     */
    virtual void addEdge(int idx1, int idx2);
    
    /**
     * @brief Remove an edge of the graph
     * @param[in] idx1 Index of the outgoing vertex of the edge
     * @param[in] idx2 Index of the incoming vertex of the edge
     */
    void removeEdge(int idx1, int idx2);
    
    /**
     * @brief Get the parent of a vertex
     * @param[in] idx Index of the desired vertex
     * @return Index of the parent of the specified vertex
     */
    virtual int getPredecessor(int idx);
    
    /**
     * @brief Get the children of a vertex
     * @param[in] idx Index of the desired vertex
     * @return Indices of the children of the specified vertex
     */
    std::vector<int> getSuccessors(int idx);
    
    /**
     * @brief Add an action to a particular vertex. This is the action that lead to this particular vertex from its parent.
     * @param[in] idx Index of the desired vertex
     * @param[in] a Action corresponding to the desired vertex
     */
    void addAction(int idx, Action a);
    
    /**
     * @brief Get the action of a vertex
     * @param[in] idx Index of the desired vertex
     * @return Action corresponding to the desired vertex
     */
    Action getAction(int idx);
    
    /**
     * @brief Update the g-value of a vertex and propogate to all its successors
     * @param[in] idx Index of the desired vertex
     * @param[in] val New g-value corresponding to the desired vertex
     */
    void updateGValue(int idx, double val);

    /**
     * @brief Get the g-value of a vertex
     * @param[in] idx Index of the desired vertex
     * @return G-value corresponding to the desired vertex
     */
    double getGValue(int idx);

    /**
     * @brief Print the state information via stdout
     * @param[in] s The state information to print
     */
    void printVertex(State s);
    
    /**
     * @brief Print the all vertices in the graph via stdout
     */
    void printVertices();
    
    /**
     * @brief Print the edges leading to a vertex via stdout
     * @param[in] idx Index of the desired vertex
     */
    void printIncomingEdges(int idx);
    
    /**
     * @brief Print the all edges in the graph via stdout
     */
    virtual void printEdges();

    /**
     * @brief Initialize the graph by adding the root vertex (idx = 0) and setting g(idx) = 0
     * @param[in] s State for the root vertex
     */
    virtual void init(State s);


  protected:
    /// Map from vertex indices to corresponding states
    std::unordered_map<int, State > vertices;

    /// Map from vertex indices to the actions leading to those vertices
    std::unordered_map<int, Action > actions;

    /// Map from vertex indices to their parent(s)
    std::unordered_map<int, std::vector<int> > edges;

    /// Map from vertex indices to their children
    std::unordered_map<int, std::vector<int> > successors;

    /// Map from vertex indices to their costs (g-values)
    std::unordered_map<int, double> g_values;
};

#endif
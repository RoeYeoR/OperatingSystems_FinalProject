#ifndef MST_STRATEGY_HPP
#define MST_STRATEGY_HPP

#include "graph.hpp"
#include <vector>

class MSTStrategy {

public:
    virtual std::vector<Graph::Edge> solve(const Graph& graph) = 0;
    virtual int totalWeight(const Graph& graph) = 0;
    virtual int longestDistance(const Graph& graph) = 0;      
    virtual double averageDistance(const Graph& graph) = 0;  
    virtual int shortestDistance(const Graph& graph, int u, int v) = 0; 
   virtual Graph& getGraph(std::vector<Graph::Edge> mst) = 0; 

    virtual ~MSTStrategy() = default;
};

#endif // MST_STRATEGY_HPP

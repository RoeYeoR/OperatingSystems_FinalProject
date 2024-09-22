#include <vector>
#include "graph.hpp"
#include "disjoint_set.hpp" 
#include "mst_strategy.hpp"

class KruskalMST : public MSTStrategy {
public:
    std::vector<Graph::Edge> solve(const Graph& graph) override;
    int totalWeight(const Graph& graph) override;
    int longestDistance(const Graph& graph) override;
    double averageDistance(const Graph& graph) override;
    int shortestDistance(const Graph& graph, int u, int v) override;

private:
    std::vector<std::vector<std::pair<int, int>>> mstAdjList; // Adjacency list for MST
    int dfsLongestPath(int node, std::vector<bool>& visited);
    int dfsLongestPath(const Graph& graph);
    int bfsShortestPath(const Graph& graph, int start, int end);
};

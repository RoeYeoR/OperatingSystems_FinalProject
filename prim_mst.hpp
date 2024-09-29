#ifndef PRIM_MST_HPP
#define PRIM_MST_HPP

#include "mst_common.hpp"

class PrimMST : public MSTCommon {
public:
    // Implement all the necessary methods
    std::vector<Graph::Edge> solve(const Graph& graph) override;
    Graph& getGraph(std::vector<Graph::Edge> mst) override { return MSTCommon::getGraph(mst); };
    int totalWeight(const Graph& graph) override { return MSTCommon::totalWeight(graph); }
    int longestDistance(const Graph& graph) override { return MSTCommon::longestDistance(graph); }
    double averageDistance(const Graph& graph) override { return MSTCommon::averageDistance(graph); }
    int shortestDistance(const Graph& graph, int u, int v) override { return MSTCommon::shortestDistance(graph, u, v); }
    int dijkstraShortestPath(const Graph& graph, int start, int end) override { return MSTCommon::dijkstraShortestPath(graph, start, end); }
};

#endif // KRUSKAL_MST_HPP
#include "graph.hpp"

Graph::Graph(int V) : V(V) {}

void Graph::addEdge(int u, int v, int w) {
    edges.push_back(Edge(u, v, w));
}

std::vector<Graph::Edge> Graph::getEdges() const {
    return edges;
}

std::vector<Graph::Edge> Graph::getEdgesFromNode(int node) const {
    std::vector<Edge> nodeEdges;
    for (const auto& edge : edges) {
        if (edge.src == node) {
            nodeEdges.push_back(edge);
        }
    }
    return nodeEdges;
}

Graph Graph::createMSTGraph(const std::vector<Edge>& mstEdges) const {
    // Create a new graph with the same number of vertices
    Graph mstGraph(V);
    
    // Add each MST edge to the new graph
    for (const auto& edge : mstEdges) {
        mstGraph.addEdge(edge.src, edge.dest, edge.weight);
    }
    
    return mstGraph;
}

int Graph::getVertices() const {
    return V;
}

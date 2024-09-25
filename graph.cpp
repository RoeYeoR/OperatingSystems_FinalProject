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



int Graph::getVertices() const {
    return V;
}

#include "graph.hpp"

Graph::Graph(int V) : V(V) {}

void Graph::addEdge(int u, int v, int w) {
    edges.push_back(Edge(u, v, w));
}

std::vector<Graph::Edge> Graph::getEdges() const {
    return edges;
}

int Graph::getVertices() const {
    return V;
}

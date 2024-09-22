#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
#include <utility>
#include <limits>

class Graph {
public:
    struct Edge {
        int src, dest, weight;
        Edge(int src, int dest, int weight) : src(src), dest(dest), weight(weight) {}
    };

    Graph(int V);  // Constructor with number of vertices

    void addEdge(int u, int v, int w);  // Add an edge to the graph
    std::vector<Edge> getEdges() const; // Get all edges
    int getVertices() const;

private:
    int V;  // Number of vertices
    std::vector<Edge> edges;  // List of edges
};

#endif // GRAPH_HPP

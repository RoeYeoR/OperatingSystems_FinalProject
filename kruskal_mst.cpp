#include "kruskal_mst.hpp"
#include <algorithm>
#include <vector>
#include <queue>
#include <utility> // For std::pair
#include <climits>
#include "disjoint_set.hpp"
#include <iostream>



std::vector<Graph::Edge> KruskalMST::solve(const Graph& graph) {
    std::vector<Graph::Edge> result;
    std::vector<Graph::Edge> edges = graph.getEdges();  // Get all edges of the graph
    int V = graph.getVertices();  // Get the number of vertices

    // Sort edges by their weight
    std::sort(edges.begin(), edges.end(), [](const Graph::Edge& a, const Graph::Edge& b) {
        return a.weight < b.weight;
    });

    // Initialize Union-Find structure to detect cycles
    DisjointSet ds(V);  // Assuming vertices are zero-indexed

    // Initialize adjacency list if needed
    mstAdjList.resize(V);

    // Iterate over all sorted edges
    for (const auto& edge : edges) {
        int u = edge.src;
        int v = edge.dest;

        // Check if including this edge creates a cycle (find if they are in different sets)
        if (ds.find(u) != ds.find(v)) {
            ds.unionSets(u, v);  // Union the sets containing u and v
            result.push_back(edge);  // Add edge to the result (MST)
            
            // Add to adjacency list (assuming an undirected graph)
            mstAdjList[u].push_back({v, edge.weight});
            mstAdjList[v].push_back({u, edge.weight});
        }

        // Stop when the MST is complete (MST has V-1 edges)
        if (result.size() == V - 1) {
            break;
        }
    }

    // Ensure that the result contains V-1 edges for a valid MST
    if (result.size() != V - 1) {
        throw std::runtime_error("Graph is not connected, no valid MST found");
    }


    return result;  // Return the edges in the MST
}


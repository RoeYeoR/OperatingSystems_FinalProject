#include "prim_mst.hpp"
#include <queue>
#include <functional>
#include <limits>
#include <climits>


std::vector<Graph::Edge> PrimMST::solve(const Graph& graph) {
    std::cout << "Prim Solve Method" << std::endl;

    int V = graph.getVertices();
    std::vector<bool> inMST(V, false);
    std::vector<Graph::Edge> result;
    mstAdjList.resize(V); // Resize the adjacency list

    // Min-heap to choose the minimum weight edge at every step
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<>> pq;
    
    std::vector<int> key(V, INT_MAX); // Key values to pick minimum weight edge
    std::vector<int> parent(V, -1);   // Array to store constructed MST
    pq.push({0, 0}); // Starting from vertex 0 with weight 0
    key[0] = 0;

    while (!pq.empty()) {
        int u = pq.top().second; // Get the vertex u
        pq.pop();

        if (inMST[u]) continue; // Skip if already in MST
        
        inMST[u] = true;

        // Add the edge to the MST (except for the first vertex)
        if (parent[u] != -1) {
            result.push_back({parent[u], u, key[u]});
            // Update adjacency list for MST
            mstAdjList[parent[u]].push_back({u, key[u]});
            mstAdjList[u].push_back({parent[u], key[u]}); // For undirected graph
        }

        // Explore adjacent vertices
        for (const auto& edge : graph.getEdges()) {
            int v = (edge.src == u) ? edge.dest : (edge.dest == u) ? edge.src : -1;
            if (v != -1 && !inMST[v] && edge.weight < key[v]) {
                // Update key and parent if edge weight is smaller than current key[v]
                key[v] = edge.weight;
                parent[v] = u;
                pq.push({key[v], v});
            }
        }
    }

    return result;
}

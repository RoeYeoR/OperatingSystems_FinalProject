#include "prim_mst.hpp"
#include <queue>
#include <functional>
#include <limits>

std::vector<Graph::Edge> PrimMST::solve(const Graph& graph) {
    int V = graph.getVertices();
    std::vector<bool> inMST(V, false);
    std::vector<Graph::Edge> result;
    mstAdjList.resize(V); // Resize the adjacency list

    // Min-heap to choose the minimum weight edge at every step
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<>> pq;
    
    pq.push({0, 0}); // Starting from vertex 0 with weight 0

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();
        
        if (inMST[u]) continue;
        
        inMST[u] = true;

        // Add edges from the current node to the result
        for (const auto& edge : graph.getEdges()) {
            if (edge.src == u && !inMST[edge.dest]) {
                pq.push({edge.weight, edge.dest});
                result.push_back(edge);
                // Update adjacency list for MST
                mstAdjList[u].push_back({edge.dest, edge.weight});
                mstAdjList[edge.dest].push_back({u, edge.weight}); // For undirected graph
            }
        }
    }

    return result;
}

int PrimMST::totalWeight(const Graph& graph) {
    std::vector<Graph::Edge> mst = solve(graph);
    int total_weight = 0;
    for (const auto& edge : mst) {
        total_weight += edge.weight;
    }
    return total_weight;
}

int PrimMST::longestDistance(const Graph& graph) {
    std::vector<bool> visited(graph.getVertices(), false);
    return dfsLongestPath(0, visited); // Start from vertex 0
}

double PrimMST::averageDistance(const Graph& graph) {
    double totalDist = 0;
    int count = 0;
    for (int i = 0; i < graph.getVertices(); ++i) {
        for (int j = i + 1; j < graph.getVertices(); ++j) {
            totalDist += shortestDistance(graph, i, j);
            count++;
        }
    }
    return (count > 0) ? (totalDist / count) : 0.0;
}

int PrimMST::shortestDistance(const Graph& graph, int u, int v) {
    return bfsShortestPath(graph, u, v);
}

// DFS to find longest path
int PrimMST::dfsLongestPath(int node, std::vector<bool>& visited) {
    visited[node] = true;
    int maxDepth = 0;

    for (const auto& neighbor : mstAdjList[node]) { // Use mstAdjList
        if (!visited[neighbor.first]) { // neighbor.first is the vertex
            maxDepth = std::max(maxDepth, dfsLongestPath(neighbor.first, visited) + neighbor.second);
        }
    }

    visited[node] = false; // Backtrack
    return maxDepth;
}

int PrimMST::dfsLongestPath(const Graph& graph) {
    std::vector<bool> visited(graph.getVertices(), false);
    int longestPath = 0;

    for (int i = 0; i < graph.getVertices(); ++i) {
        longestPath = std::max(longestPath, dfsLongestPath(i, visited));
    }

    return longestPath;
}

// BFS to find shortest path
int PrimMST::bfsShortestPath(const Graph& graph, int start, int end) {
    std::queue<std::pair<int, int>> q; // pair<node, current distance>
    std::vector<bool> visited(graph.getVertices(), false);
    q.push({start, 0});
    visited[start] = true;

    while (!q.empty()) {
        auto [node, dist] = q.front();
        q.pop();

        if (node == end) {
            return dist;
        }

        for (const auto& neighbor : mstAdjList[node]) {
            if (!visited[neighbor.first]) {
                visited[neighbor.first] = true;
                q.push({neighbor.first, dist + neighbor.second});
            }
        }
    }

    return -1; // Not reachable
}

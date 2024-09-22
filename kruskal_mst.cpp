#include "kruskal_mst.hpp"
#include <algorithm>
#include <vector>
#include <queue>
#include <utility> // For std::pair


std::vector<Graph::Edge> KruskalMST::solve(const Graph& graph) {
     std::vector<Graph::Edge> result;
    std::vector<Graph::Edge> edges = graph.getEdges();
    int V = graph.getVertices();

    // Sort edges based on their weights
    std::sort(edges.begin(), edges.end(), [](const Graph::Edge& a, const Graph::Edge& b) {
        return a.weight < b.weight;
    });

    // Union-Find structure
    DisjointSet ds(V);

    for (const auto& edge : edges) {
        int u = edge.src;
        int v = edge.dest;

        // If u and v are not in the same set, include this edge in the result
        if (ds.find(u) != ds.find(v)) {
            ds.unionSets(u, v);
            result.push_back(edge);
            mstAdjList[u].push_back({v, edge.weight}); // Add to adjacency list
            mstAdjList[v].push_back({u, edge.weight}); // For undirected graph
        }
    }

    return result;
}

int KruskalMST::totalWeight(const Graph& graph) {
    std::vector<Graph::Edge> mst = solve(graph);
    int total_weight = 0;
    for (const auto& edge : mst) {
        total_weight += edge.weight;
    }
    return total_weight;
}

int KruskalMST::longestDistance(const Graph& graph) {
    std::vector<bool> visited(graph.getVertices(), false);
    return dfsLongestPath(0, visited); // Start from vertex 0
}

double KruskalMST::averageDistance(const Graph& graph) {
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

int KruskalMST::shortestDistance(const Graph& graph, int u, int v) {
    return bfsShortestPath(graph, u, v);
}

// DFS to find longest path
int KruskalMST::dfsLongestPath(int node, std::vector<bool>& visited) {
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

int KruskalMST::dfsLongestPath(const Graph& graph) {
    std::vector<bool> visited(graph.getVertices(), false);
    int longestPath = 0;

    for (int i = 0; i < graph.getVertices(); ++i) {
        longestPath = std::max(longestPath, dfsLongestPath(i, visited));
    }

    return longestPath;
}

// BFS to find shortest path
int KruskalMST::bfsShortestPath(const Graph& graph, int start, int end) {
     std::queue<std::pair<int, int>> q; // pair<node, current distance>
    std::vector<bool> visited(graph.getVertices(), false);
    q.push({start, 0}); // Start from the starting node
    visited[start] = true;

    while (!q.empty()) {
        auto [node, dist] = q.front();
        q.pop();

        if (node == end) {
            return dist; // Return the distance if we reached the end node
        }

        for (const auto& neighbor : mstAdjList[node]) { // Assuming mstAdjList holds the MST
            int nextNode = neighbor.first; // neighbor.first is the vertex
            if (!visited[nextNode]) {
                visited[nextNode] = true;
                q.push({nextNode, dist + neighbor.second}); // Add to queue with updated distance
            }
        }
    }

    return -1; // If the end node is not reachable
}

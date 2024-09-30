#ifndef MST_COMMON_HPP
#define MST_COMMON_HPP

#include "mst_strategy.hpp"
#include <vector>
#include <queue>
#include <climits>
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
class MSTCommon : public MSTStrategy {
public:
    std::vector<Graph::Edge> mst;
    std::vector<std::vector<std::pair<int, int>>> mstAdjList; // Adjacency list for MST
   int totalWeight(const Graph& graph) {
    std::vector<Graph::Edge> mst = solve(graph);
    int total_weight = 0;
    for (const auto& edge : mst) {
        total_weight += edge.weight;
    }
    return total_weight;
}
Graph& getGraph(std::vector<Graph::Edge> mst)
{
      // Create a new Graph with the number of vertices from the original graph
    Graph* mstGraph = new Graph(mst.size() + 1); // +1 for the number of vertices; adjust if needed based on your structure

    // Add edges from the MST vector to the new graph
    for (const auto& edge : mst) {
        mstGraph->addEdge(edge.src, edge.dest, edge.weight);
        // Since it's an undirected graph, add the reverse edge as well
        mstGraph->addEdge(edge.dest, edge.src, edge.weight);
    }

    // Return the new graph containing the MST
    return *mstGraph; // Return a reference to the newly created graph

}
int longestDistance(const Graph& graph) {
    std::vector<bool> visited(graph.getVertices(), false);
    int max =0;
    for (int i = 0; i < graph.getVertices(); i++)
    {
        int temp = dfsLongestPath(i, visited);
        if(temp> max)
        {
            max = temp;
        }
    }
    return max;
}

double averageDistance(const Graph& mst) {
    double totalDist = 0;
    int count = 0;

    // Loop over all pairs of vertices
    for (int i = 0; i < mst.getVertices(); ++i) {
        for (int j = i + 1; j < mst.getVertices(); ++j) {
            // Get the distance between i and j in the MST
            totalDist += mstDistance(mst, i, j); // Use mstDistance to calculate distance in MST
            count++;
        }
    }

    return (count > 0) ? (totalDist / count) : 0.0;
}
int mstDistance(const Graph& mst, int u, int v) {
    // Use BFS or DFS to find the distance between u and v in the MST
    std::vector<bool> visited(mst.getVertices(), false);
    std::queue<std::pair<int, int>> q; // Pair of (node, accumulated distance)
    
    q.push({u, 0});
    visited[u] = true;

    while (!q.empty()) {
        int node = q.front().first;
        int dist = q.front().second;
        q.pop();

        if (node == v) {
            return dist; // We reached the target node, return the distance
        }

        // Traverse all adjacent vertices in the MST
        for (const auto& edge : mst.getEdgesFromNode(node)) {
            int nextNode = edge.dest;
            int weight = edge.weight;

            if (!visited[nextNode]) {
                visited[nextNode] = true;
                q.push({nextNode, dist + weight}); // Add the edge weight to accumulated distance
            }
        }
    }

    return -1; // Return -1 if no path exists (shouldn't happen in an MST)
}


int shortestDistance(const Graph& graph, int u, int v) {
    return dijkstraShortestPath(graph, u, v);
}
int dijkstraShortestPath(const Graph& graph, int start, int end) {
    // Edge case: Check if the start and end vertices are the same
    if (start == end) return 0;

    // Edge case: Validate that the vertices are within the valid range
    int V = graph.getVertices();
    if (start < 0 || start >= V || end < 0 || end >= V) {
        std::cerr << "Error: One or both vertices are not valid." << std::endl;
        return -1;
    }

    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<>> pq; // Min-heap priority queue
    std::vector<int> distances(graph.getVertices(), INT_MAX); // Vector to store shortest distances
    std::vector<bool> visited(graph.getVertices(), false);

    pq.push({0, start}); // Push start node with distance 0
    distances[start] = 0; // Distance to start node is 0

    while (!pq.empty()) {
        int distance = pq.top().first;
        int node = pq.top().second;
        pq.pop();

        if (visited[node]) continue;
        visited[node] = true;

        // Process all neighbors of the current node
        for (const auto& edge : graph.getEdgesFromNode(node)) {
            int nextNode = edge.dest; // Use edge's destination
            int weight = edge.weight;  // Use edge's weight

            // Check if we found a shorter path
            if (distances[node] + weight < distances[nextNode]) {
                distances[nextNode] = distances[node] + weight;
                pq.push({distances[nextNode], nextNode});
            }
        }
    }

    return distances[end] == INT_MAX ? -1 : distances[end]; // Return -1 if not reachable
}

protected:
    int dfsLongestPath(int node, std::vector<bool>& visited) {
    visited[node] = true; // Mark the current node as visited
    int maxDistance = 0;  // Initialize the maximum distance

    // Iterate through all edges originating from the current node
    for (const auto& edge : mstAdjList[node]) {
        int neighbor = edge.first; // Neighbor node
        int weight = edge.second;   // Weight of the edge

        if (!visited[neighbor]) { // If the neighbor hasn't been visited
            // Recursively call DFS and calculate the total distance
            int distance = weight + dfsLongestPath(neighbor, visited);
            maxDistance = std::max(maxDistance, distance); // Update maximum distance
        }
    }

    visited[node] = false; // Backtrack: unmark the current node
    return maxDistance; // Return the maximum distance found
}

// BFS to find shortest path
int bfsShortestPath(const Graph& graph, int start, int end) {
    std::queue<int> q; // Queue for BFS
    std::vector<int> distances(graph.getVertices(), INT_MAX); // Vector to store shortest distances
    std::vector<bool> visited(graph.getVertices(), false);
    
    q.push(start);
    distances[start] = 0; // Distance to start node is 0
    visited[start] = true;

    while (!q.empty()) {
        int node = q.front();
        q.pop();

        for (const auto& edge : graph.getEdgesFromNode(node)) { // Get edges of the current node
            int nextNode = edge.dest; // Use edge's destination
            int weight = edge.weight;  // Use edge's weight

            // Check if we found a shorter path
            if (!visited[nextNode] && distances[node] + weight < distances[nextNode]) {
                distances[nextNode] = distances[node] + weight;
                q.push(nextNode);
                visited[nextNode] = true;
            }
        }
    }

    return distances[end] == INT_MAX ? -1 : distances[end]; // Return -1 if not reachable
}
};

#endif // MST_COMMON_HPP
#include "prim_mst.hpp"
#include <queue>
#include <functional>
#include <limits>
#include <climits>


Graph& PrimMST::getGraph(std::vector<Graph::Edge> mst)
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

std::vector<Graph::Edge> PrimMST::solve(const Graph& graph) {
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


int PrimMST::dfsLongestPath(int node, std::vector<bool>& visited) {
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
int PrimMST::bfsShortestPath(const Graph& graph, int start, int end) {
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

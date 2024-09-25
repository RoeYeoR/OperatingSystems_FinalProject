#include "kruskal_mst.hpp"
#include <algorithm>
#include <vector>
#include <queue>
#include <utility> // For std::pair
#include <climits>



Graph& KruskalMST::getGraph(std::vector<Graph::Edge> mst)
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

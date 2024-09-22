#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

void sendGraphData(int clientSocket) {
    int V, E;

    // Prompt user for number of vertices
    std::cout << "Enter the number of vertices: ";
    std::cin >> V;
    write(clientSocket, &V, sizeof(V));

    // Prompt user for number of edges
    std::cout << "Enter the number of edges: ";
    std::cin >> E;
    write(clientSocket, &E, sizeof(E));

    // Create an array to store edges
    for (int i = 0; i < E; ++i) {
        int u, v, w;
        std::cout << "Set edge and it's weight (template: src dest weight) number " << (i + 1) << ": ";
        std::cin >> u >> v >> w;

        write(clientSocket, &u, sizeof(int));
        write(clientSocket, &v, sizeof(int));
        write(clientSocket, &w, sizeof(int));
    }
}

void receiveResults(int clientSocket) {
    int totalWeight;
    read(clientSocket, &totalWeight, sizeof(totalWeight));
    std::cout << "Total weight of MST: " << totalWeight << std::endl;

    int longestDistance;
    read(clientSocket, &longestDistance, sizeof(longestDistance));
    std::cout << "Longest distance between two vertices: " << longestDistance << std::endl;

    double averageDistance;
    read(clientSocket, &averageDistance, sizeof(averageDistance));
    std::cout << "Average distance between vertices: " << averageDistance << std::endl;

    int shortestDistance;
    int u, v; // Specify the vertices for the shortest path
    std::cout << "Enter two vertices (u and v) for shortest distance: ";
    std::cin >> u >> v;
    write(clientSocket, &u, sizeof(u));
    write(clientSocket, &v, sizeof(v));
    read(clientSocket, &shortestDistance, sizeof(shortestDistance));
    std::cout << "Shortest distance between vertices " << u << " and " << v << ": " << shortestDistance << std::endl;
}

int main() {
    int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1) {
        std::cerr << "Failed to create socket" << std::endl;
        return -1;
    }

    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(8080);
    inet_pton(AF_INET, "127.0.0.1", &serverAddr.sin_addr);

    if (connect(clientSocket, (sockaddr*)&serverAddr, sizeof(serverAddr)) == -1) {
        std::cerr << "Failed to connect to server" << std::endl;
        return -1;
    }

    sendGraphData(clientSocket);
    receiveResults(clientSocket);

    close(clientSocket);
    return 0;
}

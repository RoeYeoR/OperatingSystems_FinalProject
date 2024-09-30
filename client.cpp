#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
void receiveMSTMetrics(int clientSocket) {
    //First receive the length of the algorithm name
    int algoNameLength;
    std::cout << "Waiting to receive algoNameLength..." << std::endl;
    ssize_t result = read(clientSocket, &algoNameLength, sizeof(algoNameLength));
    if (result <= 0) {
        std::cerr << "Failed to receive algoNameLength." << std::endl;
        return;
    }
    std::cout << "Received algoNameLength: " << algoNameLength << std::endl;

    // Allocate a buffer to hold the algorithm name
    char* algoName = new char[algoNameLength + 1];
    result = read(clientSocket, algoName, algoNameLength);
    if (result <= 0) {
        std::cerr << "Failed to receive algoName." << std::endl;
        delete[] algoName;
        return;
    }
    algoName[algoNameLength] = '\0';  // Null-terminate the string
    std::cout << "Algorithm used for MST: " << algoName << std::endl;

    //Receive the MST metrics
    int totalWeight;
    result = read(clientSocket, &totalWeight, sizeof(totalWeight));
    if (result <= 0) {
        std::cerr << "Failed to receive totalWeight." << std::endl;
        delete[] algoName;
        return;
    }
    std::cout << "Total weight of MST: " << totalWeight << std::endl;

    int longestDistance;
    result = read(clientSocket, &longestDistance, sizeof(longestDistance));
    if (result <= 0) {
        std::cerr << "Failed to receive longestDistance." << std::endl;
        delete[] algoName;
        return;
    }
    std::cout << "Longest distance between two vertices: " << longestDistance << std::endl;

    double averageDistance;
    result = read(clientSocket, &averageDistance, sizeof(averageDistance));
    if (result <= 0) {
        std::cerr << "Failed to receive averageDistance." << std::endl;
        delete[] algoName;
        return;
    }
    std::cout << "Average distance between vertices: " << averageDistance << std::endl;

    delete[] algoName;
}
void sendGraphData(int clientSocket) {
    int V, E;

    //Prompt user for number of vertices
    std::cout << "Enter the number of vertices: ";
    std::cin >> V;
    write(clientSocket, &V, sizeof(V));

    //Prompt user for number of edges
    std::cout << "Enter the number of edges: ";
    std::cin >> E;
    write(clientSocket, &E, sizeof(E));

    // int ver=4;
    // int edges=4;
    // write(clientSocket, &ver, sizeof(ver));
    // write(clientSocket, &edges, sizeof(edges));
    

    // Receive the MST metrics

    // Create an array to store edges
    for (int i = 0; i < E; ++i) {
        int u, v, w;
        std::cout << "Set edge and it's weight (template: src dest weight) number " << (i + 1) << ": ";
        std::cin >> u >> v >> w;

        write(clientSocket, &u, sizeof(int));
        write(clientSocket, &v, sizeof(int));
        write(clientSocket, &w, sizeof(int));
    }
    // Get the MST algorithm choice from the user.
std::string algoChoice;
std::cout << "Choose MST algorithm (Prim/Kruskal): ";
std::cin >> algoChoice;

// Send the algorithm choice to the server.
std::cout << "Sending MST algorithm choice to the server: " << algoChoice << std::endl;
ssize_t result = send(clientSocket, algoChoice.c_str(), algoChoice.size(), 0);
if (result == -1) {
    std::cerr << "Failed to send MST algorithm choice." << std::endl;
    return;
}
    receiveMSTMetrics(clientSocket);
}


void calculateShortestDistance(int clientSocket) {
    int u, v;
    std::cout << "Enter two vertices (u and v) for shortest distance: ";
    std::cin >> u >> v;

    // Send the vertices to the server
    if (write(clientSocket, &u, sizeof(u)) == -1 || write(clientSocket, &v, sizeof(v)) == -1) {
        std::cerr << "Failed to send vertices to server." << std::endl;
        return;
    }

    // Receive the shortest distance from the server
    int shortestDistance;
    if (read(clientSocket, &shortestDistance, sizeof(shortestDistance)) == -1) {
        std::cerr << "Failed to receive shortest distance from server." << std::endl;
        return;
    }

    // Handle edge cases based on the server response
    if (shortestDistance == -1) {
        std::cout << "Error: Invalid vertices or no path between vertices " << u << " and " << v << "." << std::endl;
    } else {
        std::cout << "Shortest distance between vertices " << u << " and " << v << ": " << shortestDistance << std::endl;
    }
}
void receiveResults(int clientSocket) {
    // Receive MST metrics after graph submission or algorithm change
    receiveMSTMetrics(clientSocket);

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
    serverAddr.sin_port = htons(8082);
    inet_pton(AF_INET, "127.0.0.1", &serverAddr.sin_addr);

    if (connect(clientSocket, (sockaddr*)&serverAddr, sizeof(serverAddr)) == -1) {
        std::cerr << "Failed to connect to server" << std::endl;
        return -1;
    }

    // Send the graph data to the server and receive the MST metrics
    sendGraphData(clientSocket);

    while (true) {
    int choice;
    std::cout << "Choose an option: \n1. Change MST Algorithm\n2. Calculate new shortest distance\n3. Exit\n";
    std::cin >> choice;

    // Send the command to the server
    write(clientSocket, &choice, sizeof(choice));

    if (choice == 3) {
        std::cout << "Exiting..." << std::endl;
        break;
    }
    else if (choice == 1) {
        // Change MST Algorithm
        int algoChoice;
        std::cout << "Choose algorithm to use for MST (1 for Prim, 2 for Kruskal): ";
        std::cin >> algoChoice;
        write(clientSocket, &algoChoice, sizeof(algoChoice));

        // Expect updated MST metrics from the server
        receiveMSTMetrics(clientSocket);
    }
    else if (choice == 2) {
        // Calculate new shortest distance
        int u, v;
        std::cout << "Enter two vertices (u and v) for shortest distance: ";
        std::cin >> u >> v;
        write(clientSocket, &u, sizeof(u));
        write(clientSocket, &v, sizeof(v));

        int shortestDistance;
        read(clientSocket, &shortestDistance, sizeof(shortestDistance));
        std::cout << "Shortest distance between vertices " << u << " and " << v << ": " << shortestDistance << std::endl;
    }
}
}
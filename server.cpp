#include "server.hpp"
#include "active_object.hpp"
#include "mst_factory.hpp"
#include "graph.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <thread>

// Constructor
Server::Server(int port, MSTType mstType)
    : port(port), mstType(mstType), running(false), stopThreads(false), leaderAvailable(true) {
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket == -1) {
        throw std::runtime_error("Failed to create socket");
    }

    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    serverAddr.sin_addr.s_addr = INADDR_ANY;

    if (bind(serverSocket, (sockaddr*)&serverAddr, sizeof(serverAddr)) == -1) {
        throw std::runtime_error("Failed to bind socket");
    }

    if (listen(serverSocket, 5) == -1) {
        throw std::runtime_error("Failed to listen on socket");
    }

    // Initialize Active Objects for Pipeline stages
    readAO = new ActiveObject();
    processAO = new ActiveObject();
    sendAO = new ActiveObject();

    // Initialize Leader Thread
    leaderThread = std::thread(&Server::leaderWorker, this);
}

// Destructor
Server::~Server() {
    stop();
    close(serverSocket);

    if (leaderThread.joinable()) {
        leaderThread.join();
    }

    delete readAO;
    delete processAO;
    delete sendAO;
}

// Start the server (Leader-Follower model)
void Server::start() {
    running = true;

    // Start thread pool
    for (int i = 0; i < std::thread::hardware_concurrency(); ++i) {
        threadPool.emplace_back(&Server::threadWorker, this);
    }

    std::cout << "Server is running on port " << port << std::endl;
}

// Read the graph from client (Stage 1)
void Server::readGraphFromClient(int clientSocket) {
    int V;
    read(clientSocket, &V, sizeof(V));
    std::cout << "Number of vertices received from client: " << V << std::endl;
    Graph graph(V);
    int E;
    read(clientSocket, &E, sizeof(E));
    std::cout << "Number of edges received from client: " << E << std::endl;
    for (int i = 0; i < E; ++i) {
        int u, v, w;
        if (read(clientSocket, &u, sizeof(u)) != sizeof(u) ||
            read(clientSocket, &v, sizeof(v)) != sizeof(v) ||
            read(clientSocket, &w, sizeof(w)) != sizeof(w)) {
            std::cerr << "Error reading edge data from client" << std::endl;
            close(clientSocket);
            return;
        }
        graph.addEdge(u, v, w);
    }

    // Add task to the process AO (Pipeline Stage 2)
    processAO->addTask([this, graph, clientSocket]() {
        processGraph(graph, clientSocket);
    });
}

// Process the graph and calculate the MST and additional metrics (Stage 2)
void Server::processGraph(const Graph& graph, int clientSocket) {
    auto mstSolver = MSTFactory::createMST(mstType);
    Graph& mst =  mstSolver->getGraph(mstSolver->solve(graph));
    int totalWeight = mstSolver->totalWeight(mst);
    int longestDistance = mstSolver->longestDistance(mst);
    double averageDistance = mstSolver->averageDistance(mst);
   
    // Send results to the client
    sendAO->addTask([this,mst,totalWeight, longestDistance, averageDistance, clientSocket]() {
        
        sendResultToClient(mst,totalWeight, longestDistance, averageDistance, clientSocket);
    });
}

// Send the results to the client (Stage 3)
void Server::sendResultToClient(const Graph& mst,int totalWeight, int longestDistance, double averageDistance, int clientSocket) {
    auto mstSolver = MSTFactory::createMST(mstType);
    write(clientSocket, &totalWeight, sizeof(totalWeight));
    write(clientSocket, &longestDistance, sizeof(longestDistance));
    write(clientSocket, &averageDistance, sizeof(averageDistance));

    // Read shortest distance request
    int u, v;
    read(clientSocket, &u, sizeof(u));
    read(clientSocket, &v, sizeof(v));

    // Calculate and send shortest distance
    //int shortestDistance = 0; // Assuming you have a method to compute this
    int shortestDistance = mstSolver->shortestDistance(mst, u, v); // Uncomment when implemented

    write(clientSocket, &shortestDistance, sizeof(shortestDistance));
    close(clientSocket);
}

// Stop the server
void Server::stop() {
    running = false;

    // Stop the thread pool
    {
        std::unique_lock<std::mutex> lock(queueMutex);
        stopThreads = true;
    }
    condition.notify_all();

    for (auto& thread : threadPool) {
        if (thread.joinable()) {
            thread.join();
        }
    }

    readAO->stop();
    processAO->stop();
    sendAO->stop();
}

// Leader thread that accepts connections (Leader-Follower Pattern)
void Server::leaderWorker() {
    while (running) {
        sockaddr_in clientAddr;
        socklen_t clientSize = sizeof(clientAddr);
        int clientSocket = accept(serverSocket, (sockaddr*)&clientAddr, &clientSize);

        if (clientSocket == -1) {
            std::cerr << "Failed to accept client connection" << std::endl;
            continue;
        }

        // Hand over leadership to another thread
        {
            std::unique_lock<std::mutex> lock(leaderMutex);
            leaderAvailable = false;
            leaderCondition.notify_all();
        }

        // Add task to read AO (Pipeline Stage 1)
        readAO->addTask([this, clientSocket]() {
            readGraphFromClient(clientSocket);
        });

        // Wait for another thread to become leader
        std::unique_lock<std::mutex> leaderLock(leaderMutex);
        leaderCondition.wait(leaderLock, [this]() { return leaderAvailable; });
    }
}

// Worker threads process tasks
void Server::threadWorker() {
    while (true) {
        std::function<void()> task;

        {
            std::unique_lock<std::mutex> lock(queueMutex);
            condition.wait(lock, [this]() { return !taskQueue.empty() || stopThreads; });

            if (stopThreads && taskQueue.empty()) {
                return;
            }

            task = std::move(taskQueue.front());
            taskQueue.pop();
        }

        task();

        // After finishing task, take leadership if it's free
        {
            std::unique_lock<std::mutex> lock(leaderMutex);
            if (!leaderAvailable) {
                leaderAvailable = true;
                leaderCondition.notify_all();
            }
        }
    }
}

int main() {
    Server server(8082, MSTType::PRIM); // or however you initialize it
    server.start();

    // Keep the main thread alive
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }


    return 0;
}

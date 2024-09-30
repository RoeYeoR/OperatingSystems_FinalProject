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

    if (listen(serverSocket, SOMAXCONN) == -1) {
        throw std::runtime_error("Failed to listen on socket");
    }

    // Initialize Active Objects for Pipeline stages
    readAO = new ActiveObject();
    processAO = new ActiveObject();
    sendAO = new ActiveObject();
}

// Destructor
Server::~Server() {
    stop();
    close(serverSocket);

    delete readAO;
    delete processAO;
    delete sendAO;
}

void Server::start() {
    running = true;

    // Start thread pool
    unsigned int threadCount = std::thread::hardware_concurrency();
    for (unsigned int i = 0; i < threadCount; ++i) {
        threadPool.emplace_back(&Server::threadWorker, this);
    }

    // Start leader thread
    leaderThread = std::thread(&Server::leaderWorker, this);

    std::cout << "Server is running on port " << port << std::endl;
}

// Read the graph from client (Stage 1)
void Server::readGraphFromClient(int clientSocket) {
    int V;
    read(clientSocket, &V, sizeof(V));
    std::cout << "Number o  f vertices received from client: " << V << std::endl;
    Graph graph(V);
    int E;
    read(clientSocket, &E, sizeof(E));
    std::cout << "Number of edges received from client: " << E << std::endl;
    for (int i = 0; i < E; ++i) 
    {
        int u, v, w;
        if (read(clientSocket, &u, sizeof(u)) != sizeof(u) ||
            read(clientSocket, &v, sizeof(v)) != sizeof(v) ||
            read(clientSocket, &w, sizeof(w)) != sizeof(w)) {
            std::cerr << "Error reading edge data from client" << std::endl;
            close(clientSocket);
            return;
        }
        graph.addEdge(u, v, w);
        std::cout << "Added Edge: u=" << u<<" v="<<v<<" w="<<w << std::endl;

        // graph.addEdge(0, 1, 11);
        // graph.addEdge(2, 0, 1);
        // graph.addEdge(0, 3, 2);
        // graph.addEdge(2, 3, 9);

        //graph.addEdge(1, 3, 20);
        //graph.addEdge(3, 2, 1);
        //graph.addEdge(2, 4, 3);
        //graph.addEdge(4, 5, 10);
        //graph.addEdge(2, 5, 9);
        
    }
    int algoChoice;
    read(clientSocket, &algoChoice, sizeof(algoChoice));
     // Set the MST algorithm type based on the client's choice
    if (algoChoice == 1) {
        mstType = MSTType::PRIM;
        std::cout << "Algo: PRIM " << std::endl;

    } else if (algoChoice == 2) {
        mstType = MSTType::KRUSKAL;
        std::cout << "Algo: KRUSKAL " << std::endl;
    } else {
        std::cerr << "Invalid algorithm choice, defaulting to Prim's algorithm." << std::endl;
        mstType = MSTType::PRIM;
    }
    mstType = MSTType::PRIM;
    std::cout << "Using Prim's algorithm by default." << std::endl;

    // // Process the graph and calculate MST metrics
    // auto mstSolver = MSTFactory::createMST(mstType);
    // Graph mst = mstSolver->getGraph(mstSolver->solve(graph));
    // int totalWeight = mstSolver->totalWeight(mst);
    // int longestDistance = mstSolver->longestDistance(mst);
    // double averageDistance = mstSolver->averageDistance(mst);

    // //Send the MST metrics immediately after processing the graph
    // sendMSTMetricsToClient(totalWeight, longestDistance, averageDistance, clientSocket);
    MSTType chosenMSTType = (algoChoice == 2) ? MSTType::KRUSKAL : MSTType::PRIM;
    // Move to the next stage of the pipeline
    processAO->addTask([this, graph, chosenMSTType, clientSocket]() {
        processGraph(graph, chosenMSTType, clientSocket);
    });
}

// Process the graph and calculate the MST and additional metrics (Stage 2)
void Server::processGraph(  const Graph& graph, MSTType initialMSTType, int clientSocket) {
    std::unique_ptr<MSTStrategy> mstSolver = MSTFactory::createMST(initialMSTType);
    Graph mst = mstSolver->getGraph(mstSolver->solve(graph));
    int totalWeight = mstSolver->totalWeight(mst);
    int longestDistance = mstSolver->longestDistance(mst);
    double averageDistance = mstSolver->averageDistance(mst);

    // Send initial MST metrics to the client
    sendAO->addTask([this, totalWeight, longestDistance, averageDistance, clientSocket, initialMSTType]() {
        sendMSTMetricsToClient(totalWeight, longestDistance, averageDistance, clientSocket, initialMSTType);
    });

    bool clientActive = true;
    while (clientActive && running) {
        int command;
        ssize_t commandRead = read(clientSocket, &command, sizeof(command));

        if (commandRead <= 0) {
            std::cerr << "Failed to read command from client, closing connection." << std::endl;
            break;
        }

        switch (command) {
            case 1: { // Change MST algorithm
                int algoChoice;
                if (read(clientSocket, &algoChoice, sizeof(algoChoice)) <= 0) {
                    std::cerr << "Failed to read algorithm choice from client." << std::endl;
                    clientActive = false;
                    break;
                }

                MSTType newMSTType = (algoChoice == 2) ? MSTType::KRUSKAL : MSTType::PRIM;
                // Recompute MST with the new algorithm
                mstSolver = MSTFactory::createMST(newMSTType);
                mst = mstSolver->getGraph(mstSolver->solve(graph));
                totalWeight = mstSolver->totalWeight(mst);
                longestDistance = mstSolver->longestDistance(mst);
                averageDistance = mstSolver->averageDistance(mst);

                // Send the updated MST metrics after changing algorithm
                sendAO->addTask([this, totalWeight, longestDistance, averageDistance, clientSocket, newMSTType]() {
                    sendMSTMetricsToClient(totalWeight, longestDistance, averageDistance, clientSocket, newMSTType);
                });
                break;
            }
            case 2: { // Calculate and send the shortest distance between two vertices
                int u, v;
                if (read(clientSocket, &u, sizeof(u)) <= 0 || read(clientSocket, &v, sizeof(v)) <= 0) {
                    std::cerr << "Failed to read vertices from client." << std::endl;
                    clientActive = false;
                    break;
                }

                int shortestDistance = mstSolver->shortestDistance(mst, u, v);
                
                sendAO->addTask([this, shortestDistance, clientSocket]() {
                    write(clientSocket, &shortestDistance, sizeof(shortestDistance));
                });
                break;
            }
            case 3: // Exit command
                std::cout << "Client requested to close the connection." << std::endl;
                clientActive = false;
                break;
            default:
                std::cerr << "Invalid command received from client." << std::endl;
                clientActive = false;
                break;
        }
    }

    // Close the socket after all commands are processed
    close(clientSocket);
}
// Send MST metrics to the client
void Server::sendMSTMetricsToClient(int totalWeight, int longestDistance, double averageDistance, int clientSocket, MSTType mstType) {
    std::string algoName = (mstType == MSTType::PRIM) ? "Prim" : "Kruskal";
    int algoNameLength = algoName.size();

    write(clientSocket, &algoNameLength, sizeof(algoNameLength));
    write(clientSocket, algoName.c_str(), algoNameLength);
    write(clientSocket, &totalWeight, sizeof(totalWeight));
    write(clientSocket, &longestDistance, sizeof(longestDistance));
    write(clientSocket, &averageDistance, sizeof(averageDistance));
}
// Send the results to the client (Stage 3)
void Server::sendResultToClient(const Graph& mst, int totalWeight, int longestDistance, double averageDistance, int clientSocket, MSTType chosenMSTType) {
    sendMSTMetricsToClient(totalWeight, longestDistance, averageDistance, clientSocket, chosenMSTType);

    bool clientActive = true;
    while (clientActive && running) {
        int command;
        ssize_t commandRead = read(clientSocket, &command, sizeof(command));

        if (commandRead <= 0) {
            std::cerr << "Failed to read command from client, closing connection." << std::endl;
            break;
        }

        switch (command) {
            case 1: // Change MST algorithm
                handleChangeAlgorithm(mst, clientSocket);
                break;
            case 2: // Calculate shortest distance
                handleShortestDistance(mst, clientSocket);
                break;
            case 3: // Exit
                std::cout << "Client requested to close the connection." << std::endl;
                clientActive = false;
                break;
            default:
                std::cerr << "Invalid command received from client." << std::endl;
                clientActive = false;
                break;
        }
    }

    close(clientSocket);
}
void Server::handleChangeAlgorithm(const Graph& originalGraph, int clientSocket) {
    int algoChoice;
    if (read(clientSocket, &algoChoice, sizeof(algoChoice)) <= 0) {
        std::cerr << "Error reading new algorithm choice from client" << std::endl;
        return;
    }

    MSTType newMSTType = (algoChoice == 2) ? MSTType::KRUSKAL : MSTType::PRIM;
    
    // Recompute MST with the new algorithm
    auto mstSolver = MSTFactory::createMST(newMSTType);
    Graph newMST = mstSolver->getGraph(mstSolver->solve(originalGraph));
    int totalWeight = mstSolver->totalWeight(newMST);
    int longestDistance = mstSolver->longestDistance(newMST);
    double averageDistance = mstSolver->averageDistance(newMST);

    sendMSTMetricsToClient(totalWeight, longestDistance, averageDistance, clientSocket, newMSTType);
}
void Server::handleShortestDistance(const Graph& mst, int clientSocket) {
    int u, v;
    if (read(clientSocket, &u, sizeof(u)) <= 0 || read(clientSocket, &v, sizeof(v)) <= 0) {
        std::cerr << "Error reading vertices for shortest distance calculation" << std::endl;
        return;
    }

    auto mstSolver = MSTFactory::createMST(mstType);  // Use the current MST type
    int shortestDistance = mstSolver->shortestDistance(mst, u, v);
    write(clientSocket, &shortestDistance, sizeof(shortestDistance));
}
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

    // Stop the leader thread
    if (leaderThread.joinable()) {
        leaderThread.join();
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
            if (!running) break;  // Server is shutting down
            std::cerr << "Failed to accept client connection" << std::endl;
            continue;
        }

        // Add task to read AO (Pipeline Stage 1)
        readAO->addTask([this, clientSocket]() {
            readGraphFromClient(clientSocket);
        });

        // Hand over leadership to another thread
        {
            std::unique_lock<std::mutex> lock(leaderMutex);
            leaderAvailable = false;
        }
        leaderCondition.notify_all();

        // Wait for another thread to become leader
        std::unique_lock<std::mutex> leaderLock(leaderMutex);
        leaderCondition.wait(leaderLock, [this]() { return leaderAvailable || !running; });
    }
}

// Worker threads process tasks
void Server::threadWorker() {
    while (!stopThreads) {
        std::function<void()> task;

        {
            std::unique_lock<std::mutex> lock(queueMutex);
            condition.wait(lock, [this]() { return !taskQueue.empty() || stopThreads; });

            if (stopThreads && taskQueue.empty()) {
                return;
            }

            if (!taskQueue.empty()) {
                task = std::move(taskQueue.front());
                taskQueue.pop();
            }
        }

        if (task) {
            task();
        }

        // After finishing task, take leadership if it's free
        {
            std::unique_lock<std::mutex> lock(leaderMutex);
            if (!leaderAvailable) {
                leaderAvailable = true;
                leaderCondition.notify_one();
            }
        }
    }
}

int main() {
    try {
        Server server(8082, MSTType::PRIM);
        server.start();

        std::cout << "Server started. Press Enter to stop the server." << std::endl;
        std::cin.get();

        server.stop();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

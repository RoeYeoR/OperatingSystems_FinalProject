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
    : port(port), mstType(mstType), running(false), stopThreads(false), isLeader(false), currentLeaderSocket(-1) {
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

    // Start initial leader thread
    promoteToLeader(0);

    std::cout << "[Thread " << std::this_thread::get_id() << "] Server is running on port " << port << std::endl;
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
        std::cout << "[Thread " << std::this_thread::get_id() << "] Added Edge: u=" << u<<" v="<<v<<" w="<<w << std::endl;

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
char algoChoiceBuffer[10];  // Small buffer since we expect only "Prim" or "Kruskal"

// Receive the algorithm choice from the client
std::cout << "Receiving MST algorithm choice from client socket: " << clientSocket << std::endl;
ssize_t result = recv(clientSocket, algoChoiceBuffer, sizeof(algoChoiceBuffer), 0);
if (result <= 0) {
    std::cerr << "Failed to receive MST algorithm choice." << std::endl;
    close(clientSocket);
    return;
}

// Null-terminate the received string
algoChoiceBuffer[result] = '\0';  // Ensure the string is null-terminated

// Now compare the received string to select the algorithm
std::string algoChoice(algoChoiceBuffer);
MSTType clientMSTType;

if (algoChoice == "Prim") {
    clientMSTType = MSTType::PRIM;
    std::cout << "Client chose Prim's algorithm." << std::endl;
} else if (algoChoice == "Kruskal") {
    clientMSTType = MSTType::KRUSKAL;
    std::cout << "Client chose Kruskal's algorithm." << std::endl;
} else {
    std::cerr << "Invalid MST algorithm choice received from client." << std::endl;
    close(clientSocket);  // Close the connection on error
    return;
}
  std::cout << "[Thread " << std::this_thread::get_id() << "] Preparing to add task to processAO queue for client socket: " << clientSocket << std::endl;

processAO->addTask([this, graph = graph, clientMSTType, clientSocket]() {
    std::cout << "[Thread " << std::this_thread::get_id() << "] Task for processGraph added to processAO queue for client socket: " << clientSocket << std::endl;
    processGraph(graph, clientMSTType, clientSocket);
});
std::cout << "[Thread " << std::this_thread::get_id() << "] Task successfully added to processAO queue for client socket: " << clientSocket << std::endl;
}

// Process the graph and calculate the MST and additional metrics (Stage 2)
void Server::processGraph(  const Graph& graph, MSTType initialMSTType, int clientSocket) {
    std::cout << "process graph "  << std::endl;

    std::unique_ptr<MSTStrategy> mstSolver = MSTFactory::createMST(initialMSTType);
    Graph mst = mstSolver->getGraph(mstSolver->solve(graph));
    int totalWeight = mstSolver->totalWeight(mst);
    int longestDistance = mstSolver->longestDistance(mst);
    double averageDistance = mstSolver->averageDistance(mst);

    // Send initial MST metrics to the client
    sendAO->addTask([this, totalWeight, longestDistance, averageDistance, clientSocket, initialMSTType]() {
    std::cout << "[Thread " << std::this_thread::get_id() << "] Sending MST metrics to client socket: " << clientSocket << std::endl;
    sendMSTMetricsToClient(totalWeight, longestDistance, averageDistance, clientSocket, initialMSTType);
});

    bool clientActive = true;
    while (clientActive && running) {
        int command;
        ssize_t commandRead = read(clientSocket, &command, sizeof(command));

        if (commandRead <= 0) {
            std::cerr << "[Thread " << std::this_thread::get_id() << "] Failed to read command from client socket: " << clientSocket << std::endl;
            break;
        }

        std::cout << "[Thread " << std::this_thread::get_id() << "] Command received: " << command << " from client socket: " << clientSocket << std::endl;

        switch (command) {
            case 1: { // Change MST algorithm
                int algoChoice;
                if (read(clientSocket, &algoChoice, sizeof(algoChoice)) <= 0) {
                    std::cerr << "[Thread " << std::this_thread::get_id() << "] Failed to read algorithm choice from client socket: " << clientSocket << std::endl;
                    clientActive = false;
                    break;
                }

                MSTType newMSTType = (algoChoice == 2) ? MSTType::KRUSKAL : MSTType::PRIM;
                std::cout << "[Thread " << std::this_thread::get_id() << "] Changing MST algorithm to " 
                          << (newMSTType == MSTType::PRIM ? "Prim" : "Kruskal") << " for client socket: " << clientSocket << std::endl;

                // Recompute MST with the new algorithm
                mstSolver = MSTFactory::createMST(newMSTType);
                mst = mstSolver->getGraph(mstSolver->solve(graph));
                totalWeight = mstSolver->totalWeight(mst);
                longestDistance = mstSolver->longestDistance(mst);
                averageDistance = mstSolver->averageDistance(mst);

                // Send the updated MST metrics after changing algorithm
                sendAO->addTask([this, totalWeight, longestDistance, averageDistance, clientSocket, newMSTType]() {
                    std::cout << "[Thread " << std::this_thread::get_id() << "] Sending updated MST metrics to client socket: " << clientSocket << std::endl;
                    sendMSTMetricsToClient(totalWeight, longestDistance, averageDistance, clientSocket, newMSTType);
                });
                break;
            }
            case 2: { // Calculate and send the shortest distance between two vertices
                int u, v;
                if (read(clientSocket, &u, sizeof(u)) <= 0 || read(clientSocket, &v, sizeof(v)) <= 0) {
                    std::cerr << "[Thread " << std::this_thread::get_id() << "] Failed to read vertices from client socket: " << clientSocket << std::endl;
                    clientActive = false;
                    break;
                }

                std::cout << "[Thread " << std::this_thread::get_id() << "] Calculating shortest distance between " << u << " and " << v << " for client socket: " << clientSocket << std::endl;

                int shortestDistance = mstSolver->shortestDistance(mst, u, v);
                
                sendAO->addTask([this, shortestDistance, clientSocket]() {
                    std::cout << "[Thread " << std::this_thread::get_id() << "] Sending shortest distance to client socket: " << clientSocket << std::endl;
                    write(clientSocket, &shortestDistance, sizeof(shortestDistance));
                });
                break;
            }
            case 3: // Exit command
                std::cout << "[Thread " << std::this_thread::get_id() << "] Client requested to close the connection (socket: " << clientSocket << ")" << std::endl;
                clientActive = false;
                break;
            default:
                std::cerr << "[Thread " << std::this_thread::get_id() << "] Invalid command received from client socket: " << clientSocket << std::endl;
                clientActive = false;
                break;
        }
    }

    std::cout << "[Thread " << std::this_thread::get_id() << "] Closing client socket: " << clientSocket << std::endl;
    // Close the socket after all commands are processed
    close(clientSocket);
}
// Send MST metrics to the client
void Server::sendMSTMetricsToClient(int totalWeight, int longestDistance, double averageDistance, int clientSocket, MSTType mstType) {
    std::string algoName = (mstType == MSTType::PRIM) ? "Prim" : "Kruskal";
    int algoNameLength = algoName.size();
    
    std::cout << "[Thread " << std::this_thread::get_id() << "] Preparing to send MST metrics to client socket: " << clientSocket << std::endl;

    // Send algorithm name length
    ssize_t result = write(clientSocket, &algoNameLength, sizeof(algoNameLength));
    if (result <= 0) {
        std::cerr << "[Thread " << std::this_thread::get_id() << "] Failed to send algoNameLength" << std::endl;
        return;
    }
    std::cout << "Sending algoNameLength to client: " << clientSocket << std::endl;
    

    // Send algorithm name
    result = write(clientSocket, algoName.c_str(), algoNameLength);
    if (result <= 0) {
        std::cerr << "[Thread " << std::this_thread::get_id() << "] Failed to send algoName" << std::endl;
        return;
    }
    std::cout << "[Thread " << std::this_thread::get_id() << "] Sent algoName: " << algoName << std::endl;

    // Send total weight
    result = write(clientSocket, &totalWeight, sizeof(totalWeight));
    if (result <= 0) {
        std::cerr << "[Thread " << std::this_thread::get_id() << "] Failed to send totalWeight" << std::endl;
        return;
    }
    std::cout << "[Thread " << std::this_thread::get_id() << "] Sent totalWeight" << std::endl;

    // Send longest distance
    result = write(clientSocket, &longestDistance, sizeof(longestDistance));
    if (result <= 0) {
        std::cerr << "[Thread " << std::this_thread::get_id() << "] Failed to send longestDistance" << std::endl;
        return;
    }
    std::cout << "[Thread " << std::this_thread::get_id() << "] Sent longestDistance" << std::endl;

    // Send average distance
    result = write(clientSocket, &averageDistance, sizeof(averageDistance));
    if (result <= 0) {
        std::cerr << "[Thread " << std::this_thread::get_id() << "] Failed to send averageDistance" << std::endl;
        return;
    }
    std::cout << "[Thread " << std::this_thread::get_id() << "] Sent averageDistance" << std::endl;
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
                std::cout << "[Thread " << std::this_thread::get_id() << "] Client requested to close the connection." << std::endl;
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

        std::cout << "[Leader Thread " << std::this_thread::get_id() << "] Accepted connection from client socket: " << clientSocket << std::endl;

        // Add task to read from client and pass the client socket to the read active object
        readAO->addTask([this, clientSocket]() {
            readGraphFromClient(clientSocket);
        });

        // Demote current leader to follower and promote a new leader
        demoteToFollower();
        promoteToLeader((currentLeaderSocket + 1) % threadPool.size());

        // Join the follower pool
        threadWorker();
    }
}
// Worker threads process tasks
void Server::threadWorker() {
    while (!stopThreads) {
        if (isLeader.load()) {
            leaderWorker();
        } else {
            std::function<void()> task;

            {
                std::unique_lock<std::mutex> lock(queueMutex);
                condition.wait(lock, [this]() { return !taskQueue.empty() || stopThreads || isLeader.load(); });

                if (stopThreads && taskQueue.empty()) {
                    return;
                }

                if (isLeader.load()) {
                    continue;  // Skip to next iteration to enter leaderWorker()
                }

                if (!taskQueue.empty()) {
                    task = std::move(taskQueue.front());
                    taskQueue.pop();
                }
            }

            if (task) {
                std::cout << "[Worker Thread " << std::this_thread::get_id() << "] Executing task" << std::endl;
                task();
            }
        }
    }
}

void Server::promoteToLeader(int threadId) {
    std::unique_lock<std::mutex> lock(leaderMutex);
    currentLeaderSocket = threadId;
    isLeader.store(true);
    std::cout << "[Thread " << std::this_thread::get_id() << "] Promoted to leader" << std::endl;
    leaderPromotionCV.notify_all();
}

void Server::demoteToFollower() {
    std::unique_lock<std::mutex> lock(leaderMutex);
    isLeader.store(false);
    std::cout << "[Thread " << std::this_thread::get_id() << "] Demoted to follower" << std::endl;
}

void Server::stop() {
    running = false;
    stopThreads = true;
    
    condition.notify_all();
    leaderPromotionCV.notify_all();

    for (auto& thread : threadPool) {
        if (thread.joinable()) {
            thread.join();
        }
    }

    readAO->stop();
    processAO->stop();
    sendAO->stop();
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
#include "server.hpp"
#include "active_pipeline.hpp"
#include "mst_factory.hpp"
#include "graph.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <thread>

// Constructor
Server::Server(int port, MSTType mstType)
    : port(port), currentLeaderSocket(-1), mstType(mstType), 
      running(false), stopThreads(false), isLeader(false) {
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

    // Initialize Leader-Follower Thread Pool
    unsigned int threadCount = std::thread::hardware_concurrency();
    threadPool = std::make_unique<LeaderFollowerThreadPool>(threadCount);

    // Initialize Active Pipeline Stages
    readStage = std::make_shared<ActivePipelineStage>(ActivePipelineStage::StageType::SOURCE);
    processStage = std::make_shared<ActivePipelineStage>();
    sendStage = std::make_shared<ActivePipelineStage>(ActivePipelineStage::StageType::SINK);

    // Connect stages
    readStage->setNextStage(processStage);
    processStage->setNextStage(sendStage);

    // Create pipeline
    pipeline = std::make_unique<ActivePipeline>();
    pipeline->addStage(readStage);
    pipeline->addStage(processStage);
    pipeline->addStage(sendStage);
}

// Destructor
Server::~Server() {
    stop();
    close(serverSocket);
}

void Server::start() {
    running = true;

    // Start connection acceptance
    acceptConnections();
}

void Server::acceptConnections() {
    while (running) {
        // Accept client connection
        int clientSocket = accept(serverSocket, nullptr, nullptr);
        if (clientSocket == -1) {
            if (!running) break;
            std::cerr << "Failed to accept client connection" << std::endl;
            continue;
        }

        // Enqueue connection handling task
        threadPool->enqueue([this, clientSocket]() {
            // Add task to read from client and pass the client socket to the readStage
            readStage->enqueue([this, clientSocket]() {
                readGraphFromClient(clientSocket);
            });
        });
    }
}

void Server::stop() {
    running = false;
    
    // Close server socket to interrupt accept()
    if (serverSocket != -1) {
        close(serverSocket);
    }

    // Stop thread pool and pipeline
    if (threadPool) {
        threadPool->shutdown();
    }

    if (pipeline) {
        pipeline->stop();
    }
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

    std::cout << "[Thread " << std::this_thread::get_id() << "] Preparing to add task to processStage queue for client socket: " << clientSocket << std::endl;

    processStage->enqueue([this, graph = graph, clientMSTType, clientSocket]() {
        std::cout << "[Thread " << std::this_thread::get_id() << "] Task for processGraph added to processStage queue for client socket: " << clientSocket << std::endl;
        processGraph(graph, clientMSTType, clientSocket);
    });
    std::cout << "[Thread " << std::this_thread::get_id() << "] Task successfully added to processStage queue for client socket: " << clientSocket << std::endl;
}

// Process the graph and calculate the MST and additional metrics (Stage 2)
void Server::processGraph(const Graph& graph, MSTType initialMSTType, int clientSocket) {
    std::cout << "Processing graph with initial MST type: " 
              << (initialMSTType == MSTType::PRIM ? "Prim" : "Kruskal") 
              << std::endl;

    std::unique_ptr<MSTStrategy> mstSolver = MSTFactory::createMST(initialMSTType);
    std::vector<Graph::Edge> mstEdges = mstSolver->solve(graph);
    Graph mst = graph.createMSTGraph(mstEdges);
    
    int totalWeight = mstSolver->totalWeight(mst);
    int longestDistance = mstSolver->longestDistance(mst);
    double averageDistance = mstSolver->averageDistance(mst);

    // Send initial MST metrics to the client
    sendStage->enqueue([this, totalWeight, longestDistance, averageDistance, clientSocket, initialMSTType]() {
        std::cout << "[Thread " << std::this_thread::get_id() << "] Sending MST metrics to client socket: " << clientSocket << std::endl;
        sendMSTMetricsToClient(totalWeight, longestDistance, averageDistance, clientSocket, initialMSTType);
    });

    bool clientActive = true;
    int command;
    std::cout << "Waiting for client commands..." << std::endl;

    while (clientActive && running) {
        std::cout << "Attempting to read command from client socket..." << std::endl;
        ssize_t bytesRead = read(clientSocket, &command, sizeof(command));
        
        if (bytesRead <= 0) {
            std::cerr << "Error reading command or client disconnected. Bytes read: " << bytesRead << std::endl;
            break;
        }

        std::cout << "Received command: " << command << std::endl;

        switch(command) {
            case 1: { // Change MST algorithm
                int algoChoice;
                if (read(clientSocket, &algoChoice, sizeof(algoChoice)) <= 0) {
                    std::cerr << "Failed to read algorithm choice" << std::endl;
                    break;
                }

                // Convert client's numeric choice to MSTType
                MSTType newMSTType;
                switch(algoChoice) {
                    case 1:
                        newMSTType = MSTType::PRIM;
                        break;
                    case 2:
                        newMSTType = MSTType::KRUSKAL;
                        break;
                    default:
                        std::cerr << "Invalid algorithm choice: " << algoChoice << std::endl;
                        continue;
                }

                // Recompute MST with the new algorithm
                mstSolver = MSTFactory::createMST(newMSTType);
                mstEdges = mstSolver->solve(graph);
                mst = graph.createMSTGraph(mstEdges);
                
                totalWeight = mstSolver->totalWeight(mst);
                longestDistance = mstSolver->longestDistance(mst);
                averageDistance = mstSolver->averageDistance(mst);

                // Send updated metrics to the client
                sendStage->enqueue([this, totalWeight, longestDistance, averageDistance, clientSocket, newMSTType]() {
                    std::cout << "[Thread " << std::this_thread::get_id() << "] Sending updated MST metrics to client socket: " << clientSocket << std::endl;
                    sendMSTMetricsToClient(totalWeight, longestDistance, averageDistance, clientSocket, newMSTType);
                });
                break;
            }
            case 2: { // Shortest distance
                int u, v;
                if (read(clientSocket, &u, sizeof(u)) <= 0 || 
                    read(clientSocket, &v, sizeof(v)) <= 0) {
                    std::cerr << "Failed to read vertices for shortest distance" << std::endl;
                    break;
                }

                int shortestDistance = mstSolver->shortestDistance(mst, u, v);
                
                sendStage->enqueue([this, shortestDistance, clientSocket]() {
                    std::cout << "[Thread " << std::this_thread::get_id() << "] Sending shortest distance to client socket: " << clientSocket << std::endl;
                    write(clientSocket, &shortestDistance, sizeof(shortestDistance));
                });
                break;
            }
            case 3: // Exit
                clientActive = false;
                break;
            default:
                std::cerr << "Unknown command: " << command << std::endl;
                clientActive = false;
                break;
        }
    }

    // Close the socket after all commands are processed
    std::cout << "Closing client socket" << std::endl;
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
    std::cout << "Handling algorithm change request..." << std::endl;
    
    // Read algorithm choice from client
    ssize_t bytesRead = read(clientSocket, &algoChoice, sizeof(algoChoice));
    if (bytesRead <= 0) {
        std::cerr << "Failed to read algorithm choice. Bytes read: " << bytesRead << std::endl;
        return;
    }

    std::cout << "Received algorithm choice: " << algoChoice << std::endl;

    // Convert client's numeric choice to MSTType
    MSTType newMSTType;
    switch(algoChoice) {
        case 1:  // Prim
            newMSTType = MSTType::PRIM;
            break;
        case 2:  // Kruskal
            newMSTType = MSTType::KRUSKAL;
            break;
        default:
            std::cerr << "Invalid algorithm choice: " << algoChoice << std::endl;
            return;
    }

    std::cout << "Selected MST algorithm: " 
              << (newMSTType == MSTType::PRIM ? "Prim" : "Kruskal") 
              << std::endl;

    // Create MST solver with new algorithm
    std::unique_ptr<MSTStrategy> mstSolver = MSTFactory::createMST(newMSTType);
    std::vector<Graph::Edge> mstEdges = mstSolver->solve(originalGraph);
    Graph mst = originalGraph.createMSTGraph(mstEdges);
    
    int totalWeight = mstSolver->totalWeight(mst);
    int longestDistance = mstSolver->longestDistance(mst);
    double averageDistance = mstSolver->averageDistance(mst);

    std::cout << "MST Metrics - Total Weight: " << totalWeight 
              << ", Longest Distance: " << longestDistance 
              << ", Average Distance: " << averageDistance 
              << std::endl;

    // Send updated metrics to client
    sendStage->enqueue([this, totalWeight, longestDistance, averageDistance, clientSocket, newMSTType]() {
        std::cout << "Sending updated MST metrics to client" << std::endl;
        sendMSTMetricsToClient(totalWeight, longestDistance, averageDistance, clientSocket, newMSTType);
    });
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
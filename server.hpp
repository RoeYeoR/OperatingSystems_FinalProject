#ifndef SERVER_HPP
#define SERVER_HPP

#include <vector>
#include <thread>
#include <atomic>
#include <memory>
#include "graph.hpp"
#include "mst_factory.hpp"
#include "active_pipeline.hpp"
#include "leader_follower.hpp"

class Server {
public:
    Server(int port, MSTType mstType = MSTType::PRIM);
    ~Server();

    void start();
    void stop();

private:
    // Socket and connection management
    int serverSocket;
    int port;
    int currentLeaderSocket;

    // Concurrency control
    std::unique_ptr<LeaderFollowerThreadPool> threadPool;
    std::atomic<bool> running;
    std::atomic<bool> stopThreads;
    std::atomic<bool> isLeader;

    // MST algorithm configuration
    MSTType mstType;

    // Active Pipeline Stages
    std::shared_ptr<ActivePipelineStage> readStage;
    std::shared_ptr<ActivePipelineStage> processStage;
    std::shared_ptr<ActivePipelineStage> sendStage;
    std::unique_ptr<ActivePipeline> pipeline;

    // Server methods
    void readGraphFromClient(int clientSocket);
    void processGraph(const Graph& graph, MSTType initialMSTType, int clientSocket);
    void sendMSTMetricsToClient(int totalWeight, int longestDistance, double averageDistance, int clientSocket, MSTType mstType);
    void sendResultToClient(const Graph& mst, int totalWeight, int longestDistance, double averageDistance, int clientSocket, MSTType chosenMSTType);
    void handleChangeAlgorithm(const Graph& originalGraph, int clientSocket);
    void handleShortestDistance(const Graph& mst, int clientSocket);

    // Connection handling methods
    void acceptConnections();
};

#endif // SERVER_HPP

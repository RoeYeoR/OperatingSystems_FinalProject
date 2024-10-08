#ifndef SERVER_HPP
#define SERVER_HPP

#include "mst_factory.hpp"
#include "graph.hpp"
#include "active_object.hpp"
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <vector>
#include <functional>
#include <netinet/in.h>

class Server {
public:
    Server(int port, MSTType mstType);
    ~Server();

    void start();
    void stop();

private:
    std::atomic<bool> isLeader;
    std::atomic<int> currentLeaderSocket;
    std::condition_variable leaderPromotionCV;

    
    // Methods for pipeline stages
    void readGraphFromClient(int clientSocket); // Stage 1
    void processGraph( const Graph& graph, MSTType initialMSTType, int clientSocket);//Stage 2
    //void sendResultToClient(int totalWeight, int clientSocket); // Stage 3
    void sendResultToClient(const Graph& mst, int totalWeight, int longestDistance, double averageDistance, int clientSocket, MSTType chosenMSTType);
    void sendMSTMetricsToClient(int totalWeight, int longestDistance, double averageDistance, int clientSocket, MSTType mstType);
    // Leader-Follower methods
    void leaderWorker();  // Leader thread to accept connections
    void threadWorker();  // Worker threads to process tasks
    void handleChangeAlgorithm(const Graph& originalGraph, int clientSocket);
    void handleShortestDistance(const Graph& mst, int clientSocket) ;
    void demoteToFollower() ;
    void promoteToLeader(int threadId) ;

    // Server properties
    int serverSocket;
    int port;
    MSTType mstType;
    bool running;

    // Threading and concurrency
    std::thread leaderThread;  // Leader thread for accepting clients
    std::mutex leaderMutex;    // Mutex for leader thread coordination
    std::condition_variable leaderCondition;
    bool leaderAvailable;

    std::mutex queueMutex;    // Mutex for task queue
    std::condition_variable condition;
    std::queue<std::function<void()>> taskQueue;
    std::vector<std::thread> threadPool; // Worker threads
    bool stopThreads;

    // Active Objects for pipeline stages
    ActiveObject* readAO;    // For reading graph from client (Stage 1)
    ActiveObject* processAO; // For processing graph and MST (Stage 2)
    ActiveObject* sendAO;    // For sending results to the client (Stage 3)
};

#endif // SERVER_HPP

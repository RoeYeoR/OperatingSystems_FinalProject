#ifndef LEADER_FOLLOWER_HPP
#define LEADER_FOLLOWER_HPP

#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional>
#include <queue>
#include <chrono>
#include <memory>

class LeaderFollowerThreadPool {
public:
    // Enum for thread state
    enum class ThreadState {
        FOLLOWER,
        LEADER,
        WAITING,
        PROCESSING
    };

    // Task type with function object
    using Task = std::function<void()>;
    
    // Configuration options for thread pool
    struct PoolConfig {
        size_t threadCount = std::thread::hardware_concurrency();
        size_t maxQueueSize = 100;
        std::chrono::milliseconds leaderTimeout = std::chrono::milliseconds(500);
    };

    // Constructor with advanced configuration
    LeaderFollowerThreadPool(const PoolConfig& config = PoolConfig());
    ~LeaderFollowerThreadPool();

    // Enhanced task submission
    void enqueue(Task task);
    void enqueuePriority(Task task);  // High-priority tasks
    
    // Thread pool management
    void shutdown(bool waitForTasks = true);
    size_t pendingTasks() const;
    
    // Advanced error handling
    void setErrorHandler(std::function<void(const std::exception&)> handler);

private:
    // Worker thread management
    void workerLoop(size_t workerId);
    void promoteNewLeader();
    
    // Thread synchronization primitives
    std::vector<std::thread> threads;
    std::queue<Task> taskQueue;
    std::queue<Task> priorityTaskQueue;
    mutable std::mutex queueMutex;
    std::condition_variable condition;
    
    // Thread pool state
    std::atomic<bool> isRunning;
    std::atomic<bool> hasLeader;
    std::atomic<size_t> activeThreads;
    
    // Configuration and state tracking
    PoolConfig config;
    
    // Use non-atomic vector for thread states
    std::vector<ThreadState> threadStates;
    
    // Error handling
    std::function<void(const std::exception&)> errorHandler;
};

#endif // LEADER_FOLLOWER_HPP

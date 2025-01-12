#include "leader_follower.hpp"
#include <iostream>
#include <stdexcept>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <chrono>
#include <functional>
#include <algorithm>

enum class ThreadState {
    FOLLOWER,
    WAITING,
    LEADER,
    PROCESSING
};

struct PoolConfig {
    size_t threadCount;
    size_t maxQueueSize;
};

struct Task {
    std::function<void()> task;
};

class LeaderFollowerThreadPool {
public:
    LeaderFollowerThreadPool(const PoolConfig& config);
    ~LeaderFollowerThreadPool();

    void enqueue(Task task);
    void enqueuePriority(Task task);
    void shutdown(bool waitForTasks);
    size_t pendingTasks() const;
    void setErrorHandler(std::function<void(const std::exception&)> handler);

private:
    void promoteNewLeader();
    void workerLoop(size_t workerId);

    const PoolConfig config;
    std::vector<std::thread> threads;
    std::queue<Task> taskQueue;
    std::queue<Task> priorityTaskQueue;
    std::mutex queueMutex;
    std::condition_variable condition;
    bool isRunning;
    bool hasLeader;
    size_t activeThreads;
    std::vector<ThreadState> threadStates;
    std::function<void(const std::exception&)> errorHandler;
};

LeaderFollowerThreadPool::LeaderFollowerThreadPool(const PoolConfig& config) 
    : config(config), 
      isRunning(true), 
      hasLeader(false), 
      activeThreads(0),
      threadStates(config.threadCount, ThreadState::FOLLOWER),
      errorHandler([](const std::exception& e) { 
          std::cerr << "Unhandled thread pool error: " << e.what() << std::endl; 
      }) 
{
    // Initialize worker threads
    for (size_t i = 0; i < config.threadCount; ++i) {
        threads.emplace_back(&LeaderFollowerThreadPool::workerLoop, this, i);
    }
}

LeaderFollowerThreadPool::~LeaderFollowerThreadPool() {
    shutdown(false);
}

void LeaderFollowerThreadPool::enqueue(Task task) {
    std::unique_lock<std::mutex> lock(queueMutex);
    
    // Wait if queue is full
    if (taskQueue.size() >= config.maxQueueSize) {
        throw std::runtime_error("Task queue is full");
    }
    
    taskQueue.push(std::move(task));
    condition.notify_one();
}

void LeaderFollowerThreadPool::enqueuePriority(Task task) {
    std::unique_lock<std::mutex> lock(queueMutex);
    
    // Wait if queue is full
    if (priorityTaskQueue.size() >= config.maxQueueSize) {
        throw std::runtime_error("Priority task queue is full");
    }
    
    priorityTaskQueue.push(std::move(task));
    condition.notify_one();
}

void LeaderFollowerThreadPool::shutdown(bool waitForTasks) {
    // Graceful shutdown
    isRunning = false;
    condition.notify_all();
    
    // Wait for all threads to complete
    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    
    // Clear remaining tasks if not waiting
    if (!waitForTasks) {
        std::unique_lock<std::mutex> lock(queueMutex);
        while (!taskQueue.empty()) taskQueue.pop();
        while (!priorityTaskQueue.empty()) priorityTaskQueue.pop();
    }
}

size_t LeaderFollowerThreadPool::pendingTasks() const {
    std::unique_lock<std::mutex> lock(queueMutex);
    return taskQueue.size() + priorityTaskQueue.size();
}

void LeaderFollowerThreadPool::setErrorHandler(std::function<void(const std::exception&)> handler) {
    errorHandler = handler;
}

void LeaderFollowerThreadPool::workerLoop(size_t workerId) {
    while (isRunning) {
        Task task;
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            
            // Wait for tasks or shutdown
            condition.wait(lock, [this]() { 
                return !isRunning || !taskQueue.empty() || !priorityTaskQueue.empty(); 
            });
            
            // Exit if pool is stopped and no tasks
            if (!isRunning && taskQueue.empty() && priorityTaskQueue.empty()) {
                return;
            }
            
            // Prioritize high-priority tasks
            if (!priorityTaskQueue.empty()) {
                task = std::move(priorityTaskQueue.front());
                priorityTaskQueue.pop();
            } else if (!taskQueue.empty()) {
                task = std::move(taskQueue.front());
                taskQueue.pop();
            }
        }
        
        // Execute task
        try {
            threadStates[workerId] = ThreadState::PROCESSING;
            task.task();
            threadStates[workerId] = ThreadState::FOLLOWER;
        } catch (const std::exception& e) {
            threadStates[workerId] = ThreadState::FOLLOWER;
            errorHandler(e);
        }
    }
}

void LeaderFollowerThreadPool::promoteNewLeader() {
    // Leader promotion logic can be implemented here
    // This is a placeholder for more advanced leader selection
    hasLeader = false;
}

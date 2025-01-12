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
      errorHandler([](const std::exception& e) {
          std::cerr << "Unhandled thread pool error: " << e.what() << std::endl;
      })
{
    // Initialize thread states
    threadStates.resize(config.threadCount, ThreadState::FOLLOWER);
    
    // Create worker threads
    for (size_t i = 0; i < config.threadCount; ++i) {
        threads.emplace_back(&LeaderFollowerThreadPool::workerLoop, this, i);
    }
}

LeaderFollowerThreadPool::~LeaderFollowerThreadPool() {
    shutdown(true);
}

void LeaderFollowerThreadPool::enqueue(Task task) {
    std::unique_lock<std::mutex> lock(queueMutex);
    
    // Wait if queue is full
    if (!condition.wait_for(lock, std::chrono::seconds(1), [this]() { 
        return taskQueue.size() < config.maxQueueSize; 
    })) {
        throw std::runtime_error("Task queue is full. Cannot enqueue.");
    }

    taskQueue.push(std::move(task));
    condition.notify_one();
}

void LeaderFollowerThreadPool::enqueuePriority(Task task) {
    std::unique_lock<std::mutex> lock(queueMutex);
    
    // Wait if queue is full
    if (!condition.wait_for(lock, std::chrono::seconds(1), [this]() { 
        return priorityTaskQueue.size() < config.maxQueueSize; 
    })) {
        throw std::runtime_error("Priority task queue is full. Cannot enqueue.");
    }

    priorityTaskQueue.push(std::move(task));
    condition.notify_one();
}

void LeaderFollowerThreadPool::shutdown(bool waitForTasks) {
    isRunning = false;
    condition.notify_all();

    // Wait for all threads to complete
    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
}

size_t LeaderFollowerThreadPool::pendingTasks() const {
    std::unique_lock<std::mutex> lock(queueMutex);
    return taskQueue.size() + priorityTaskQueue.size();
}

void LeaderFollowerThreadPool::setErrorHandler(std::function<void(const std::exception&)> handler) {
    errorHandler = handler;
}

void LeaderFollowerThreadPool::promoteNewLeader() {
    hasLeader = false;
    condition.notify_one();
}

void LeaderFollowerThreadPool::workerLoop(size_t workerId) {
    while (isRunning) {
        Task task;
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            
            // Update thread state to waiting
            threadStates[workerId] = ThreadState::WAITING;
            
            // Wait for a task or shutdown signal
            condition.wait(lock, [this]() { 
                return !priorityTaskQueue.empty() || 
                       !taskQueue.empty() || 
                       !isRunning; 
            });

            // Check if we should exit
            if (!isRunning && priorityTaskQueue.empty() && taskQueue.empty()) {
                return;
            }

            // Try to become the leader
            bool expectedLeader = false;
            if (!hasLeader.compare_exchange_strong(expectedLeader, true)) {
                continue;
            }

            // Leader thread processing
            threadStates[workerId] = ThreadState::LEADER;
            
            // Prioritize priority tasks
            if (!priorityTaskQueue.empty()) {
                task = std::move(priorityTaskQueue.front());
                priorityTaskQueue.pop();
            } else if (!taskQueue.empty()) {
                task = std::move(taskQueue.front());
                taskQueue.pop();
            }

            // Notify any waiting producers
            condition.notify_one();
        }

        // Execute the task if there is one
        if (task) {
            try {
                threadStates[workerId] = ThreadState::PROCESSING;
                
                // Execute task
                task.task();
                
                threadStates[workerId] = ThreadState::FOLLOWER;
            } catch (const std::exception& e) {
                // Handle task execution error
                errorHandler(e);
            }
        }

        // Promote a new leader
        promoteNewLeader();
    }
}

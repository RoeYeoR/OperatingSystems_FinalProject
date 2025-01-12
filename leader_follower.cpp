#include "leader_follower.hpp"
#include <stdexcept>
#include <algorithm>

LeaderFollowerThreadPool::LeaderFollowerThreadPool(const PoolConfig& config) 
    : config(config), 
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
            task();  // Directly invoke the task
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

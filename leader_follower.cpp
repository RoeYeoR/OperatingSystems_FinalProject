#include "leader_follower.hpp"
#include <iostream>

LeaderFollowerThreadPool::LeaderFollowerThreadPool(size_t threadCount) 
    : isRunning(true), hasLeader(false) {
    for (size_t i = 0; i < threadCount; ++i) {
        threads.emplace_back(&LeaderFollowerThreadPool::workerLoop, this);
    }
}

LeaderFollowerThreadPool::~LeaderFollowerThreadPool() {
    shutdown();
    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
}

void LeaderFollowerThreadPool::enqueue(std::function<void()> task) {
    std::unique_lock<std::mutex> lock(queueMutex);
    
    // Log task enqueuing
    std::cout << "[LEADER-FOLLOWER] Task enqueued. Current queue size: " << taskQueue.size() 
              << " Thread ID: " << std::this_thread::get_id() << std::endl;
    
    taskQueue.push(std::move(task));
    condition.notify_one();
}

void LeaderFollowerThreadPool::shutdown() {
    isRunning = false;
    condition.notify_all();
}

void LeaderFollowerThreadPool::promoteNewLeader() {
    hasLeader = false;
    condition.notify_one();
}

void LeaderFollowerThreadPool::workerLoop() {
    while (isRunning) {
        std::function<void()> task;
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            
            // Wait for a task or shutdown signal
            condition.wait(lock, [this]() { 
                return !taskQueue.empty() || !isRunning; 
            });

            // Check if we should exit
            if (!isRunning && taskQueue.empty()) {
                return;
            }

            // Try to become the leader
            bool expectedLeader = false;
            if (!hasLeader.compare_exchange_strong(expectedLeader, true)) {
                // Another thread is already the leader, wait
                continue;
            }

            // Leader thread processing
            if (!taskQueue.empty()) {
                task = std::move(taskQueue.front());
                taskQueue.pop();
            }
        }

        // Execute the task if there is one
        if (task) {
            try {
                task();
            } catch (const std::exception& e) {
                std::cerr << "Task execution error: " << e.what() << std::endl;
            }
        }

        // Promote a new leader
        promoteNewLeader();
    }
}

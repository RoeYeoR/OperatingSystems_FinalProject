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
    // Unique identifier for this worker thread
    thread_local static size_t workerID = 0;
    static std::atomic<size_t> workerCounter{0};
    workerID = ++workerCounter;

    while (isRunning) {
        std::function<void()> task;
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            
            // Detailed logging for worker thread state
            std::cout << "[LEADER-FOLLOWER-DEBUG] Worker #" << workerID 
                      << " waiting. Queue size: " << taskQueue.size() 
                      << " Thread ID: " << std::this_thread::get_id() 
                      << " Running: " << isRunning << std::endl;

            // Wait for a task or shutdown signal
            condition.wait(lock, [this]() { 
                return !taskQueue.empty() || !isRunning; 
            });

            // Check if we should exit
            if (!isRunning && taskQueue.empty()) {
                std::cout << "[LEADER-FOLLOWER-DEBUG] Worker #" << workerID 
                          << " exiting. No more tasks." << std::endl;
                return;
            }

            // Try to become the leader
            bool expectedLeader = false;
            if (!hasLeader.compare_exchange_strong(expectedLeader, true)) {
                std::cout << "[LEADER-FOLLOWER-DEBUG] Worker #" << workerID 
                          << " failed to become leader. Waiting." << std::endl;
                // Another thread is already the leader, wait
                continue;
            }

            // Leader thread processing
            std::cout << "[LEADER-FOLLOWER-DEBUG] Worker #" << workerID 
                      << " BECAME LEADER! Thread ID: " << std::this_thread::get_id() << std::endl;

            if (!taskQueue.empty()) {
                task = std::move(taskQueue.front());
                taskQueue.pop();
                
                std::cout << "[LEADER-FOLLOWER-DEBUG] Leader Worker #" << workerID 
                          << " picked up task. Remaining queue: " << taskQueue.size() << std::endl;
            }
        }

        // Execute the task if there is one
        if (task) {
            try {
                std::cout << "[LEADER-FOLLOWER-DEBUG] Leader Worker #" << workerID 
                          << " executing task. Thread ID: " << std::this_thread::get_id() << std::endl;
                
                task();
                
                std::cout << "[LEADER-FOLLOWER-DEBUG] Leader Worker #" << workerID 
                          << " completed task execution." << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "[LEADER-FOLLOWER-ERROR] Worker #" << workerID 
                          << " task execution error: " << e.what() << std::endl;
            }
        }

        // Promote a new leader
        std::cout << "[LEADER-FOLLOWER-DEBUG] Worker #" << workerID 
                  << " promoting new leader." << std::endl;
        promoteNewLeader();
    }
}

#ifndef LEADER_FOLLOWER_HPP
#define LEADER_FOLLOWER_HPP

#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional>
#include <queue>

class LeaderFollowerThreadPool {
public:
    LeaderFollowerThreadPool(size_t threadCount = 4);
    ~LeaderFollowerThreadPool();

    void enqueue(std::function<void()> task);
    void shutdown();

private:
    void workerLoop();
    void promoteNewLeader();

    std::vector<std::thread> threads;
    std::queue<std::function<void()>> taskQueue;
    std::mutex queueMutex;
    std::condition_variable condition;
    std::atomic<bool> isRunning;
    std::atomic<bool> hasLeader;
};

#endif // LEADER_FOLLOWER_HPP

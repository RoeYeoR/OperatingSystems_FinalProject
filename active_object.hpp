#ifndef ACTIVE_OBJECT_HPP
#define ACTIVE_OBJECT_HPP

#include <vector>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional>

class ActiveObject {
public:
    ActiveObject();
    ~ActiveObject();

    void addTask(std::function<void()> task);
    void stop();

private:
    void workerThread();

    std::vector<std::thread> workers;
    std::queue<std::function<void()>> taskQueue;
    std::mutex queueMutex;
    std::condition_variable condition;
    std::atomic<bool> running;
};

#endif // ACTIVE_OBJECT_HPP

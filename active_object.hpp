#ifndef ACTIVE_OBJECT_HPP
#define ACTIVE_OBJECT_HPP

#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <thread>
#include <atomic>

class ActiveObject {
public:
    ActiveObject();
    ~ActiveObject();

    void addTask(std::function<void()> task);
    void stop();

private:
    void workerThread();

    std::queue<std::function<void()>> taskQueue;
    std::mutex queueMutex;
    std::condition_variable condition;
    std::atomic<bool> running;
    std::thread worker;
};

#endif // ACTIVE_OBJECT_HPP

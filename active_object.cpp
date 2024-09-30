#include "active_object.hpp"
#include <iostream>

ActiveObject::ActiveObject() : running(true) {
    // Start multiple worker threads (e.g., 4 worker threads)
    for (int i = 0; i < 4; ++i) {
        workers.emplace_back(&ActiveObject::workerThread, this);
    }
}

ActiveObject::~ActiveObject() {
    stop();
    for (std::thread &worker : workers) {
        if (worker.joinable()) {
            worker.join();
        }
    }
}

void ActiveObject::addTask(std::function<void()> task) {
    {
        std::unique_lock<std::mutex> lock(queueMutex);
        taskQueue.push(std::move(task));
        std::cout << "[ActiveObject] Task added to queue. Queue size: " << taskQueue.size() << std::endl;
    }
    condition.notify_one();
}

void ActiveObject::stop() {
    running = false;
    condition.notify_all();
}

void ActiveObject::workerThread() {
    while (running) {
        std::function<void()> task;

        {
            std::unique_lock<std::mutex> lock(queueMutex);
            condition.wait(lock, [this]() { return !taskQueue.empty() || !running; });

            if (!running && taskQueue.empty()) {
                return;
            }

            task = std::move(taskQueue.front());
            taskQueue.pop();
            std::cout << "[ActiveObject] Task dequeued and ready to execute." << std::endl;
        }

        if (task) {
            std::cout << "[ActiveObject] Executing task..." << std::endl;
            task();
        }
    }
}

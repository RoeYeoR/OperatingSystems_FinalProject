#include "active_object.hpp"

ActiveObject::ActiveObject() : running(true) {
    worker = std::thread(&ActiveObject::workerThread, this);
}

ActiveObject::~ActiveObject() {
    stop();
    if (worker.joinable()) {
        worker.join();
    }
}

void ActiveObject::addTask(std::function<void()> task) {
    {
        std::unique_lock<std::mutex> lock(queueMutex);
        taskQueue.push(std::move(task));
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
        }

        task();
    }
}

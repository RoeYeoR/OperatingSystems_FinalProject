#include "active_pipeline.hpp"
#include <iostream>

ActivePipelineStage::ActivePipelineStage(StageType type, size_t bufferSize)
    : stageType(type), 
      maxBufferSize(bufferSize), 
      running(true),
      nextStage(nullptr),
      previousStage(nullptr),
      transformTask([](Task t) { return t; }) {
    // Start worker thread for this stage
    worker = std::make_unique<std::thread>(&ActivePipelineStage::workerThread, this);
}

ActivePipelineStage::~ActivePipelineStage() {
    stop();
}

void ActivePipelineStage::setNextStage(std::shared_ptr<ActivePipelineStage> next) {
    std::unique_lock<std::mutex> lock(queueMutex);
    nextStage = next;
}

void ActivePipelineStage::connectPreviousStage(std::shared_ptr<ActivePipelineStage> prev) {
    std::unique_lock<std::mutex> lock(queueMutex);
    previousStage = prev;
}

void ActivePipelineStage::enqueue(Task task) {
    std::unique_lock<std::mutex> lock(queueMutex);
    
    // Wait if buffer is full
    condition.wait(lock, [this]() { 
        return taskQueue.size() < maxBufferSize || !running; 
    });

    if (!running) {
        throw std::runtime_error("Stage is not running");
    }

    // Log task enqueuing in pipeline stage
    std::cout << "[ACTIVE-PIPELINE] Task enqueued in " 
              << (stageType == StageType::SOURCE ? "SOURCE" : 
                  stageType == StageType::SINK ? "SINK" : "INTERMEDIATE") 
              << " stage. Current queue size: " << taskQueue.size() 
              << " Thread ID: " << std::this_thread::get_id() << std::endl;

    taskQueue.push(std::move(task));
    condition.notify_one();
}

void ActivePipelineStage::stop() {
    running = false;
    condition.notify_all();
    
    if (worker && worker->joinable()) {
        worker->join();
    }
}

bool ActivePipelineStage::isRunning() const {
    return running;
}

void ActivePipelineStage::setTransformation(std::function<Task(Task)> transform) {
    std::unique_lock<std::mutex> lock(queueMutex);
    transformTask = transform;
}

void ActivePipelineStage::workerThread() {
    while (running) {
        Task task;
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            
            // Wait for a task or stop signal
            condition.wait(lock, [this]() { 
                return !taskQueue.empty() || !running; 
            });

            // Check if we should exit
            if (!running && taskQueue.empty()) {
                return;
            }

            // Get the task
            task = std::move(taskQueue.front());
            taskQueue.pop();
            
            // Notify any waiting producers
            condition.notify_one();
        }

        // Process the task
        if (task) {
            try {
                // Apply stage-specific transformation
                task = transformTask(task);
                
                // Log task processing
                std::cout << "[ACTIVE-PIPELINE] Processing task in " 
                          << (stageType == StageType::SOURCE ? "SOURCE" : 
                              stageType == StageType::SINK ? "SINK" : "INTERMEDIATE") 
                          << " stage. Remaining queue size: " << taskQueue.size() 
                          << " Thread ID: " << std::this_thread::get_id() << std::endl;

                // Process the task
                processTask(task);
            } catch (const std::exception& e) {
                std::cerr << "Pipeline stage task error: " << e.what() << std::endl;
            }
        }
    }
}

void ActivePipelineStage::processTask(Task& task) {
    // Execute the task
    task();

    // Pass to next stage if exists
    if (nextStage && stageType != StageType::SINK) {
        std::cout << "[ACTIVE-PIPELINE] Passing task to next stage" << std::endl;
        try {
            nextStage->enqueue(task);
        } catch (const std::exception& e) {
            std::cerr << "Error passing task to next stage: " << e.what() << std::endl;
        }
    }
}

ActivePipeline::ActivePipeline() {}

ActivePipeline::~ActivePipeline() {
    stop();
}

void ActivePipeline::addStage(std::shared_ptr<ActivePipelineStage> stage) {
    if (!stages.empty()) {
        // Link previous stage to the new stage
        stages.back()->setNextStage(stage);
        stage->connectPreviousStage(stages.back());
    }
    stages.push_back(stage);
}

void ActivePipeline::start(ActivePipelineStage::Task initialTask) {
    if (stages.empty()) {
        throw std::runtime_error("No stages in pipeline");
    }

    // Start with the first stage (source stage)
    stages.front()->enqueue(initialTask);
}

void ActivePipeline::stop() {
    // Stop stages in reverse order
    for (auto it = stages.rbegin(); it != stages.rend(); ++it) {
        (*it)->stop();
    }
}

#include "active_pipeline.hpp"
#include <iostream>
#include <stdexcept>

// ActivePipelineStage Implementation
ActivePipelineStage::ActivePipelineStage(
    StageType type, 
    size_t bufferSize, 
    size_t maxRetries
) : 
    stageType(type), 
    maxBufferSize(bufferSize),
    maxRetries(maxRetries),
    running(true),
    nextStage(nullptr),
    previousStage(nullptr),
    transformTask([](Task t) { return t; }),
    errorHandler([](const std::exception& e) {
        std::cerr << "Unhandled pipeline stage error: " << e.what() << std::endl;
    })
{
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

    std::cout << "[PIPELINE-DEBUG] Enqueuing task in " 
              << (stageType == StageType::READ ? "READ" : 
                  stageType == StageType::SEND ? "SEND" : "PROCESS") 
              << " stage. Queue size: " << taskQueue.size() << std::endl;

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

void ActivePipelineStage::setErrorHandler(std::function<void(const std::exception&)> handler) {
    std::unique_lock<std::mutex> lock(queueMutex);
    errorHandler = handler;
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
                
                // Pass to next stage if exists and not a send stage
                if (nextStage && stageType != StageType::SEND) {
                    std::cout << "[PIPELINE-DEBUG] Passing task to next stage" << std::endl;
                    nextStage->enqueue(task);
                }
            } catch (const std::exception& e) {
                // Handle or propagate error
                errorHandler(e);
                
                // Optional: retry mechanism
                if (maxRetries > 0) {
                    // Implement retry logic here
                }
            }
        }
    }
}

// ActivePipeline Implementation
ActivePipeline::ActivePipeline(size_t concurrencyLevel) 
    : concurrencyLevel(concurrencyLevel),
      globalErrorHandler([](const std::exception& e) {
          std::cerr << "Unhandled pipeline error: " << e.what() << std::endl;
      }) 
{}

ActivePipeline::~ActivePipeline() {
    stop();
}

void ActivePipeline::addStage(std::shared_ptr<ActivePipelineStage> stage) {
    stages.push_back(stage);
    
    // Connect stages sequentially
    if (stages.size() > 1) {
        stages[stages.size() - 2]->setNextStage(stage);
        stage->connectPreviousStage(stages[stages.size() - 2]);
    }
}

void ActivePipeline::start(ActivePipelineStage::Task initialTask) {
    if (stages.empty()) {
        throw std::runtime_error("No stages in pipeline");
    }

    // Start with the initial task in the first stage
    stages.front()->enqueue(initialTask);
}

void ActivePipeline::stop() {
    for (auto& stage : stages) {
        stage->stop();
    }
}

void ActivePipeline::setGlobalErrorHandler(std::function<void(const std::exception&)> handler) {
    globalErrorHandler = handler;
}

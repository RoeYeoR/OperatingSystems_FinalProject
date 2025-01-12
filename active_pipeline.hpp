#ifndef ACTIVE_PIPELINE_HPP
#define ACTIVE_PIPELINE_HPP

#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional>
#include <memory>
#include <stdexcept>
#include <any>

class ActivePipelineStage {
public:
    // Generic task type that can handle any input/output
    using Task = std::function<std::any(std::any)>;
    
    enum class StageType {
        READ,       // Input stage
        PROCESS,    // Transformation stage
        SEND        // Output/Sink stage
    };

    // Primary constructor with default arguments
    explicit ActivePipelineStage(
        StageType type = StageType::PROCESS, 
        size_t bufferSize = 10,
        size_t maxRetries = 3
    );

    // Simplified constructors to resolve overloading
    explicit ActivePipelineStage(StageType type) : ActivePipelineStage(type, 10, 3) {}

    // Destructor
    ~ActivePipelineStage();

    // Pipeline stage management
    void setNextStage(std::shared_ptr<ActivePipelineStage> next);
    void connectPreviousStage(std::shared_ptr<ActivePipelineStage> prev);
    
    // Enhanced task submission with error handling
    void enqueue(Task task);
    void enqueue(std::function<void()> task);  // Additional overload
    void stop();
    bool isRunning() const;

    // Advanced stage configuration
    void setTransformation(std::function<Task(Task)> transform);
    void setErrorHandler(std::function<void(const std::exception&)> handler);

private:
    void workerThread();
    void processTask(Task& task);

    // Enhanced concurrency primitives
    std::queue<Task> taskQueue;
    std::queue<std::exception_ptr> errorQueue;
    std::mutex queueMutex;
    std::condition_variable condition;
    std::unique_ptr<std::thread> worker;

    // Stage configuration and state
    StageType stageType;
    size_t maxBufferSize;
    size_t maxRetries;
    std::atomic<bool> running;

    // Stage connections
    std::shared_ptr<ActivePipelineStage> nextStage;
    std::shared_ptr<ActivePipelineStage> previousStage;

    // Error and transformation handling
    std::function<Task(Task)> transformTask;
    std::function<void(const std::exception&)> errorHandler;
};

class ActivePipeline {
public:
    // Primary constructor with default argument
    explicit ActivePipeline(size_t concurrencyLevel = 4);

    // Destructor
    ~ActivePipeline();

    // Enhanced pipeline construction
    void addStage(std::shared_ptr<ActivePipelineStage> stage);
    void start(ActivePipelineStage::Task initialTask);
    void stop();
    void setGlobalErrorHandler(std::function<void(const std::exception&)> handler);

private:
    std::vector<std::shared_ptr<ActivePipelineStage>> stages;
    size_t concurrencyLevel;
    std::function<void(const std::exception&)> globalErrorHandler;
};

#endif // ACTIVE_PIPELINE_HPP

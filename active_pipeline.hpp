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

class ActivePipelineStage {
public:
    using Task = std::function<void()>;
    
    enum class StageType {
        SOURCE,
        PROCESSOR,
        SINK
    };

    ActivePipelineStage(
        StageType type = StageType::PROCESSOR, 
        size_t bufferSize = 10
    );
    ~ActivePipelineStage();

    // Pipeline stage management
    void setNextStage(std::shared_ptr<ActivePipelineStage> next);
    void connectPreviousStage(std::shared_ptr<ActivePipelineStage> prev);
    
    // Task submission and processing
    void enqueue(Task task);
    void stop();
    bool isRunning() const;

    // Stage-specific transformation
    void setTransformation(std::function<Task(Task)> transform);

private:
    void workerThread();
    void processTask(Task& task);

    // Concurrency primitives
    std::queue<Task> taskQueue;
    std::mutex queueMutex;
    std::condition_variable condition;
    std::unique_ptr<std::thread> worker;

    // Stage configuration
    StageType stageType;
    size_t maxBufferSize;
    std::atomic<bool> running;

    // Stage connections
    std::shared_ptr<ActivePipelineStage> nextStage;
    std::shared_ptr<ActivePipelineStage> previousStage;

    // Optional task transformation
    std::function<Task(Task)> transformTask;
};

class ActivePipeline {
public:
    ActivePipeline();
    ~ActivePipeline();

    // Pipeline construction
    void addStage(std::shared_ptr<ActivePipelineStage> stage);
    void start(ActivePipelineStage::Task initialTask);
    void stop();

private:
    std::vector<std::shared_ptr<ActivePipelineStage>> stages;
};

#endif // ACTIVE_PIPELINE_HPP

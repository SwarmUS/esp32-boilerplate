#include "taskManager/AbstractTask.h"
#include <task.h>

AbstractTask::AbstractTask(const char* taskName, portSHORT stackDepth, UBaseType_t priority) {
    m_taskName = taskName;
    m_stackDepth = stackDepth;
    m_priority = priority;
    m_taskHandle = NULL;
}

void AbstractTask::start() {
    xTaskCreate(wrapper, m_taskName, m_stackDepth, this, m_priority, &this->m_taskHandle);
}

TaskHandle_t AbstractTask::getTaskHandle() const { return m_taskHandle; }

void AbstractTask::wrapper(void* params) {
    auto* task = static_cast<AbstractTask*>(params);
    task->task();
}
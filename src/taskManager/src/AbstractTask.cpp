#include "taskManager/AbstractTask.h"
#include <task.h>

template <unsigned int T>
AbstractTask<T>::AbstractTask(const char* taskName, UBaseType_t priority) {
    m_taskName = taskName;
    m_priority = priority;
    m_taskHandle = NULL;
}

template <unsigned int T>
void AbstractTask<T>::start() {
    m_taskHandle =
        xTaskCreateStatic(wrapper, m_taskName, T, this, m_priority, m_stackArray, &m_taskBuffer);
}

template <unsigned int T>
TaskHandle_t AbstractTask<T>::getTaskHandle() const {
    return m_taskHandle;
}

template <unsigned int T>
void AbstractTask<T>::wrapper(void* params) {
    auto* task = static_cast<AbstractTask*>(params);
    task->task();
}
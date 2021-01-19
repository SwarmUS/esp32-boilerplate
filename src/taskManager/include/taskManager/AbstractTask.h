#ifndef ABSTRACTTASK_H
#define ABSTRACTTASK_H

#include <FreeRTOS.h>
#include <task.h>
#include <algorithm>

class AbstractTask {
  public:
    AbstractTask(const char* taskName, portSHORT stackDepth, UBaseType_t priority);
    virtual ~AbstractTask() = default;
    void start();
    TaskHandle_t getTaskHandle() const;

    virtual void task() = 0;

  protected:
    static void wrapper(void* params);

    const char* m_taskName;
    portSHORT m_stackDepth;
    UBaseType_t m_priority;
    TaskHandle_t m_taskHandle;
};

#endif // ABSTRACTTASK_H

#include <algorithm>
#ifndef ABSTRACTTASK_H
#define ABSTRACTTASK_H

#include <FreeRTOS.h>
#include <task.h>

class AbstractTask {
  public:
    AbstractTask(const char* taskName, portSHORT stackDepth, UBaseType_t priority);
    virtual ~AbstractTask() = default;
    void start();

    virtual void task() = 0;

  protected:
    static void wrapper(void* params);

    const char* m_taskName;
    portSHORT m_stackDepth;
    UBaseType_t m_priority;
    TaskHandle_t m_taskHandle;
};

#endif // ABSTRACTTASK_H

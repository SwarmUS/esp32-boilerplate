#ifndef ABSTRACTTASK_H
#define ABSTRACTTASK_H

#include <FreeRTOS.h>
#include <algorithm>
#include <array>
#include <task.h>

template <unsigned int stackSize>
class AbstractTask {
  public:
    AbstractTask(const char* taskName, UBaseType_t priority);
    virtual ~AbstractTask();
    void start();
    TaskHandle_t getTaskHandle() const;

    virtual void task() = 0;

  protected:
    static void wrapper(void* params);

    const char* m_taskName;
    std::array<uint8_t*, stackSize> m_stackArray;
    StaticTask_t m_taskBuffer;
    UBaseType_t m_priority;
    TaskHandle_t m_taskHandle;
};

#endif // ABSTRACTTASK_H

#include "AbstractTask.h"
#include "Task.h"
#include "bsp/Container.h"

#ifdef __cplusplus
extern "C" {
#endif

class StmMessageSenderTask : public AbstractTask<2 * configMINIMAL_STACK_SIZE> {
  public:
    StmMessageSenderTask(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority) {}

    ~StmMessageSenderTask() override = default;

  private:
    void task() override {
        auto& spi = BspContainer::getSpiStm();
        while (true) {
            if (!spi.isBusy()) {
                const char message[] = "Hello STM";
                spi.send((uint8_t*)message, sizeof(message));
            }
            Task::delay(100);
        }
    }
};

void app_main(void) {
    IBSP* bsp = &BspContainer::getBSP();
    bsp->initChip();

    static StmMessageSenderTask s_spiMessageSend("spi_send", tskIDLE_PRIORITY + 1);

    // s_spiMessageSend.start();
}

#ifdef __cplusplus
}
#endif

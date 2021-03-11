#include "AbstractTask.h"
#include "NetworkContainer.h"
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

class TCPMessageSenderTask : public AbstractTask<3 * configMINIMAL_STACK_SIZE> {
  public:
    TCPMessageSenderTask(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority) {}

    ~TCPMessageSenderTask() override = default;

  private:
    void task() override {
        auto& client = NetworkContainer::getTCPClient();
        while (NetworkContainer::getNetworkManager().getNetworkStatus() !=
               NetworkStatus::Connected) {
            Task::delay(500);
        }
        while (true) {
            if (client.setDestination("10.0.0.162")) {
                const char message[] = "Hello Server";
                client.send((uint8_t*)message, sizeof(message));
            }
            Task::delay(100);
        }
    }
};

void app_main(void) {
    IBSP* bsp = &BspContainer::getBSP();
    bsp->initChip();

    static StmMessageSenderTask s_spiMessageSend("spi_send", tskIDLE_PRIORITY + 1);
    static TCPMessageSenderTask s_tcpMessageSender("tcp_send", tskIDLE_PRIORITY + 1);

    // s_spiMessageSend.start();
    s_tcpMessageSender.start();
}

#ifdef __cplusplus
}
#endif

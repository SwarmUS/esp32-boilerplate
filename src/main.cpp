#include "AbstractTask.h"
#include "NetworkContainer.h"
#include "Task.h"
#include "bsp/Container.h"
#include "logger/LoggerContainer.h"

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
        while (!spi.isConnected()) {
            Task::delay(500);
        }
        while (true) {
            if (!spi.isBusy() && spi.isConnected()) {
                const char message[] = "Hello STM";
                spi.send((uint8_t*)message, sizeof(message));
                LoggerContainer::getLogger().log(LogLevel::Info, "Sent message to spi");
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
        char message[50] = "Hello Server!";
        uint16_t id = 0;
        auto& client = NetworkContainer::getNetworkOutputStream();
        while (NetworkContainer::getNetworkManager().getNetworkStatus() !=
               NetworkStatus::Connected) {
            Task::delay(500);
        }
        while (true) {

            if (client.setDestination("10.0.0.163")) {
                client.send((uint8_t*)message, sizeof(message));
                id++;
                client.close();
            } else {
                LoggerContainer::getLogger().log(LogLevel::Warn, "Failed to set destination");
            }
            Task::delay(100);
        }
    }
};

class TCPMessageReceiverTask : public AbstractTask<3 * configMINIMAL_STACK_SIZE> {
  public:
    TCPMessageReceiverTask(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority) {}

    ~TCPMessageReceiverTask() override = default;

  private:
    void task() override {
        char message[50];
        auto& server = NetworkContainer::getNetworkInputStream();
        while (NetworkContainer::getNetworkManager().getNetworkStatus() !=
               NetworkStatus::Connected) {
            Task::delay(500);
        }
        while (true) {
            if (!server.receive((uint8_t*)message, sizeof(message))) {
                LoggerContainer::getLogger().log(LogLevel::Error, "Failed to receive");
            }
            LoggerContainer::getLogger().log(LogLevel::Info, "Received: %s", message);
            Task::delay(100);
        }
    }
};

void app_main(void) {
    IBSP* bsp = &BspContainer::getBSP();
    bsp->initChip();
    INetworkManager* networkManager = &NetworkContainer::getNetworkManager();
    networkManager->start();

    static StmMessageSenderTask s_spiMessageSend("spi_send", tskIDLE_PRIORITY + 1);
    static TCPMessageSenderTask s_tcpMessageSender("tcp_send", tskIDLE_PRIORITY + 1);
    static TCPMessageReceiverTask s_tcpMessageReceiver("tcp_receive", tskIDLE_PRIORITY + 1);

     s_spiMessageSend.start();
    //s_tcpMessageReceiver.start();
    //s_tcpMessageSender.start();
}

#ifdef __cplusplus
}
#endif

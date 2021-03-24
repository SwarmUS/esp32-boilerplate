#include "AbstractTask.h"
#include "NetworkContainer.h"
#include "Task.h"
#include "bsp/Container.h"
#include "hivemind-host/HiveMindHostDeserializer.h"
#include "logger/LoggerContainer.h"
#include "message-handler/MessageHandlerContainer.h"
#include "message-handler/MessageSender.h"
#include "message-handler/NetworkDeserializer.h"
#include "message-handler/NetworkSerializer.h"

#ifdef __cplusplus
extern "C" {
#endif

class StmMessageSenderTask : public AbstractTask<2 * configMINIMAL_STACK_SIZE> {
  public:
    StmMessageSenderTask(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority), m_logger(LoggerContainer::getLogger()) {}

    ~StmMessageSenderTask() override = default;

  private:
    ILogger& m_logger;

    void task() override {
        auto& spi = BspContainer::getSpiStm();
        while (!spi.isConnected()) {
            Task::delay(500);
        }
        HiveMindHostSerializer serializer(spi);
        MessageSender messageSender(MessageHandlerContainer::getHivemindOutputQueue(), serializer,
                                    BspContainer::getBSP(), m_logger);
        while (!spi.isConnected()) {
            Task::delay(100);
        }

        while (true) {
            // Note: this is place-holder logic for handling the greet process. The proper loop for
            // this will be addressed in future development
            while (!spi.isConnected()) {
                messageSender.greet();
                if (!messageSender.processAndSerialize()) {
                    m_logger.log(LogLevel::Warn, "Fail to process/serialize spi while greeting");
                }
            }
            if (!messageSender.processAndSerialize()) {
                m_logger.log(LogLevel::Warn, "Fail to process/serialize spi");
            }
            Task::delay(100);
        }
    }
};

class StmMessageDispatcherTask : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  public:
    StmMessageDispatcherTask(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority), m_logger(LoggerContainer::getLogger()) {}

    void task() override {
        auto& spi = BspContainer::getSpiStm();
        HiveMindHostDeserializer deserializer(spi);
        NetworkAPIHandler networkApiHandler = MessageHandlerContainer::createNetworkApiHandler();
        MessageDispatcher dispatcher =
            MessageHandlerContainer::createMessageDispatcher(deserializer, networkApiHandler);

        while (true) {
            if (spi.isConnected()) {
                if (!dispatcher.deserializeAndDispatch()) {
                    m_logger.log(LogLevel::Error, "Failed to deserialize/dispatch STM");
                }
            }
            m_logger.log(LogLevel::Warn, "Cannot deserialize/dispatch STM while disconnected");
            Task::delay(500);
        }
    }

  private:
    ILogger& m_logger;
};

class UnicastMessageSenderTask : public AbstractTask<3 * configMINIMAL_STACK_SIZE> {
  public:
    UnicastMessageSenderTask(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority),
        m_logger(LoggerContainer::getLogger()),
        m_networkManager(NetworkContainer::getNetworkManager()) {}

    ~UnicastMessageSenderTask() override = default;

  private:
    ILogger& m_logger;
    INetworkManager& m_networkManager;
    void task() override {
        char message[50] = "Hello Server!";
        uint16_t id = 0;
        auto& stream = NetworkContainer::getNetworkOutputStream();
        NetworkSerializer serializer(stream, m_networkManager);
        MessageSender messageSender(MessageHandlerContainer::getUnicastOutputQueue(), serializer,
                                    BspContainer::getBSP(), m_logger);

        while (NetworkContainer::getNetworkManager().getNetworkStatus() !=
               NetworkStatus::Connected) {
            Task::delay(500);
        }
        while (true) {
            if (!messageSender.processAndSerialize()) {
                m_logger.log(LogLevel::Error, "Fail to process/serialize unicast");
            }
        }
    }
};

class UnicastMessageDispatcher : public AbstractTask<3 * configMINIMAL_STACK_SIZE> {
  public:
    UnicastMessageDispatcher(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority),
        m_logger(LoggerContainer::getLogger()),
        m_networkManager(NetworkContainer::getNetworkManager()) {}

    ~UnicastMessageDispatcher() override = default;

  private:
    ILogger& m_logger;
    INetworkManager& m_networkManager;
    void task() override {
        auto& stream = NetworkContainer::getNetworkInputStream();

        NetworkDeserializer deserializer(stream, m_networkManager);
        NetworkAPIHandler networkApiHandler = MessageHandlerContainer::createNetworkApiHandler();
        MessageDispatcher dispatcher =
            MessageHandlerContainer::createMessageDispatcher(deserializer, networkApiHandler);

        while (NetworkContainer::getNetworkManager().getNetworkStatus() !=
               NetworkStatus::Connected) {
            Task::delay(500);
        }
        while (true) {
            if (!dispatcher.deserializeAndDispatch()) {
                m_logger.log(LogLevel::Error, "Fail to deserialize/dispatch unicast");
            }
        }
    }
};

void app_main(void) {
    IBSP* bsp = &BspContainer::getBSP();
    bsp->initChip();
    INetworkManager* networkManager = &NetworkContainer::getNetworkManager();
    networkManager->start();

    static StmMessageSenderTask s_spiMessageSend("spi_send", tskIDLE_PRIORITY + 1);
    static UnicastMessageSenderTask s_tcpMessageSender("tcp_send", tskIDLE_PRIORITY + 1);
    static UnicastMessageDispatcher s_tcpMessageReceiver("tcp_receive", tskIDLE_PRIORITY + 1);

    s_spiMessageSend.start();
    s_tcpMessageReceiver.start();
    s_tcpMessageSender.start();
}

#ifdef __cplusplus
}
#endif

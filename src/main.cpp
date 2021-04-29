#include "AbstractTask.h"
#include "NetworkContainer.h"
#include "Task.h"
#include "bsp/Container.h"
#include "logger/LoggerContainer.h"
#include "message-handler/MessageHandlerContainer.h"
#include "message-handler/MessageSender.h"
#include "message-handler/NetworkDeserializer.h"
#include "message-handler/NetworkSerializer.h"
#include "pheromones/HiveMindHostAccumulatorSerializer.h"
#include "pheromones/HiveMindHostDeserializer.h"

#ifdef __cplusplus
extern "C" {
#endif

class HiveMindMessageSender : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  public:
    HiveMindMessageSender(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority), m_logger(LoggerContainer::getLogger()) {}

    ~HiveMindMessageSender() override = default;

  private:
    ILogger& m_logger;

    void task() override {
        auto& spi = BspContainer::getSpiStm();
        HiveMindHostAccumulatorSerializer serializer(spi);
        MessageSender messageSender(MessageHandlerContainer::getHivemindOutputQueue(), serializer,
                                    BspContainer::getBSP(), m_logger);
        while (!spi.isConnected()) {
            Task::delay(100);
        }
        while (true) {
            // Greet HiveMind to obtain uuid
            if (BspContainer::getBSP().getHiveMindUUID() == 0) {
                messageSender.greet();
                if (!messageSender.processAndSerialize()) {
                    m_logger.log(LogLevel::Error, "Fail to process/serialize spi while greeting");
                }
                Task::delay(1000);
            }
            // Standard loop
            else if (!messageSender.processAndSerialize()) {
                m_logger.log(LogLevel::Warn, "Fail to process/serialize spi");
            }
            // Handle disconnections
            if (!spi.isConnected()) {
                m_logger.log(LogLevel::Error, "Lost connection to HiveMind");
                BspContainer::getBSP().setHiveMindUUID(0);
            }
        }
    }
};

class HiveMindDispatcher : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  public:
    HiveMindDispatcher(const char* taskName, UBaseType_t priority) :
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
            } else {
                m_logger.log(LogLevel::Warn, "Cannot deserialize/dispatch STM while disconnected");
                Task::delay(500);
            }
        }
    }

  private:
    ILogger& m_logger;
};

class UnicastMessageSenderTask : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
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

class UnicastMessageDispatcher : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  public:
    UnicastMessageDispatcher(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority), m_logger(LoggerContainer::getLogger()) {}

    ~UnicastMessageDispatcher() override = default;

  private:
    ILogger& m_logger;
    void task() override {
        auto& stream = NetworkContainer::getNetworkInputStream();

        NetworkDeserializer deserializer(stream);
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

class BroadcastMessageSenderTask : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  public:
    BroadcastMessageSenderTask(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority),
        m_logger(LoggerContainer::getLogger()),
        m_networkManager(NetworkContainer::getNetworkManager()) {}

    ~BroadcastMessageSenderTask() override = default;

  private:
    ILogger& m_logger;
    INetworkManager& m_networkManager;
    void task() override {
        auto& stream = NetworkContainer::getNetworkBroadcast();
        HiveMindHostAccumulatorSerializer serializer(stream);
        MessageSender messageSender(MessageHandlerContainer::getBroadcastOutputQueue(), serializer,
                                    BspContainer::getBSP(), m_logger);

        while (NetworkContainer::getNetworkManager().getNetworkStatus() !=
                   NetworkStatus::Connected ||
               BspContainer::getBSP().getHiveMindUUID() == 0) {
            Task::delay(100);
        }
        while (true) {
            if (!messageSender.processAndSerialize()) {
                m_logger.log(LogLevel::Error, "Fail to process/serialize broadcast");
            }
        }
    }
};

class BroadcastMessageDispatcher : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  public:
    BroadcastMessageDispatcher(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority), m_logger(LoggerContainer::getLogger()) {}

    ~BroadcastMessageDispatcher() override = default;

  private:
    ILogger& m_logger;
    void task() override {
        auto& stream = NetworkContainer::getNetworkBroadcast();

        HiveMindHostDeserializer deserializer(stream);
        NetworkAPIHandler networkApiHandler = MessageHandlerContainer::createNetworkApiHandler();
        MessageDispatcher dispatcher =
            MessageHandlerContainer::createMessageDispatcher(deserializer, networkApiHandler);

        while (NetworkContainer::getNetworkManager().getNetworkStatus() !=
               NetworkStatus::Connected) {
            Task::delay(500);
        }
        while (true) {
            if (!dispatcher.deserializeAndDispatch()) {
                m_logger.log(LogLevel::Error, "Fail to deserialize/dispatch broadcast");
            }
        }
    }
};

class BroadcastIPTask : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  public:
    BroadcastIPTask(const char* taskName,
                    UBaseType_t priority,
                    IBSP& bsp,
                    INetworkManager& networkManager,
                    ILogger& logger) :
        AbstractTask(taskName, priority),
        m_bsp(bsp),
        m_networkManager(networkManager),
        m_logger(logger) {}
    ~BroadcastIPTask() override = default;

  private:
    IBSP& m_bsp;
    INetworkManager& m_networkManager;
    ILogger& m_logger;
    void task() override {
        MessageDTO message;
        message.setDestinationId(0);
        while (true) {

            // Wait for device to be connected and having a valid id
            while (NetworkContainer::getNetworkManager().getNetworkStatus() !=
                       NetworkStatus::Connected ||
                   BspContainer::getBSP().getHiveMindUUID() == 0) {
                Task::delay(100);
            }
            if (message.getSourceId() != BspContainer::getBSP().getHiveMindUUID()) {
                message.setSourceId(BspContainer::getBSP().getHiveMindUUID());
            }
            auto& broadcastQueue = MessageHandlerContainer::getBroadcastOutputQueue();
            IPDiscoveryDTO ipDiscoveryDto(m_networkManager.getSelfIP());
            MessageDTO message(m_bsp.getHiveMindUUID(), 0, ipDiscoveryDto);
            broadcastQueue.push(message);
            // Only send message every five seconds
            Task::delay(5000);
        }
    }
};

class StressTestsCommunicationsSend : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  public:
    StressTestsCommunicationsSend(const char* taskName,
                                  UBaseType_t priority,
                                  IBSP& bsp,
                                  INetworkManager& networkManager,
                                  ILogger& logger) :
        AbstractTask(taskName, priority),
        m_bsp(bsp),
        m_networkManager(networkManager),
        m_logger(logger) {}
    ~StressTestsCommunicationsSend() override = default;

  private:
    IBSP& m_bsp;
    INetworkManager& m_networkManager;
    ILogger& m_logger;
    void task() override {
        MessageDTO message;
        while (true) {
            if (m_networkManager.getNetworkStatus() != NetworkStatus::Connected) {
                Task::delay(1000);
                continue;
            }
            uint8_t dest = 1;
            if (m_bsp.getHiveMindUUID() == 1) {
                dest = 2;
            }
            auto& sending = NetworkContainer::getNetworkOutputStream();
            if (m_networkManager.getIPFromAgentID(dest).has_value()) {
                if (sending.setDestination(m_networkManager.getIPFromAgentID(dest).value())) {
                    char message[] = "Communications tests";
                    sending.send((uint8_t*)message, sizeof(message));
                    sending.close();
                }
            }
            Task::delay(100);
        }
    }
};

class StressTestsCommunicationsReceive : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  public:
    StressTestsCommunicationsReceive(const char* taskName,
                                     UBaseType_t priority,
                                     IBSP& bsp,
                                     INetworkManager& networkManager,
                                     ILogger& logger) :
        AbstractTask(taskName, priority),
        m_bsp(bsp),
        m_networkManager(networkManager),
        m_logger(logger) {}
    ~StressTestsCommunicationsReceive() override = default;

  private:
    IBSP& m_bsp;
    INetworkManager& m_networkManager;
    ILogger& m_logger;
    void task() override {
        MessageDTO message;
        while (true) {
            if (m_networkManager.getNetworkStatus() != NetworkStatus::Connected) {
                Task::delay(1000);
                continue;
            }
            char message[] = "Communications tests";
            char buffer[sizeof(message)] = "";
            auto& receiving = NetworkContainer::getNetworkInputStream();
            if (receiving.receive((uint8_t*)buffer, sizeof(message))) {
                m_logger.log(LogLevel::Info, "Message received: %s", buffer);
                receiving.closeCurrentClient();
            }
        }
    }
};

void app_main(void) {
    IBSP* bsp = &BspContainer::getBSP();
    bsp->initChip();
    INetworkManager* networkManager = &NetworkContainer::getNetworkManager();
    networkManager->start();

    /*static HiveMindMessageSender s_spiMessageSend("hivemind_send", tskIDLE_PRIORITY + 1);
    static HiveMindDispatcher s_spiDispatch("hivemind_receive", tskIDLE_PRIORITY + 1);

    static UnicastMessageSenderTask s_tcpMessageSender("unicast_send", tskIDLE_PRIORITY + 1);
    static UnicastMessageDispatcher s_tcpMessageReceiver("unicast_receive", tskIDLE_PRIORITY + 1);*/

    static BroadcastMessageSenderTask s_broadcastMessageSender("broadcast_send",
                                                               tskIDLE_PRIORITY + 1);
    static BroadcastMessageDispatcher s_broadcastReceiver("broadcast_send", tskIDLE_PRIORITY + 1);
    static BroadcastIPTask s_broadcastIpTask(
        "broad_casting_ip", configMINIMAL_STACK_SIZE, BspContainer::getBSP(),
        NetworkContainer::getNetworkManager(), LoggerContainer::getLogger());

    static StressTestsCommunicationsSend s_testSend(
        "test_send", tskIDLE_PRIORITY + 1, BspContainer::getBSP(),
        NetworkContainer::getNetworkManager(), LoggerContainer::getLogger());

    static StressTestsCommunicationsReceive s_testReceive(
        "test_receive", tskIDLE_PRIORITY + 1, BspContainer::getBSP(),
        NetworkContainer::getNetworkManager(), LoggerContainer::getLogger());

    /*s_spiMessageSend.start();
    s_spiDispatch.start();

    s_tcpMessageReceiver.start();
    s_tcpMessageSender.start();*/

    s_broadcastMessageSender.start();
    s_broadcastReceiver.start();
    s_broadcastIpTask.start();

    s_testSend.start();
    s_testReceive.start();
}

#ifdef __cplusplus
}
#endif

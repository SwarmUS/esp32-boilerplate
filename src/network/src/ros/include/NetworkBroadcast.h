#ifndef HIVE_CONNECT_NETWORKBROADCAST_H
#define HIVE_CONNECT_NETWORKBROADCAST_H

#include "BaseTask.h"
#include "INetworkBroadcast.h"
#include "bsp/IBSP.h"
#include "c-common/circular_buff.h"
#include "hive_connect/Broadcast.h"
#include "logger/ILogger.h"
#include <array>
#include <condition_variable>
#include <netinet/in.h>
#include <ros/ros.h>

constexpr uint32_t g_maxBroadcastReceiveSize = 2048;

class NetworkBroadcast : public INetworkBroadcast {
  public:
    NetworkBroadcast(ILogger& logger,
                     IBSP& bsp,
                     const char* publishingTopicPrefix,
                     const char* subscribingTopicPrefix);
    ~NetworkBroadcast() override;

    bool send(const uint8_t* data, uint16_t length) override;
    bool receive(uint8_t* data, uint16_t length) override;
    bool start() override;
    bool stop() override;

  private:
    ILogger& m_logger;
    IBSP& m_bsp;
    CircularBuff m_circularBuffer;
    std::array<uint8_t, g_maxBroadcastReceiveSize> m_data;

    std::string m_pubTopicPrefix;
    std::string m_subTopicPrefix;

    ros::Publisher m_publisher;
    ros::Subscriber m_subscriber;

    std::condition_variable m_conditionVariable;
    std::mutex m_mutex;
    bool m_receivedBytes;

    void handleReception(const hive_connect::Broadcast& msg);
};

#endif // HIVE_CONNECT_NETWORKBROADCAST_H

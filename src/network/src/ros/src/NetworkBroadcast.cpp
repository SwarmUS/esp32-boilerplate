#include "NetworkBroadcast.h"
#include <unistd.h>

void NetworkBroadcast::handleReception(const hive_connect::Broadcast& msg) {
    if (msg.data.empty()) {
        m_logger.log(LogLevel::Warn, "Received empty data in broadcast");
        return;
    }
    CircularBuff_put(&m_circularBuffer, msg.data.data(), msg.data.size());
}

NetworkBroadcast::NetworkBroadcast(ILogger& logger,
                                   IBSP& bsp,
                                   const char* publishingTopicPrefix,
                                   const char* subscribingTopicPrefix) :
    m_logger(logger), m_bsp(bsp) {

    m_pubTopic = publishingTopicPrefix + std::to_string(m_bsp.getHiveMindUUID());
    m_subTopic = subscribingTopicPrefix + std::to_string(m_bsp.getHiveMindUUID());

    CircularBuff_init(&m_circularBuffer, m_data.data(), m_data.size());
    ros::NodeHandle handle("~");
    m_publisher = handle.advertise<hive_connect::Broadcast>(m_pubTopic, 1000);
    m_subscriber = handle.subscribe(m_subTopic, 1000, &NetworkBroadcast::handleReception, this);
}

NetworkBroadcast::~NetworkBroadcast() { stop(); }

bool NetworkBroadcast::start() { return true; }

bool NetworkBroadcast::stop() { return true; }

bool NetworkBroadcast::send(const uint8_t* data, uint16_t length) {
    hive_connect::Broadcast msg;
    msg.source_robot = m_bsp.getHiveMindUUID();
    msg.data.insert(msg.data.end(), data, &data[length - 1]);
    m_publisher.publish(msg);
    return true;
}

bool NetworkBroadcast::receive(uint8_t* data, uint16_t length) {
    if (CircularBuff_getLength(&m_circularBuffer) < length) {
        // Add condition var
    }

    return CircularBuff_put(&m_circularBuffer, data, length) == CircularBuff_Ret_Ok;
}
#include "NetworkManager.h"
#include <ros/ros.h>

NetworkManager::NetworkManager(ILogger& logger, IHashMap<uint16_t, uint32_t>& hashMap) :
    AbstractNetworkManager(logger, hashMap) {}

NetworkStatus NetworkManager::getNetworkStatus() const { return NetworkStatus::Connected; }

uint32_t NetworkManager::getSelfIP() const {
    ros::NodeHandle nodeHandle("~");
    uint32_t port = nodeHandle.param("tcp_listen_port", 54321);
    m_logger.log(LogLevel::Info, "Returned port value of %d", port);
    return port;
}

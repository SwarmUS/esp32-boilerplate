#include "NetworkManager.h"
#include <ros/ros.h>

NetworkManager::NetworkManager(ILogger& logger, IHashMap<uint16_t, uint32_t>& hashMap, IBSP& bsp, INetworkBroadcast& networkBroadcast) :
    AbstractNetworkManager(logger, hashMap), m_bsp(bsp), m_networkBroadcast(networkBroadcast) {}

NetworkStatus NetworkManager::getNetworkStatus() const { 
    if (m_bsp.getHiveMindUUID() == 0) {
        return NetworkStatus::Connecting;
    }

    if (!m_networkBroadcast.isStarted()) {
        m_networkBroadcast.start();
    }

    return m_networkBroadcast.isStarted() ? NetworkStatus::Connected : NetworkStatus::Connecting; 
    }

uint32_t NetworkManager::getSelfIP() const {
    ros::NodeHandle nodeHandle("~");
    uint32_t port = nodeHandle.param("tcp_listen_port", 54321);
    return port;
}

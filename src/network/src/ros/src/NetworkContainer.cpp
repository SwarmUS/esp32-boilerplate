#include "NetworkContainer.h"
#include "DummyNetworkManager.h"
#include "NetworkBroadcast.h"
#include "NetworkInputStream.h"
#include "NetworkOutputStream.h"
#include "bsp/Container.h"
#include <ros/ros.h>

INetworkManager& NetworkContainer::getNetworkManager() {
    static DummyNetworkManager s_networkManager;

    return s_networkManager;
}

INetworkInputStream& NetworkContainer::getNetworkInputStream() {
    ros::NodeHandle nodeHandle("~");
    int port = nodeHandle.param("tcp_listen_port", 54321);
    static NetworkInputStream s_inputStream(LoggerContainer::getLogger(), port);

    static std::once_flag s_startOnce;
    std::call_once(s_startOnce, [&]() {
        if (!s_inputStream.start()) {
            LoggerContainer::getLogger().log(LogLevel::Error, "Failed to start tcp server");
        }
    });

    return s_inputStream;
}

INetworkOutputStream& NetworkContainer::getNetworkOutputStream() {
    static NetworkOutputStream s_outputStream(LoggerContainer::getLogger());

    return s_outputStream;
}

INetworkBroadcast& NetworkContainer::getNetworkBroadcast() {
    // Could be supplied by launch file when integrated in simulation
    std::string outputPrefix("/Communication/broadcastOutput/" +
                             std::to_string(BspContainer::getBSP().getHiveMindUUID()));
    std::string inputPrefix("/Communication/broadcastInput/" +
                            std::to_string(BspContainer::getBSP().getHiveMindUUID()));
    static NetworkBroadcast s_broadcast(LoggerContainer::getLogger(), BspContainer::getBSP(),
                                        outputPrefix.c_str(), inputPrefix.c_str());
    return s_broadcast;
}
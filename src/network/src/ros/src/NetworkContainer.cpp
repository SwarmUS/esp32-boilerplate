#include "NetworkContainer.h"
#include "DummyNetworkManager.h"
#include "DummyNetworkOutputStream.h"
#include "NetworkInputStream.h"
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
    static DummyNetworkOutputStream s_outputStream;

    return s_outputStream;
}
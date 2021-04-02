#ifndef HIVE_CONNECT_NETWORKBROADCAST_H
#define HIVE_CONNECT_NETWORKBROADCAST_H

#include "INetworkBroadcast.h"

class NetworkBroadcast : public INetworkBroadcast {
  public:
    NetworkBroadcast() = default;
    ~NetworkBroadcast() override = default;

    bool send(const uint8_t* data, uint16_t length) override {
        (void)data;
        (void)length;
        return false;
    }

    bool receive(uint8_t* data, uint16_t length) override {
        (void)data;
        (void)length;
        return false;
    }

    bool start() override { return false; }

    bool stop() override { return false; }
};

#endif // HIVE_CONNECT_NETWORKBROADCAST_H

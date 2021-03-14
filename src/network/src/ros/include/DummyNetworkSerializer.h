#ifndef HIVE_CONNECT_DUMMYNETWORKSERIALIZER_H
#define HIVE_CONNECT_DUMMYNETWORKSERIALIZER_H

#include "INetworkOutputStream.h"

class DummyNetworkSerializer : public INetworkOutputStream {
  public:
    DummyNetworkSerializer() = default;
    ~DummyNetworkSerializer() = default;

    bool receive(uint8_t* data, uint16_t length) override {
        (void)data;
        (void)length;
        return false;
    };

    bool send(const uint8_t* data, uint16_t length) override {
        (void)data;
        (void)length;
        return false;
    };

    bool setDestination(const char* destination) override {
        (void)destination;
        return false;
    }

    bool close() override { return false; };
};

#endif // HIVE_CONNECT_DUMMYNETWORKSERIALIZER_H

#ifndef HIVE_CONNECT_DUMMYNETWORKOUTPUTSTREAM_H
#define HIVE_CONNECT_DUMMYNETWORKOUTPUTSTREAM_H

#include "INetworkOutputStream.h"

class DummyNetworkOutputStream : public INetworkOutputStream {
  public:
    DummyNetworkOutputStream() = default;
    ~DummyNetworkOutputStream() = default;

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

#endif // HIVE_CONNECT_DUMMYNETWORKOUTPUTSTREAM_H

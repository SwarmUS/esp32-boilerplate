#ifndef HIVE_CONNECT_DUMMYNETWORKINPUTSTREAM_H
#define HIVE_CONNECT_DUMMYNETWORKINPUTSTREAM_H

#include "INetworkInputStream.h"

class DummyNetworkInputStream : public INetworkInputStream {
  public:
    DummyNetworkInputStream() = default;
    ~DummyNetworkInputStream() override = default;

    bool receive(uint8_t* data, uint16_t length) override {
        (void)data;
        (void)length;
        return false;
    };

    bool isReady() override { return false; }

    bool start() override { return false; };
    bool stop() override { return false; };
};

#endif // HIVE_CONNECT_DUMMYNETWORKINPUTSTREAM_H

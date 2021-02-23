#ifndef HIVE_CONNECT_SPISTM_H
#define HIVE_CONNECT_SPISTM_H

#include "bsp/ISpiStm.h"

// TODO: implement this class as a tcp socket
class SpiStm : public ISpiStm {
  public:
    SpiStm() = default;
    ~SpiStm() override = default;

    bool send(const uint8_t* buffer, uint16_t length) override {
        (void)buffer;
        (void)length;
        return false;
    };

    bool isBusy() const override { return false; }
};

#endif // HIVE_CONNECT_SPISTM_H

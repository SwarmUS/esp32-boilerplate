#ifndef SPI_READ_H
#define SPI_READ_H

#include "hal/ihal.hpp"

class Spi
{
public:
    char read();

    Spi(IHal* hal){
        hal_ = hal;
    }

private:
    IHal* hal_;
};
#endif //SPI_READ_H
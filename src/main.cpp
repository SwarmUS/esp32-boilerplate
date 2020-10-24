#include "spi/spi.hpp"
#include "hal/ihal.hpp"
#include "hal/hal.hpp"

int main(int argc, char **argv) {
    IHal* hal = new Hal();

    Spi* spi = new Spi(hal);
    spi->read();

    delete (spi);
    delete(hal);
    
    return 0;
}
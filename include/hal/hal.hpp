#ifndef HAL_REGISTER_H
#define HAL_REGISTER_H

#include "ihal.hpp"

class Hal : public IHal
{
public:
    int readRegister(int address) override;
    
};

#endif //HAL_REGISTER_H
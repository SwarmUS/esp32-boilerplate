#include "spi.hpp"

char Spi::read(){
    return (char)this->hal_->readRegister(1);
}
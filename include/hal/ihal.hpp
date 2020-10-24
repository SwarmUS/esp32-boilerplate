#ifndef I_REGISTER_HPP
#define I_REGISTER_HPP

class IHal
{
public:
    virtual int readRegister(int address) = 0;

    virtual ~IHal() = default;
    
};

#endif //I_REGISTER_HPP
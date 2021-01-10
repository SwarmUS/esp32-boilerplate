#ifndef IBSP_H
#define IBSP_H

#include <stdint.h>

struct ChipInfo {
    uint8_t m_cores;
    enum OS_TYPE { ESP, SIMULATION } m_osType;
};
class IBSP {
  public:
    virtual ~IBSP() = default;

    /**
     * @brief Initialise chip for usage. Should be called within the main.
     */
    virtual void initChip() = 0;
    virtual ChipInfo getChipInfo() = 0;
};

#endif // IBSP_H

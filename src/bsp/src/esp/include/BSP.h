#ifndef BSP_H
#define BSP_H

#include "bsp/IBSP.h"
#include <array>

class BSP : public IBSP {
  public:
    BSP();
    ~BSP() override = default;

    void initChip() override;
    ChipInfo getChipInfo() override;
    uint16_t getUUID() override;
    void setUUID(uint16_t uuid) override;

  private:
    void initSPI();
    uint16_t m_UUID;
};

#endif // BSP_H

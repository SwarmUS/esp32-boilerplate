#ifndef BSP_H
#define BSP_H

#include "bsp/IBSP.h"
#include <array>

class BSP : public IBSP {
  public:
    BSP();
    ~BSP() override;

    void initChip() override;
    ChipInfo getChipInfo() override;

  private:
    void initSPI();
};

#endif // BSP_H

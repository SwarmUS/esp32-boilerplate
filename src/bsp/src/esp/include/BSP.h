#ifndef BSP_H
#define BSP_H

#include "bsp/IBSP.h"

class BSP : public IBSP {
  public:
    BSP(ILogger& logger);
    ~BSP() override;

    void initChip() override;
    ChipInfo getChipInfo() override;
    virtual const ILogger* getLogger() override;

  private:
    ILogger* m_logger;
};

#endif // BSP_H

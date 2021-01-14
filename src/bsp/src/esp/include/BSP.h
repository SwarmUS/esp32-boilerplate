#ifndef BSP_H
#define BSP_H

#include "bsp/IBSP.h"

class BSP : public IBSP {
  public:
    BSP(ILogger& logger);
    ~BSP() override;

    void initChip() override;
    ChipInfo getChipInfo() override;
    virtual void log(LogLevel level, const char* format, ...) override;

  private:
    ILogger* m_logger;
};

#endif // BSP_H

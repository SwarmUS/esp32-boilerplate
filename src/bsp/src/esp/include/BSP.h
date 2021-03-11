#ifndef BSP_H
#define BSP_H

#include "INetworkManager.h"
#include "bsp/IBSP.h"
#include <array>

class BSP : public IBSP {
  public:
    BSP(INetworkManager& networkManager);
    ~BSP() override = default;

    void initChip() override;
    ChipInfo getChipInfo() override;

  private:
    INetworkManager& m_networkManager;
    void initSPI();
};

#endif // BSP_H

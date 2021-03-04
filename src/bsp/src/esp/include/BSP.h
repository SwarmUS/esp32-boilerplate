#ifndef BSP_H
#define BSP_H

#include "NetworkManager.h"
#include "bsp/IBSP.h"
#include <array>

class BSP : public IBSP {
  public:
    BSP(NetworkManager& networkManager);
    ~BSP() override = default;

    void initChip() override;
    ChipInfo getChipInfo() override;

  private:
    NetworkManager m_networkManager;
    void initSPI();
};

#endif // BSP_H

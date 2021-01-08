#ifndef BSP_H
#define BSP_H

#include "bsp/IBSP.h"

class BSP : public IBSP {
public:
    BSP();
    ~BSP() override;

    void initChip() override;
};

#endif //BSP_H

#include "bsp/Container.h"
#include "BSP.h"

namespace BspContainer {

IBSP& getBSP() {
    static BSP s_bsp;

    return s_bsp;
}
} // namespace BspContainer
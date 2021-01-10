#include "bsp/Factory.h"
#include "BSP.h"

namespace BspFactory {

std::shared_ptr<IBSP> getBSP() {
    static BSP s_bsp;

    return std::make_shared<BSP>(s_bsp);
}
} // namespace BspFactory
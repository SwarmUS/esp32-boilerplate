#include "bsp/Factory.h"
#include "BSP.h"

namespace BspFactory {

    std::shared_ptr<IBSP> getBSP() {
        static BSP s_bsp;
        static bool s_isCreated = false;
        if(!s_isCreated) {
            new (&s_bsp) BSP();
        }


        return std::make_shared<BSP>(s_bsp);
    }
}
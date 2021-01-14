#include "bsp/Container.h"
#include "BSP.h"
#include "Logger.h"

namespace BspContainer {

IBSP& getBSP() {
    static BSP s_bsp(getLogger());

    return s_bsp;
}

ILogger& getLogger() {
    static Logger s_logger(INFO);

    return s_logger;
}
} // namespace BspContainer
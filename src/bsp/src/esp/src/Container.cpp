#include "bsp/Container.h"
#include "BSP.h"
#include "UserInterface.h"

namespace BspContainer {

IBSP& getBSP() {
    static BSP s_bsp;

    return s_bsp;
}

IUserInterface& getUserInterface() {
    static UserInterface s_ui;

    return s_ui;
}
} // namespace BspContainer
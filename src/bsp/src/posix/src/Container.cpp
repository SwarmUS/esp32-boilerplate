#include "bsp/Container.h"
#include "BSP.h"
#include "SpiStmMock.h"
#include "UserInterface.h"
#include "logger/LoggerContainer.h"

#define ROS_SPIN_LOOP_RATE_MS 200

namespace BspContainer {

IBSP& getBSP() {
    static ros::NodeHandle s_handle("~/");

    static BSP s_bsp(s_handle, ROS_SPIN_LOOP_RATE_MS);
    return s_bsp;
}

IUserInterface& getUserInterface() {
    static UserInterface s_ui;

    return s_ui;
}

ISpiStm& getSpiStm() {
    int port = 6868;
    static SpiStmMock s_spiStm(LoggerContainer::getLogger(), port);

    return s_spiStm;
}
} // namespace BspContainer
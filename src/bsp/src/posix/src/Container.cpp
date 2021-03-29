#include "bsp/Container.h"
#include "BSP.h"
#include "SpiStmMock.h"
#include "UserInterface.h"
#include "logger/LoggerContainer.h"

namespace BspContainer {

IBSP& getBSP() {
    static ros::NodeHandle s_handle("~/");

    static BSP s_bsp(s_handle);
    return s_bsp;
}

IUserInterface& getUserInterface() {
    static UserInterface s_ui;

    return s_ui;
}

ISpiStm& getSpiStm() {
    ros::NodeHandle nodeHandle("~/");
    int port = nodeHandle.param("spi_mock_port", 9001);
    std::string address = nodeHandle.param("spi_mock_address", std::string("127.0.0.1"));
    static SpiStmMock s_spiStm(LoggerContainer::getLogger(), address.c_str(), port);

    return s_spiStm;
}
} // namespace BspContainer
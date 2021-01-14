#include "bsp/Container.h"
#include "BSP.h"
#include "ROSLogger.h"

#define ROS_SPIN_LOOP_RATE_MS 200

namespace BspContainer {

IBSP& getBSP() {
    static ros::NodeHandle s_handle("~/");

    static BSP s_bsp(s_handle, ROS_SPIN_LOOP_RATE_MS, getLogger());
    return s_bsp;
}

ILogger& getLogger() {
    static LogLevel s_logLevel = INFO;
    static ROSLogger s_logger(s_logLevel);

    return s_logger;
}
} // namespace BspContainer
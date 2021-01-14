#include "bsp/Container.h"
#include "BSP.h"

#define ROS_SPIN_LOOP_RATE_MS 200

namespace BspContainer {

IBSP& getBSP() {
    static ros::NodeHandle s_handle("~/");

    static BSP s_bsp(s_handle, ROS_SPIN_LOOP_RATE_MS);
    return s_bsp;
}
} // namespace BspContainer
#include "bsp/Factory.h"
#include "BSP.h"

#define ROS_SPIN_LOOP_RATE_MS 200

namespace BspFactory {

    std::shared_ptr<IBSP> getBSP() {
        static BSP s_bsp;
        static bool s_isCreated = false;
        if(!s_isCreated) {
            static ros::NodeHandle s_handle("~/");
            new (&s_bsp) BSP(s_handle, ROS_SPIN_LOOP_RATE_MS);
        }
        return std::make_shared<BSP>(s_bsp);

    }
}
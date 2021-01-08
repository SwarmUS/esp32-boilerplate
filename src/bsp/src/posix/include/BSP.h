#ifndef BSP_H
#define BSP_H

#include "bsp/IBSP.h"
#include "ros/ros.h"

class BSP: public IBSP {
public:
    BSP();
    BSP(const ros::NodeHandle &nodeHandle, int loopRate);
    ~BSP() override;

    void initChip() override;

    std::shared_ptr<ros::NodeHandle> getNodeHandle();

private:
    std::shared_ptr<ros::NodeHandle> m_rosNodeHandle;
    int m_loopRate;


};

#endif //_BSP_H

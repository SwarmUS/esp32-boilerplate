#ifndef BSP_H
#define BSP_H

#include "bsp/IBSP.h"
#include "ros/ros.h"

class BSP : public IBSP {
  public:
    BSP(const ros::NodeHandle& nodeHandle, int loopRate, const ILogger& logger);
    ~BSP() override;

    void initChip() override;
    ChipInfo getChipInfo() override;
    const ILogger* getLogger() override;

    std::shared_ptr<ros::NodeHandle> getNodeHandle();

  private:
    std::shared_ptr<ros::NodeHandle> m_rosNodeHandle;
    const ILogger* m_logger;

    int m_loopRate;
};

#endif //_BSP_H

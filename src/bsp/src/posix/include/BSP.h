#ifndef BSP_H
#define BSP_H

#include "bsp/IBSP.h"
#include "ros/ros.h"

class BSP : public IBSP {
  public:
    BSP(const ros::NodeHandle& nodeHandle);
    ~BSP() override;

    void initChip() override;
    ChipInfo getChipInfo() override;
    uint16_t getHiveMindUUID() override;
    void setHiveMindUUID(uint16_t uuid) override;

    std::shared_ptr<ros::NodeHandle> getNodeHandle();

  private:
    ros::AsyncSpinner m_spinner;
    std::shared_ptr<ros::NodeHandle> m_rosNodeHandle;
    uint16_t m_UUID;
};

#endif //_BSP_H

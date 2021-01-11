#include "BSP.h"
#include "System.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>

void app_main() {}

class BspTests : public testing::Test {
  public:
    BSP* m_bsp;
    ros::NodeHandle* m_node;

    void SetUp() override {
        int argc = 0;
        ros::init(argc, nullptr, "test_bsp");
        m_node = new ros::NodeHandle("~/");
        m_bsp = new BSP(*m_node, 1000);
    }

    void TearDown() override { delete m_bsp; }
};

TEST_F(BspTests, ValidateChipInfo) {
    ChipInfo expectedChipInfo = {.m_cores = (uint8_t)std::thread::hardware_concurrency(),
                                 .m_osType = ChipInfo::SIMULATION};
    ASSERT_EQ(expectedChipInfo.m_cores, this->m_bsp->getChipInfo().m_cores);
    ASSERT_EQ(expectedChipInfo.m_osType, this->m_bsp->getChipInfo().m_osType);
}

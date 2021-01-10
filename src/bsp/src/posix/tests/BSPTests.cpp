#include "BSP.h"
#include <gtest/gtest.h>
#include <thread>

class BspTests : public testing::Test {
  public:
    BSP* m_bsp;

    void SetUp() override { m_bsp = new BSP(); }

    void TearDown() override { delete m_bsp; }
};

TEST_F(BspTests, ValidateChipInfo) {
    ChipInfo expectedChipInfo = {.m_cores = (uint8_t)std::thread::hardware_concurrency(),
                                 .m_osType = ChipInfo::SIMULATION};
    ASSERT_EQ(expectedChipInfo.m_cores, this->m_bsp->getChipInfo().m_cores);
    ASSERT_EQ(expectedChipInfo.m_osType, this->m_bsp->getChipInfo().m_osType);
}

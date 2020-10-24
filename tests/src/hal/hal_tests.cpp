#include <gtest/gtest.h>
#include "hal/hal.hpp"

class HalTestFixture : public testing::Test {

public:
    Hal hal;

protected:

    void SetUp() override { 
        hal = Hal();
    }

    void TearDown() override {
    }
};

TEST_F(HalTestFixture, Register_Read_Address1_Returns1) {
  ASSERT_EQ(hal.readRegister(1), 2);
}

TEST_F(HalTestFixture, Register_Read_AddressNeg_Returns1) {
  ASSERT_EQ(hal.readRegister(-1), 0);
}


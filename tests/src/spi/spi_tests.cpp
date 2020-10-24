#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "spi/spi.hpp"

using ::testing::Return;
using ::testing::Test;

class MockHal : public IHal {
 public:
  MOCK_METHOD(int, readRegister, (int address), (override));
};


class SpiTestFixture : public Test {

public:
    Spi spi = NULL;

protected:
    MockHal* hal_mock_ = NULL;

    void SetUp() override { 
        hal_mock_ = new MockHal();
        spi = Spi(hal_mock_);
    }

    void TearDown() override {
        delete(hal_mock_);
    }
};

TEST_F(SpiTestFixture, Read_Value_1) {
    EXPECT_CALL(*hal_mock_, readRegister(1))
    .Times(1)
    .WillOnce(Return(1));

    ASSERT_EQ(spi.read(), (char)1);
}

TEST_F(SpiTestFixture, Read_Value_65485_Overflow) {
    EXPECT_CALL(*hal_mock_, readRegister(1))
    .Times(1)
    .WillOnce(Return(0xffcd));

    ASSERT_EQ(spi.read(), (char)0xcd);
}

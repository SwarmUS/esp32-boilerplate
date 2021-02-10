#include "common/CRC.h"
#include <gtest/gtest.h>

class CRCTestsFixture : public testing::Test {};

TEST_F(CRCTestsFixture, TestCRC8_AllZeros) {
    uint8_t buff[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    EXPECT_NE(CRC::calculateCRC8(buff, sizeof(buff)), 0);
}

TEST_F(CRCTestsFixture, TestCRC8_SpecificHeader) {
    uint8_t buff[] = {0, 3, 4};
    EXPECT_EQ(CRC::calculateCRC8(buff, sizeof(buff)), 87);
}
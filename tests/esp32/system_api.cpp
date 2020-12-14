#include <gtest/gtest.h>
#include "esp_system.h"

void app_main() {} // work around for esp main function set-up


class Esp32StubTests : public testing::Test {};

TEST_F(Esp32StubTests, ValidateChipInfo)
{
    esp_chip_info_t chipInfo;
    esp_chip_info(&chipInfo);

    ASSERT_EQ(chipInfo.cores, 8);
}


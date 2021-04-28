#include "NetworkManager.h"
#include "cpp-common/HashMapStack.h"
#include "mocks/BSPMock.h"
#include "mocks/BroadcastMock.h"
#include "mocks/LoggerInterfaceMock.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

class NetworkManagerFixture : public testing::Test {
  public:
    void SetUp() override {
        m_hashMap = new HashMapStack<uint16_t, uint32_t, 10>();
        m_networkManager = new NetworkManager(m_logger, *m_hashMap, m_bsp, m_broadcast);
    }
    void TearDown() override {
        delete m_networkManager;
        delete m_hashMap;
    }

  protected:
    HashMapStack<uint16_t, uint32_t, 10>* m_hashMap;
    LoggerInterfaceMock m_logger;
    BSPMock m_bsp;
    BroadcastMock m_broadcast;
    NetworkManager* m_networkManager;
};

TEST_F(NetworkManagerFixture, register_and_get_ip_success) {
    // Given
    std::pair<uint16_t, uint32_t> agent(1, 42);

    // Then
    ASSERT_TRUE(m_networkManager->registerAgent(agent.first, agent.second));

    // Expect
    auto val = m_networkManager->getIPFromAgentID(agent.first);
    ASSERT_TRUE(val.has_value());
    ASSERT_EQ(val.value(), agent.second);
}

TEST_F(NetworkManagerFixture, get_ip_fail_not_present) {
    // Given
    std::pair<uint16_t, uint16_t> agent(1, 42);

    // Expect
    auto val = m_networkManager->getIPFromAgentID(agent.first);
    ASSERT_FALSE(val.has_value());
}

TEST_F(NetworkManagerFixture, get_status_noUUID) {
    EXPECT_CALL(m_bsp, getHiveMindUUID()).Times(1).WillOnce(testing::Return(0));

    ASSERT_EQ(m_networkManager->getNetworkStatus(), NetworkStatus::Connecting);
}

TEST_F(NetworkManagerFixture, get_statusUUID_broadcastNotStarted) {
    EXPECT_CALL(m_bsp, getHiveMindUUID()).Times(1).WillOnce(testing::Return(42));

    EXPECT_CALL(m_broadcast, isStarted())
        .Times(2)
        .WillOnce(testing::Return(false))
        .WillOnce(testing::Return(true));

    EXPECT_CALL(m_broadcast, start()).Times(1);

    ASSERT_EQ(m_networkManager->getNetworkStatus(), NetworkStatus::Connected);
}

TEST_F(NetworkManagerFixture, get_statusUUID_broadcastAlreadyStarted) {
    EXPECT_CALL(m_bsp, getHiveMindUUID()).Times(1).WillOnce(testing::Return(42));

    EXPECT_CALL(m_broadcast, isStarted())
        .Times(2)
        .WillOnce(testing::Return(true))
        .WillOnce(testing::Return(true));

    ASSERT_EQ(m_networkManager->getNetworkStatus(), NetworkStatus::Connected);
}

// Main for ros test

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "network_manager_rostest_node");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}
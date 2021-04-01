#include "NetworkManager.h"
#include "cpp-common/HashMap.h"
#include "mocks/LoggerInterfaceMock.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

class NetworkManagerFixture : public testing::Test {
  public:
    void SetUp() override {
        m_hashMap = new HashMap<uint16_t, uint32_t, gs_MAX_AGENT_IN_MAP>();
        m_networkManager = new NetworkManager(m_logger, *m_hashMap);
    }
    void TearDown() override {
        delete m_networkManager;
        delete m_hashMap;
    }

  protected:
    HashMap<uint16_t, uint32_t, gs_MAX_AGENT_IN_MAP>* m_hashMap;
    LoggerInterfaceMock m_logger;
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

TEST_F(NetworkManagerFixture, get_status) {
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
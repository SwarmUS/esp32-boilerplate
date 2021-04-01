#include "NetworkManager.h"
#include "cpp-common/HashMap.h"
#include "mocks/LoggerInterfaceMock.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

class NetworkManagerFixture : public testing::Test {
  public:
    void SetUp() override {
        m_hashMap = new HashMap<uint16_t, uint16_t, gs_MAX_AGENT_IN_MAP>();
        m_networkManager = new NetworkManager(m_logger, *m_hashMap);
    }
    void TearDown() override {
        delete m_networkManager;
        delete m_hashMap;
    }

  protected:
    HashMap<uint16_t, uint16_t, gs_MAX_AGENT_IN_MAP>* m_hashMap;
    LoggerInterfaceMock m_logger;
    NetworkManager* m_networkManager;
};

TEST_F(NetworkManagerFixture, register_and_get_ip_success) {
    // Given
    std::pair<uint16_t, uint16_t> agent(1, 42);

    // Then
    ASSERT_TRUE(m_networkManager->registerAgent(agent.first, std::to_string(agent.second).c_str()));
    char buffer[16];

    // Expect
    ASSERT_TRUE(m_networkManager->getIPFromAgentID(agent.first, buffer, sizeof(buffer)));
    ASSERT_EQ(atoi(buffer), agent.second);
}

TEST_F(NetworkManagerFixture, get_ip_fail_not_present) {
    // Given
    std::pair<uint16_t, uint16_t> agent(1, 42);

    // Expect
    char buffer[16];
    ASSERT_FALSE(m_networkManager->getIPFromAgentID(agent.first, buffer, sizeof(buffer)));
}

TEST_F(NetworkManagerFixture, get_ip_fail_null_buffer) {
    // Given
    std::pair<uint16_t, uint16_t> agent(1, 42);

    // Then
    ASSERT_TRUE(m_networkManager->registerAgent(agent.first, std::to_string(agent.second).c_str()));

    // Expect
    ASSERT_FALSE(m_networkManager->getIPFromAgentID(agent.first, nullptr, 0));
}

TEST_F(NetworkManagerFixture, register_ip_fail_null_buffer) {
    // Given
    std::pair<uint16_t, uint16_t> agent(1, 42);

    // Expect
    ASSERT_FALSE(m_networkManager->registerAgent(agent.first, nullptr));
}

TEST_F(NetworkManagerFixture, get_status) {
    ASSERT_EQ(m_networkManager->getNetworkStatus(), NetworkStatus::Connected);
}

TEST_F(NetworkManagerFixture, get_self_ip_success) {
    char buffer[64];
    ASSERT_TRUE(m_networkManager->getSelfIP(buffer, sizeof(buffer)));
    ASSERT_EQ(atoi(buffer), 42);
}

// Main for ros test

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "network_manager_rostest_node");

    ros::NodeHandle nodeHandle("~");
    int port = nodeHandle.param("tcp_listen_port", 54321);
    std::vector<std::string> params;
    nodeHandle.getParamNames(params);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}
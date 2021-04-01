#include "../src/broker/config/TopicDefines.h"
#include "NetworkBroadcast.h"
#include "mocks/BSPMock.h"
#include "mocks/LoggerInterfaceMock.h"
#include <chrono>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>

class BroadCastTestFixture : public testing::Test {
  public:
    void SetUp() override {
        m_sub = ros::NodeHandle().subscribe(std::string(BROADCAST_OUTPUT_TOPIC) + "1", 1000,
                                            &BroadCastTestFixture::handleStuff, this);
        createRobot(m_robot1, 1);
        createRobot(m_robot2, 2);
        createRobot(m_robot3, 3);

        m_robot1.m_broadcastInstance->start();
        m_robot2.m_broadcastInstance->start();
        m_robot3.m_broadcastInstance->start();
    }

    void TearDown() override {
        delete m_robot1.m_broadcastInstance;
        delete m_robot1.m_bspMock;

        delete m_robot2.m_broadcastInstance;
        delete m_robot2.m_bspMock;

        delete m_robot3.m_broadcastInstance;
        delete m_robot3.m_bspMock;
    }

    struct RobotAssembly {
        testing::NiceMock<BSPMock>* m_bspMock;
        INetworkBroadcast* m_broadcastInstance;
    };

  protected:
    RobotAssembly m_robot1, m_robot2, m_robot3;
    LoggerInterfaceMock m_logger;

    void createRobot(RobotAssembly& robot, uint16_t uuid) {
        robot.m_bspMock = new testing::NiceMock<BSPMock>(uuid);
        std::string outputTopic(BROADCAST_OUTPUT_TOPIC);
        std::string inputTopic(BROADCAST_INPUT_TOPIC);
        robot.m_broadcastInstance = new NetworkBroadcast(m_logger, *robot.m_bspMock,
                                                         outputTopic.c_str(), inputTopic.c_str());
    }
};

TEST_F(BroadCastTestFixture, test_robot_creation) {
    ASSERT_EQ(m_robot1.m_bspMock->getHiveMindUUID(), 1);
    ASSERT_EQ(m_robot2.m_bspMock->getHiveMindUUID(), 2);
    ASSERT_EQ(m_robot3.m_bspMock->getHiveMindUUID(), 3);
}

TEST_F(BroadCastTestFixture, test_broadcast) {
    char message[] = "This message should be broadcasted";

    // Send message
    m_robot1.m_broadcastInstance->send((uint8_t*)message, sizeof(message));
    char charRobot2 = 0;
    char charRobot3 = 0;

    for (int i = 0; i < strlen(message); i++) {
        ASSERT_TRUE(m_robot2.m_broadcastInstance->receive((uint8_t*)&charRobot2, 1));
        ASSERT_TRUE(m_robot3.m_broadcastInstance->receive((uint8_t*)&charRobot3, 1));

        ASSERT_EQ(charRobot2, message[i]);
        ASSERT_EQ(charRobot3, message[i]);
    }
}

// Main for tests
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "broadcast_test_node");

    ros::AsyncSpinner spinner(4);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}
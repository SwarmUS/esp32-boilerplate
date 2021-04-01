#include "NetworkBroadcast.h"
#include "mocks/BSPMock.h"
#include "mocks/LoggerInterfaceMock.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

class BroadCastTestFixture : public testing::Test {
  public:
    void SetUp() override {
        createRobot(m_robot1, 1);
        createRobot(m_robot2, 2);
        createRobot(m_robot3, 3);
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
        IBSP* m_bspMock;
        INetworkBroadcast* m_broadcastInstance;
    };

  protected:
    RobotAssembly m_robot1, m_robot2, m_robot3;
    LoggerInterfaceMock m_logger;

    void createRobot(RobotAssembly& robot, uint16_t uuid) {
        robot.m_bspMock = new BSPMock(uuid);
        std::string outputTopic("/Communication/broadcastOutput/" +
                                std::to_string(robot.m_bspMock->getHiveMindUUID()));
        std::string inputTopic("/Communication/broadcastInput/" +
                               std::to_string(robot.m_bspMock->getHiveMindUUID()));
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
    ASSERT_TRUE(m_robot1.m_broadcastInstance->send((uint8_t*)message, sizeof(message)));
}

// Main for tests
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "broadcast_test_node");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}
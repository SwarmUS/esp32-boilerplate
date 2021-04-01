#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(TestSuite, example) { ASSERT_TRUE(true); }
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
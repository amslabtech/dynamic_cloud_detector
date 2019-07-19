#include <gtest/gtest.h>

#include <ros/ros.h>

#include "dynamic_cloud_detector/dynamic_cloud_detector.h"

TEST(TestSuite, test0)
{
    EXPECT_NEAR(1.0, 1.0, 0.01);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "dynamic_cloud_detector_test");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Duration(3.0).sleep();

    int r_e_t = RUN_ALL_TESTS();

    spinner.stop();

    ros::shutdown();

    return r_e_t;
}

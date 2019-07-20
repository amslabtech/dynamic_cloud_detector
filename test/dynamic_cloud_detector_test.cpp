#include <gtest/gtest.h>

#include <ros/ros.h>

#include "dynamic_cloud_detector/dynamic_cloud_detector.h"

TEST(TestSuite, test0)
{
    DynamicCloudDetector dcd;
    const double X = 5;
    const double Y = 5;
    int i = dcd.get_index_from_xy(X, Y);
    EXPECT_EQ(i, 30150);
    double x = dcd.get_x_from_index(i);
    EXPECT_EQ(x, X);
    double y = dcd.get_y_from_index(i);
    EXPECT_EQ(y, Y);
}

TEST(TestSuite, test1)
{
    DynamicCloudDetector dcd;
    DynamicCloudDetector::CloudXYZIPtr cloud(new DynamicCloudDetector::CloudXYZI);
    DynamicCloudDetector::PointXYZI point;
    point.x = 1;
    point.y = 1;
    cloud->points.push_back(point);
    dcd.input_cloud_to_grid_cells(cloud);
    Eigen::Vector3d t(0.1, 0.1, 0);
    dcd.move_grid_cells(0.1, t);
    point.x = 0.995171;
    point.y = 1.19484;
    cloud->points.push_back(point);
    DynamicCloudDetector::CloudXYZIPtr d_cloud(new DynamicCloudDetector::CloudXYZI);
    DynamicCloudDetector::CloudXYZIPtr s_cloud(new DynamicCloudDetector::CloudXYZI);
    dcd.devide_cloud(cloud, d_cloud, s_cloud);
    std::cout << "dynamic" << std::endl;
    for(auto pt : d_cloud->points){
        std::cout << pt << std::endl;
    }
    std::cout << "static" << std::endl;
    for(auto pt : s_cloud->points){
        std::cout << pt << std::endl;
    }
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

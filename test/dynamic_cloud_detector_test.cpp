// Copyright 2023 amsl

#include <gtest/gtest.h>

#include <ros/ros.h>

#include "dynamic_cloud_detector/dynamic_cloud_detector.h"

TEST(TestSuite, test0)
{
  DynamicCloudDetector dcd;
  const double X = 5;
  const double Y = 5;
  int i = dcd.get_index_from_xy(X, Y);
  EXPECT_EQ(i, 25125);
  double x = dcd.get_x_from_index(i);
  EXPECT_EQ(x, X);
  double y = dcd.get_y_from_index(i);
  EXPECT_EQ(y, Y);
  int _i = dcd.get_index_from_xy(x, y);
  EXPECT_EQ(_i, 25125);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "dynamic_cloud_detector_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Duration(1.0).sleep();

  int r_e_t = RUN_ALL_TESTS();

  spinner.stop();

  ros::shutdown();

  return r_e_t;
}

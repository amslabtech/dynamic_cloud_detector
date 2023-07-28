// Copyright 2023 amsl

#include "dynamic_cloud_detector/dynamic_cloud_detector.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamic_cloud_detector");
  DynamicCloudDetector dynamic_cloud_detector;
  dynamic_cloud_detector.process();
  return 0;
}

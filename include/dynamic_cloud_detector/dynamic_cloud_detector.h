#ifndef __DYNAMIC_CLOUD_DETECTOR_H
#define __DYNAMIC_CLOUD_DETECTOR_H

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/tf.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// PCL
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>

// OMP
#include <omp.h>

class DynamicCloudDetector
{
public:
    typedef pcl::PointXYZI PointXYZI;
    typedef pcl::PointCloud<PointXYZI> CloudXYZI;
    typedef pcl::PointCloud<PointXYZI>::Ptr CloudXYZIPtr;

    class GridCell
    {
    public:
        GridCell(void)
        {
            clear_count = 0;
            occupied_count = 0;
            state = -1;
        }
        int clear_count;
        int occupied_count;
        int state;// -1: unknown, 0: clear, 100: occupied
    private:
    };

    DynamicCloudDetector(void);

    void callback(const sensor_msgs::PointCloud2ConstPtr&, const nav_msgs::OdometryConstPtr&);
    void input_cloud_to_grid_cells(const CloudXYZIPtr&);
    void move_grid_cells(const double, const Eigen::Vector3d&);
    void devide_cloud(const CloudXYZIPtr&, CloudXYZIPtr&, CloudXYZIPtr&);
    int get_index_from_xy(const double, const double);
    int get_x_index_from_index(const int);
    int get_y_index_from_index(const int);
    double get_x_from_index(const int);
    double get_y_from_index(const int);
    void process(void);

private:
    double RESOLUTION;
    double WIDTH;
    double WIDTH_2;
    int GRID_WIDTH;
    int GRID_WIDTH_2;
    int GRID_NUM;
    double OCCUPANCY_THRESHOLD;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    ros::Publisher dynamic_pub;
    ros::Publisher static_pub;
    ros::Publisher grid_pub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> sync_subs;
    message_filters::Subscriber<sensor_msgs::PointCloud2> obstacles_cloud_sub;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
    message_filters::Synchronizer<sync_subs> sync;

    std::vector<GridCell> grid_cells;
    bool first_flag;
};

#endif// __DYNAMIC_CLOUD_DETECTOR_H

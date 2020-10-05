#ifndef __DYNAMIC_CLOUD_DETECTOR_H
#define __DYNAMIC_CLOUD_DETECTOR_H

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

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
    typedef pcl::PointXYZINormal PointXYZIN;
    typedef pcl::PointCloud<PointXYZIN> CloudXYZIN;
    typedef pcl::PointCloud<PointXYZIN>::Ptr CloudXYZINPtr;

    class GridCell
    {
    public:
        GridCell(void);
        double get_occupancy(void);
        double get_log_odds(void);
        void add_log_odds(double);

        double log_odds;
    private:
    };
    typedef std::vector<GridCell> OccupancyGridMap;

    DynamicCloudDetector(void);

    void callback(const sensor_msgs::PointCloud2ConstPtr&, const nav_msgs::OdometryConstPtr&);
    void input_cloud_to_occupancy_grid_map(const CloudXYZINPtr&);
    void devide_cloud(const CloudXYZINPtr&, CloudXYZINPtr&, CloudXYZINPtr&);
    int get_index_from_xy(const double, const double);
    int get_x_index_from_index(const int);
    int get_y_index_from_index(const int);
    double get_x_from_index(const int);
    double get_y_from_index(const int);
    void publish_occupancy_grid_map(const ros::Time&, const std::string&);
    std::string remove_first_slash(std::string);
    bool is_valid_point(double, double);
    void transform_occupancy_grid_map(const Eigen::Vector2d&, double, OccupancyGridMap&);
    void set_clear_grid_cells(const std::vector<double>&, const std::vector<bool>&, OccupancyGridMap&);
    void process(void);

private:
    double resolution_;
    double width_;
    double width_2_;
    int grid_width_;
    int grid_width_2_;
    int grid_num_;
    double occupancy_threshold_;
    int beam_num_;

    ros::NodeHandle nh_;
    ros::NodeHandle local_nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener listener_;

    ros::Publisher dynamic_pub_;
    ros::Publisher static_pub_;
    ros::Publisher grid_pub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> sync_subs;
    message_filters::Subscriber<sensor_msgs::PointCloud2> obstacles_cloud_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Synchronizer<sync_subs> sync_;

    OccupancyGridMap occupancy_grid_map_;
};

#endif// __DYNAMIC_CLOUD_DETECTOR_H

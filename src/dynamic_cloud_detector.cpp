// Copyright 2023 amsl

#include <algorithm>
#include <string>
#include <vector>

#include "dynamic_cloud_detector/dynamic_cloud_detector.h"

DynamicCloudDetector::DynamicCloudDetector(void)
  : local_nh_("~")
  , listener_(tf_buffer_)
  , obstacles_cloud_sub_(nh_, "/velodyne_obstacles", 10)
  , odom_sub_(nh_, "/odom/complement", 10)
  , sync_(sync_subs(10), obstacles_cloud_sub_, odom_sub_)
{
  local_nh_.param("resolution", resolution_, {0.2});
  local_nh_.param("width", width_, {40.0});
  local_nh_.param("occupancy_threshold", occupancy_threshold_, {0.2});
  local_nh_.param("beam_num", beam_num_, {720});
  local_nh_.param("log_odds_increase", log_odds_increase_, {0.4});
  local_nh_.param("log_odds_decrease", log_odds_decrease_, {0.2});

  grid_width_ = width_ / resolution_;
  grid_num_ = grid_width_ * grid_width_;
  width_2_ = width_ / 2.0;
  grid_width_2_ = grid_width_ / 2.0;

  dynamic_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud/dynamic", 1);
  static_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud/static", 1);
  grid_pub_ = local_nh_.advertise<nav_msgs::OccupancyGrid>("occupancy_grid", 1);

  sync_.registerCallback(boost::bind(&DynamicCloudDetector::callback, this, _1, _2));

  occupancy_grid_map_.resize(grid_num_);

  std::cout << "=== dynamic cloud detector ===" << std::endl;
  std::cout << "resolution: " << resolution_ << std::endl;
  std::cout << "width: " << width_ << std::endl;
  std::cout << "width_2: " << width_2_ << std::endl;
  std::cout << "grid_num: " << grid_num_ << std::endl;
  std::cout << "occupancy_threshold: " << occupancy_threshold_ << std::endl;
  std::cout << "log_odds_increase: " << log_odds_increase_ << std::endl;
  std::cout << "log_odds_decrease: " << log_odds_decrease_ << std::endl;
}

void DynamicCloudDetector::callback(
    const sensor_msgs::PointCloud2ConstPtr& msg_obstacles_cloud,
    const nav_msgs::OdometryConstPtr& msg_odom)
{
  // const double start_time = ros::Time::now().toSec();
  static Eigen::Vector2d last_odom_position(
      msg_odom->pose.pose.position.x,
      msg_odom->pose.pose.position.y);
  static double last_yaw = tf2::getYaw(msg_odom->pose.pose.orientation);

  std::cout << "--- callback ---" << std::endl;
  CloudXYZINPtr cloud_ptr(new CloudXYZIN);
  pcl::fromROSMsg(*msg_obstacles_cloud, *cloud_ptr);

  std::cout << "received cloud size: " << cloud_ptr->points.size() << std::endl;

  // transform pointcloud to base frame
  const std::string sensor_frame_id = remove_first_slash(msg_obstacles_cloud->header.frame_id);
  const std::string base_frame_id = remove_first_slash(msg_odom->child_frame_id);
  try
  {
    geometry_msgs::TransformStamped transform;
    transform = tf_buffer_.lookupTransform(base_frame_id, sensor_frame_id, ros::Time(0));
    const Eigen::Matrix4d mat = tf2::transformToEigen(transform.transform).matrix().cast<double>();
    pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, mat);
    cloud_ptr->header.frame_id = base_frame_id;
  }
  catch (tf2::TransformException& ex)
  {
    std::cout << ex.what() << std::endl;
    return;
  }

  // transform occupancy grid map
  const Eigen::Vector2d odom_position(msg_odom->pose.pose.position.x, msg_odom->pose.pose.position.y);
  const double yaw = tf2::getYaw(msg_odom->pose.pose.orientation);
  const Eigen::Vector2d diff_odom =
      Eigen::Rotation2Dd(-last_yaw).toRotationMatrix() * (odom_position - last_odom_position);
  double diff_yaw = yaw - last_yaw;
  diff_yaw = atan2(sin(diff_yaw), cos(diff_yaw));
  std::cout << "diff odom: " << diff_odom.transpose() << std::endl;
  std::cout << "diff yaw: " << diff_yaw << std::endl;

  transform_occupancy_grid_map(-diff_odom, -diff_yaw, occupancy_grid_map_);

  input_cloud_to_occupancy_grid_map(cloud_ptr);

  publish_occupancy_grid_map(msg_odom->header.stamp, base_frame_id);

  CloudXYZINPtr dynamic_cloud_ptr(new CloudXYZIN);
  dynamic_cloud_ptr->header = cloud_ptr->header;
  CloudXYZINPtr static_cloud_ptr(new CloudXYZIN);
  static_cloud_ptr->header = cloud_ptr->header;
  devide_cloud(cloud_ptr, dynamic_cloud_ptr, static_cloud_ptr);

  dynamic_pub_.publish(dynamic_cloud_ptr);
  static_pub_.publish(static_cloud_ptr);

  last_odom_position = odom_position;
  last_yaw = yaw;
  // std::cout << "time: " << ros::Time::now().toSec() - start_time << "[s]" << std::endl;
}

void DynamicCloudDetector::input_cloud_to_occupancy_grid_map(const CloudXYZINPtr& cloud_ptr)
{
  std::cout << "--- input cloud to occupancy grid map ---" << std::endl;
  std::vector<double> beam_list(beam_num_, sqrt(2) * width_2_);
  const double beam_angle_resolution = 2.0 * M_PI / static_cast<double>(beam_num_);

  // occupancy_grid_map_.clear();
  // occupancy_grid_map_.resize(grid_num_);
  const int cloud_size = cloud_ptr->points.size();
  std::vector<bool> obstacle_indices(grid_num_, false);
  for (int i = 0; i < cloud_size; i++)
  {
    const auto& p = cloud_ptr->points[i];
    if (!is_valid_point(p.x, p.y))
    {
      continue;
    }
    // occupancy_grid_map_[get_index_from_xy(p.x, p.y)].add_log_odds(0.01);
    const double distance = sqrt(p.x * p.x + p.y * p.y);
    const double direction = atan2(p.y, p.x);
    const int beam_index = (direction + M_PI) / beam_angle_resolution;
    if (0 <= beam_index && beam_index < beam_num_)
    {
      beam_list[beam_index] = std::min(beam_list[beam_index], distance);
    }
    const int index = get_index_from_xy(p.x, p.y);
    if (index < 0 || grid_num_ <= index)
    {
      continue;
    }
    obstacle_indices[get_index_from_xy(p.x, p.y)] = true;
  }

  for (int i = 0; i < grid_num_; i++)
  {
    if (obstacle_indices[i])
    {
      occupancy_grid_map_[i].add_log_odds(log_odds_increase_);
    }
  }

  set_clear_grid_cells(beam_list, obstacle_indices, occupancy_grid_map_);
}

void DynamicCloudDetector::devide_cloud(
    const CloudXYZINPtr& cloud,
    CloudXYZINPtr& dynamic_cloud,
    CloudXYZINPtr& static_cloud)
{
  dynamic_cloud->points.clear();
  static_cloud->points.clear();
  for (const auto& pt : cloud->points)
  {
    if (-width_2_ <= pt.x && pt.x <= width_2_ && -width_2_ <= pt.y && pt.y <= width_2_)
    {
      const int index = get_index_from_xy(pt.x, pt.y);
      if (0 <= index && index < grid_num_)
      {
        const double occupancy = occupancy_grid_map_[index].get_occupancy();
        if (occupancy < occupancy_threshold_)
        {
          dynamic_cloud->points.push_back(pt);
        }
        else
        {
          static_cloud->points.push_back(pt);
        }
      }
    }
  }
}

double DynamicCloudDetector::get_x_from_index(const int index)
{
  return (get_x_index_from_index(index) - grid_width_2_) * resolution_;
}

double DynamicCloudDetector::get_y_from_index(const int index)
{
  return (get_y_index_from_index(index) - grid_width_2_) * resolution_;
}

void DynamicCloudDetector::publish_occupancy_grid_map(const ros::Time& stamp, const std::string& frame_id)
{
  std::cout << "--- publish occupancy grid map ---" << std::endl;
  nav_msgs::OccupancyGrid og;
  og.header.stamp = stamp;
  og.header.frame_id = frame_id;
  og.info.resolution = resolution_;
  og.info.width = grid_width_;
  og.info.height = grid_width_;
  og.info.origin.position.x = -width_2_;
  og.info.origin.position.y = -width_2_;
  og.info.origin.orientation.w = 1.0;
  og.data.resize(grid_num_);
  for (int i = 0; i < grid_num_; i++)
  {
    og.data[i] = occupancy_grid_map_[i].get_occupancy() * 100;
  }
  grid_pub_.publish(og);
}

std::string DynamicCloudDetector::remove_first_slash(std::string frame_id)
{
  const int slash_pos = frame_id.find('/');
  if (slash_pos == 0)
  {
    frame_id.erase(0, 1);
  }
  return frame_id;
}

void DynamicCloudDetector::transform_occupancy_grid_map(
    const Eigen::Vector2d& translation,
    double diff_yaw,
    OccupancyGridMap& map)
{
  const double dx = translation(0);
  const double dy = translation(1);
  const double c_yaw = cos(diff_yaw);
  const double s_yaw = sin(diff_yaw);

  std::cout
      << "scrolling\n\tdx: "
      << dx
      << ", dy: "
      << dy
      << ", cos(theta): "
      << c_yaw
      << ", sin(theta)"
      << s_yaw
      << std::endl;

  const double dx_grid = dx / resolution_;
  const double dy_grid = dy / resolution_;
  std::cout << "dx_grid: " << dx_grid << std::endl;
  std::cout << "dy_grid: " << dy_grid << std::endl;

  Eigen::Matrix3d affine;
  affine << c_yaw, -s_yaw, dx_grid,
      s_yaw, c_yaw, dy_grid,
      0, 0, 1;
  std::cout << "forward affine:\n"
            << affine << std::endl;
  const Eigen::Matrix3d affine_inverse = affine.inverse();
  std::cout << "reversed affine:\n"
            << affine_inverse << std::endl;

  OccupancyGridMap ogm(grid_num_);
  // const int show_i = grid_num_ * 0.5 + grid_width_ - 1;
  for (int i = 0; i < grid_num_; i++)
  {
    // if(i == show_i)
    //     std::cout << "i: " << i << std::endl;
    const double x_i = get_x_index_from_index(i) - grid_width_2_;
    const double y_i = get_y_index_from_index(i) - grid_width_2_;
    Eigen::Vector3d ogm_i(x_i, y_i, 1);
    // if(i == show_i)
    //     std::cout << "ogm_i.transpose(): " << ogm_i.transpose() << std::endl;
    // if(i == show_i)
    //     std::cout << x_i * resolution_ << ", " << y_i * resolution_ << std::endl;
    Eigen::Vector3d map_i = affine_inverse * ogm_i;
    // if(i == show_i)
    //     std::cout << "map_i.transpose(): " << map_i.transpose() << std::endl;
    // if(i == show_i)
    //     std::cout << map_i(0) * resolution_ << ", " << map_i(1) * resolution_ << std::endl;

    // bilinear interpolation
    const int x_0 = std::floor(map_i(0));
    const int x_1 = x_0 + 1;
    const int y_0 = std::floor(map_i(1));
    const int y_1 = y_0 + 1;
    // if(i == show_i)
    //     std::cout << x_0 << ", " << x_1 << ", " << y_0 << ", " << y_1 << std::endl;
    if (x_0 < -grid_width_2_ || grid_width_2_ <= x_1)
    {
      continue;
    }
    if (y_0 < -grid_width_2_ || grid_width_2_ <= y_1)
    {
      continue;
    }
    const int index_0_0 = (y_0 + grid_width_2_) * grid_width_ + x_0 + grid_width_2_;
    const int index_0_1 = (y_1 + grid_width_2_) * grid_width_ + x_0 + grid_width_2_;
    const int index_1_0 = (y_0 + grid_width_2_) * grid_width_ + x_1 + grid_width_2_;
    const int index_1_1 = (y_1 + grid_width_2_) * grid_width_ + x_1 + grid_width_2_;
    // if(i == show_i)
    //     std::cout << index_0_0 << ", " << index_0_1 << ", " << index_1_0 << ", " << index_1_1 << std::endl;

    const Eigen::Vector2d y_vec(y_1 - map_i(1), map_i(1) - y_0);
    const Eigen::Vector2d x_vec(x_1 - map_i(0), map_i(0) - x_0);
    Eigen::Matrix2d value_mat;
    // value_mat << map[index_0_0].get_log_odds(), map[index_1_0].get_log_odds(),
    //              map[index_0_1].get_log_odds(), map[index_1_1].get_log_odds();
    value_mat << map[index_0_0].get_occupancy(), map[index_1_0].get_occupancy(),
        map[index_0_1].get_occupancy(), map[index_1_1].get_occupancy();

    const double ogm_value = y_vec.transpose() * value_mat * x_vec;
    ogm[i].log_odds = std::log(ogm_value / (1 - ogm_value));
    // if(i == show_i)
    //     std::cout << "\033[31m" << ogm_value << "\033[0m" << std::endl;
  }
  map.clear();
  map = ogm;
}

void DynamicCloudDetector::set_clear_grid_cells(
    const std::vector<double>& beam_list,
    const std::vector<bool>& obstacle_indices,
    OccupancyGridMap& map)
{
  std::vector<bool> clear_indices(grid_num_, false);
  const double beam_angle_resolution = 2.0 * M_PI / static_cast<double>(beam_num_);
  for (int i = 0; i < beam_num_; i++)
  {
    double direction = i * beam_angle_resolution - M_PI;
    direction = atan2(sin(direction), cos(direction));
    // std::cout << i << ": " << direction << ", " << beam_list[i] << std::endl;
    const double c = cos(direction);
    const double s = sin(direction);
    for (double range = 0.0; range < beam_list[i]; range += resolution_)
    {
      const double x = range * c;
      const double y = range * s;
      if (is_valid_point(x, y))
      {
        const int index = get_index_from_xy(x, y);
        if (!obstacle_indices[index])
        {
          clear_indices[index] = true;
        }
        else
        {
          break;
        }
      }
      else
      {
        break;
      }
    }
  }
  for (int i = 0; i < grid_num_; ++i)
  {
    if (clear_indices[i])
    {
      map[i].add_log_odds(-log_odds_decrease_);
    }
  }
}

void DynamicCloudDetector::process(void)
{
  ros::spin();
}

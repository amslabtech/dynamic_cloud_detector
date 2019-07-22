#include "dynamic_cloud_detector/dynamic_cloud_detector.h"

DynamicCloudDetector::DynamicCloudDetector(void)
:local_nh("~"), obstacles_cloud_sub(nh, "/velodyne_obstacles", 10), odom_sub(nh, "/odom/complement", 10)
, sync(sync_subs(10), obstacles_cloud_sub, odom_sub)
{
    local_nh.param("RESOLUTION", RESOLUTION, {0.1});
    local_nh.param("WIDTH", WIDTH, {20.0});
    local_nh.param("OCCUPANCY_THRESHOLD", OCCUPANCY_THRESHOLD, {0.2});
    local_nh.param("BEAM_NUM", BEAM_NUM, {720});
    GRID_WIDTH = WIDTH / RESOLUTION;
    GRID_NUM = GRID_WIDTH * GRID_WIDTH;
    WIDTH_2 = WIDTH / 2.0;
    GRID_WIDTH_2 = GRID_WIDTH / 2.0;

    grid_cells.resize(GRID_NUM);
    first_flag = true;

    dynamic_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud/dynamic", 1);
    static_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud/static", 1);
    grid_pub = local_nh.advertise<nav_msgs::OccupancyGrid>("occupancy_grid", 1);

    sync.registerCallback(boost::bind(&DynamicCloudDetector::callback, this, _1, _2));

    std::cout << "=== dynamic cloud detector ===" << std::endl;
    std::cout << "RESOLUTION: " << RESOLUTION << std::endl;
    std::cout << "WIDTH: " << WIDTH << std::endl;
    std::cout << "OCCUPANCY_THRESHOLD: " << OCCUPANCY_THRESHOLD << std::endl;
    std::cout << "BEAM_NUM: " << BEAM_NUM << std::endl;
}

void DynamicCloudDetector::callback(const sensor_msgs::PointCloud2ConstPtr& msg_obstacles_cloud, const nav_msgs::OdometryConstPtr& msg_odom)
{
    std::cout << "=== dynamic cloud detector ===" << std::endl;

    double start = ros::Time::now().toSec();
    Eigen::Vector3d current_position(msg_odom->pose.pose.position.x, msg_odom->pose.pose.position.y, msg_odom->pose.pose.position.z);
    double current_yaw = tf::getYaw(msg_odom->pose.pose.orientation);
    std::cout << "current_position: " << current_position.transpose() << std::endl;
    std::cout << "current_yaw: " << current_yaw << std::endl;
    static Eigen::Vector3d last_position;
    static double last_yaw;
    std::cout << "last_position: " << last_position.transpose() << std::endl;
    std::cout << "last_yaw: " << last_yaw << std::endl;
    CloudXYZIPtr obstacles_cloud(new CloudXYZI);
    pcl::fromROSMsg(*msg_obstacles_cloud, *obstacles_cloud);
    int cloud_size = obstacles_cloud->points.size();
    std::cout << "cloud size: " <<  cloud_size << std::endl;

    if(!first_flag){
        double d_yaw = current_yaw - last_yaw;
        d_yaw = atan2(sin(d_yaw), cos(d_yaw));

        Eigen::Matrix3d last_yaw_rotation;
        last_yaw_rotation = Eigen::AngleAxisd(-last_yaw, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d _current_position = last_yaw_rotation * current_position;
        Eigen::Vector3d _last_position = last_yaw_rotation * last_position;
        Eigen::Vector3d dp = _current_position - _last_position;
        std::cout << "dp: " << dp.transpose() << std::endl;
        std::cout << "dyaw: " << d_yaw << "[rad]" << std::endl;

        std::cout << "move grid cells" << std::endl;
        move_grid_cells(-d_yaw, -dp);

        std::cout << "cloud to grid cells" << std::endl;
        input_cloud_to_grid_cells(obstacles_cloud);

        CloudXYZIPtr dynamic_cloud(new CloudXYZI);
        dynamic_cloud->header = obstacles_cloud->header;
        CloudXYZIPtr static_cloud(new CloudXYZI);
        static_cloud->header = obstacles_cloud->header;

        devide_cloud(obstacles_cloud, dynamic_cloud, static_cloud);

        std::cout << "dynamic cloud size: " << dynamic_cloud->points.size() << std::endl;
        sensor_msgs::PointCloud2 _dynamic_cloud;
        pcl::toROSMsg(*dynamic_cloud, _dynamic_cloud);
        dynamic_pub.publish(_dynamic_cloud);

        std::cout << "static cloud size: " << static_cloud->points.size() << std::endl;
        sensor_msgs::PointCloud2 _static_cloud;
        pcl::toROSMsg(*static_cloud, _static_cloud);
        static_pub.publish(_static_cloud);

        nav_msgs::OccupancyGrid grid;
        grid.header = pcl_conversions::fromPCL(obstacles_cloud->header);
        grid.info.resolution = RESOLUTION;
        grid.info.width = GRID_WIDTH;
        grid.info.height = GRID_WIDTH;
        grid.info.origin.position.x = -WIDTH * 0.5;
        grid.info.origin.position.y = -WIDTH * 0.5;
        grid.info.origin.orientation = tf::createQuaternionMsgFromYaw(0);
        for(auto gc : grid_cells){
            // grid.data.push_back(gc.state);
            if(gc.occupied_count + gc.clear_count > 0){
                grid.data.push_back(gc.get_occupancy() * 255);
            }else{
                grid.data.push_back(UNKNOWN);
            }
        }
        grid_pub.publish(grid);
    }else{
        first_flag = false;
        std::cout << "cloud to grid cells" << std::endl;
        input_cloud_to_grid_cells(obstacles_cloud);
        last_position = current_position;
        last_yaw = current_yaw;
    }
    last_position = current_position;
    last_yaw = current_yaw;

    std::cout << "time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
}

void DynamicCloudDetector::input_cloud_to_grid_cells(const CloudXYZIPtr& cloud)
{
    std::vector<GridCell> _grid_cells(grid_cells);

    // search occupied grid cells
    int count = 0;
    for(const auto& pt : cloud->points){
        if(-WIDTH_2 <= pt.x && pt.x <= WIDTH_2 && -WIDTH_2 <= pt.y && pt.y <= WIDTH_2){
            int index = get_index_from_xy(pt.x, pt.y);
            if(0 <= index && index < GRID_NUM){
                _grid_cells[index].state = OCCUPIED;
                count++;
            }
        }
    }
    std::cout << count << " obstacles was added" << std::endl;

    // search clear grid cells
    std::vector<double> beam_list(BEAM_NUM, WIDTH);
    const double BEAM_ANGLE_RESOLUTION = 2.0 * M_PI / (double)BEAM_NUM;
    for(const auto& pt: cloud->points){
        double distance = sqrt(pt.x * pt.x + pt.y * pt.y);
        if(-WIDTH_2 <= pt.x && pt.x <= WIDTH_2 && -WIDTH_2 <= pt.y && pt.y <= WIDTH_2){
            double angle = atan2(pt.y, pt.x);
            int beam_index = (angle + M_PI) / BEAM_ANGLE_RESOLUTION;
            if(0 <= beam_index && beam_index < BEAM_NUM){
                if(beam_list[beam_index] > distance){
                    beam_list[beam_index] = distance;
                }
            }
        }
    }
    std::cout << "set clear cells" << std::endl;
    for(int i=0;i<BEAM_NUM;i++){
        double angle = i * BEAM_ANGLE_RESOLUTION - M_PI;
        for(double range=0.0;range<beam_list[i];range+=RESOLUTION){
            double x = range * cos(angle);
            double y = range * sin(angle);
            if(-WIDTH_2 <= x && x <= WIDTH_2 && -WIDTH_2 <= y && y <= WIDTH_2){
                int index = get_index_from_xy(x, y);
                if(0 <= index && index < GRID_NUM){
                    _grid_cells[index].state = CLEAR;
                }
            }
        }
    }

    // update grid cells
    std::cout << "update grid cells" << std::endl;
    for(int i=0;i<GRID_NUM;i++){
        grid_cells[i].update_state(_grid_cells[i].state);
    }
}

void DynamicCloudDetector::move_grid_cells(const double rot, const Eigen::Vector3d& trans)
{
    int count = 0;
    for(auto gc : grid_cells){
        if(gc.state > 0){
            count++;
        }
    }
    std::cout << "obs cell : " << count << std::endl;

    Eigen::Translation<double, 3> _trans(trans(0), trans(1), trans(2));
    // Eigen::Translation<double, 3> _trans(0, 0, 0);
    Eigen::Matrix3d _rot;
    _rot = Eigen::AngleAxisd(rot, Eigen::Vector3d::UnitZ());
    // _rot = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    Eigen::Affine3d affine = _rot * _trans;
    std::cout << "affine:\n" << affine.translation() << std::endl;

    std::vector<GridCell> _grid_cells(GRID_NUM);

    for(int i=0;i<GRID_NUM;i++){
        double x = get_x_from_index(i);
        double y = get_y_from_index(i);
        Eigen::Vector3d point(x, y, 0);

        point = affine * point;
        double _x = point(0);
        double _y = point(1);
        // std::cout << x << ", " << y << " to " << _x << ", " << _y << std::endl;

        if(-WIDTH_2 <= _x && _x <= WIDTH_2 && -WIDTH_2 <= _y && _y <= WIDTH_2){
            int index = get_index_from_xy(_x, _y);
            if(0 <= index && index < GRID_NUM){
                // _grid_cells[index] = grid_cells[i];
                _grid_cells[index].occupied_count = grid_cells[i].occupied_count;
                _grid_cells[index].clear_count = grid_cells[i].clear_count;
                // if(_grid_cells[index].state > 0){
                //     std::cout << x << ", " << y << " to " << _x << ", " << _y << std::endl;
                // }
            }
        }
    }
    grid_cells = _grid_cells;
}

void DynamicCloudDetector::devide_cloud(const CloudXYZIPtr& cloud, CloudXYZIPtr& dynamic_cloud, CloudXYZIPtr& static_cloud)
{
    std::cout << "devide cloud" << std::endl;
    for(const auto& pt : cloud->points){
        // std::cout << pt << std::endl;
        if(-WIDTH_2 <= pt.x && pt.x <= WIDTH_2 && -WIDTH_2 <= pt.y && pt.y <= WIDTH_2){
            int index = get_index_from_xy(pt.x, pt.y);
            if(0 <= index && index < GRID_NUM){
                GridCell gc = grid_cells[index];
                if(gc.clear_count + gc.occupied_count > 0){
                    double occupancy = gc.get_occupancy();
                    if(occupancy < OCCUPANCY_THRESHOLD){
                        dynamic_cloud->points.push_back(pt);
                    }else{
                        static_cloud->points.push_back(pt);
                    }
                }else{
                        static_cloud->points.push_back(pt);
                }
            }
        }
    }
}

int DynamicCloudDetector::get_index_from_xy(const double x, const double y)
{
    int _x = floor(x / RESOLUTION + GRID_WIDTH_2 + 0.5);
    int _y = floor(y / RESOLUTION + GRID_WIDTH_2 + 0.5);
    return _y * GRID_WIDTH + _x;
}

int DynamicCloudDetector::get_x_index_from_index(const int index)
{
    return index % GRID_WIDTH;
}

int DynamicCloudDetector::get_y_index_from_index(const int index)
{
    return index / GRID_WIDTH;
}

double DynamicCloudDetector::get_x_from_index(const int index)
{
    return (get_x_index_from_index(index) - GRID_WIDTH_2) * RESOLUTION;
}

double DynamicCloudDetector::get_y_from_index(const int index)
{
    return (get_y_index_from_index(index) - GRID_WIDTH_2) * RESOLUTION;
}

void DynamicCloudDetector::process(void)
{
    ros::spin();
}

void DynamicCloudDetector::GridCell::update_state(const int _state)
{
    state = _state;
    switch(_state){
        case CLEAR:
            clear_count++;
            break;
        case OCCUPIED:
            occupied_count++;
            break;
        // case UNKNOWN:
        //     clear_count = 0;
        //     occupied_count = 0;
        //     break;
    }
}

double DynamicCloudDetector::GridCell::get_occupancy(void)
{
    double occupancy = occupied_count / (double)(clear_count + occupied_count);
    return occupancy;
}

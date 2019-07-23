#include "dynamic_cloud_detector/dynamic_cloud_detector.h"

DynamicCloudDetector::DynamicCloudDetector(void)
:local_nh("~"), obstacles_cloud_sub(nh, "/velodyne_obstacles", 10), odom_sub(nh, "/odom/complement", 10)
, sync(sync_subs(10), obstacles_cloud_sub, odom_sub)
{
    local_nh.param("RESOLUTION", RESOLUTION, {0.1});
    local_nh.param("WIDTH", WIDTH, {20.0});
    local_nh.param("OCCUPANCY_THRESHOLD", OCCUPANCY_THRESHOLD, {0.2});
    local_nh.param("BEAM_NUM", BEAM_NUM, {720});
    local_nh.param("BUFFER_SIZE", BUFFER_SIZE, {5});

    GRID_WIDTH = WIDTH / RESOLUTION;
    GRID_NUM = GRID_WIDTH * GRID_WIDTH;
    WIDTH_2 = WIDTH / 2.0;
    GRID_WIDTH_2 = GRID_WIDTH / 2.0;

    grid_cells.resize(GRID_NUM);
    first_flag = true;

    cloud_buffer.reserve(BUFFER_SIZE + 1);
    beam_buffer.reserve(BUFFER_SIZE + 1);
    position_buffer.reserve(BUFFER_SIZE + 1);
    yaw_buffer.reserve(BUFFER_SIZE + 1);

    dynamic_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud/dynamic", 1);
    static_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud/static", 1);
    grid_pub = local_nh.advertise<nav_msgs::OccupancyGrid>("occupancy_grid", 1);

    sync.registerCallback(boost::bind(&DynamicCloudDetector::callback, this, _1, _2));

    std::cout << "=== dynamic cloud detector ===" << std::endl;
    std::cout << "RESOLUTION: " << RESOLUTION << std::endl;
    std::cout << "WIDTH: " << WIDTH << std::endl;
    std::cout << "OCCUPANCY_THRESHOLD: " << OCCUPANCY_THRESHOLD << std::endl;
    std::cout << "BEAM_NUM: " << BEAM_NUM << std::endl;
    std::cout << "BUFFER_SIZE: " << BUFFER_SIZE << std::endl;
}

void DynamicCloudDetector::callback(const sensor_msgs::PointCloud2ConstPtr& msg_obstacles_cloud, const nav_msgs::OdometryConstPtr& msg_odom)
{
    std::cout << "=== dynamic cloud detector ===" << std::endl;

    double start = ros::Time::now().toSec();
    Eigen::Vector3d current_position(msg_odom->pose.pose.position.x, msg_odom->pose.pose.position.y, msg_odom->pose.pose.position.z);
    double current_yaw = tf::getYaw(msg_odom->pose.pose.orientation);
    std::cout << "current_position: " << current_position.transpose() << std::endl;
    std::cout << "current_yaw: " << current_yaw << std::endl;

    if(!first_flag){
        CloudXYZIPtr obstacles_cloud(new CloudXYZI);
        pcl::fromROSMsg(*msg_obstacles_cloud, *obstacles_cloud);
        int cloud_size = obstacles_cloud->points.size();
        std::cout << "cloud size: " <<  cloud_size << std::endl;

        std::vector<double> beam_list;
        get_beam_list(obstacles_cloud, beam_list);

        cloud_buffer.push_back(obstacles_cloud);
        beam_buffer.push_back(beam_list);
        position_buffer.push_back(current_position);
        yaw_buffer.push_back(current_yaw);
        if(cloud_buffer.size() > BUFFER_SIZE){
            cloud_buffer.erase(cloud_buffer.begin());
            beam_buffer.erase(beam_buffer.begin());
            position_buffer.erase(position_buffer.begin());
            yaw_buffer.erase(yaw_buffer.begin());
        }

        input_cloud_to_grid_cells(cloud_buffer, beam_buffer, position_buffer, yaw_buffer);

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
                grid.data.push_back(gc.get_occupancy() * 100);
            }else{
                grid.data.push_back(UNKNOWN);
            }
        }
        grid_pub.publish(grid);
    }else{
        first_flag = false;
    }

    std::cout << "time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
}

void DynamicCloudDetector::input_cloud_to_grid_cells(const std::vector<CloudXYZIPtr>& _cloud_buffer, const std::vector<std::vector<double> >& _beam_buffer, const std::vector<Eigen::Vector3d>& _position_buffer, std::vector<double>& _yaw_buffer)
{
    // reset
    grid_cells = std::vector<GridCell>(GRID_NUM);

    int buffer_size = _cloud_buffer.size();
    Eigen::Vector3d current_position = position_buffer.back();
    double current_yaw = yaw_buffer.back();

    std::cout << "buffer size: " << buffer_size << std::endl;

    for(int i=0;i<buffer_size;i++){
        std::cout << "buffer t-" << fabs(i - buffer_size + 1) << std::endl;
        std::vector<int> states(GRID_NUM, UNKNOWN);

        double d_yaw = current_yaw - yaw_buffer[i];
        d_yaw = atan2(sin(d_yaw), cos(d_yaw));

        Eigen::Matrix3d last_yaw_rotation;
        last_yaw_rotation = Eigen::AngleAxisd(-yaw_buffer[i], Eigen::Vector3d::UnitZ());
        Eigen::Vector3d _current_position = last_yaw_rotation * current_position;
        Eigen::Vector3d _last_position = last_yaw_rotation * position_buffer[i];
        Eigen::Vector3d dp = _current_position - _last_position;
        std::cout << "dp: " << dp.transpose() << std::endl;
        std::cout << "dyaw: " << d_yaw << "[rad]" << std::endl;

        Eigen::Translation<double, 3> _trans(-dp(0), -dp(1), -dp(2));
        Eigen::Matrix3d _rot;
        _rot = Eigen::AngleAxisd(-d_yaw, Eigen::Vector3d::UnitZ());
        Eigen::Affine3d affine = _rot * _trans;
        //std::cout << "affine:\n" << affine.translation() << std::endl;

        CloudXYZIPtr transformed_cloud(new CloudXYZI);
        pcl::transformPointCloud(*_cloud_buffer[i], *transformed_cloud, affine);

        // search occupied grid cells
        int count = 0;
        for(const auto& pt : transformed_cloud->points){
            if(-WIDTH_2 <= pt.x && pt.x <= WIDTH_2 && -WIDTH_2 <= pt.y && pt.y <= WIDTH_2){
                int index = get_index_from_xy(pt.x, pt.y);
                if(0 <= index && index < GRID_NUM){
                    states[index] = OCCUPIED;
                    count++;
                }
            }
        }
        std::cout << count << " obstacle grid cells were added" << std::endl;

        // search clear grid cells
        std::cout << "set clear cells" << std::endl;
        count = 0;
        const double BEAM_ANGLE_RESOLUTION = 2.0 * M_PI / (double)BEAM_NUM;
        for(int j=0;j<BEAM_NUM;j++){
            double angle = j * BEAM_ANGLE_RESOLUTION - M_PI - d_yaw;
            angle = atan2(sin(angle), cos(angle));
            for(double range=0.0;range<beam_buffer[i][j];range+=RESOLUTION){
                double x = range * cos(angle) - dp(0);
                double y = range * sin(angle) - dp(1);
                if(-WIDTH_2 <= x && x <= WIDTH_2 && -WIDTH_2 <= y && y <= WIDTH_2){
                    int index = get_index_from_xy(x, y);
                    if(0 <= index && index < GRID_NUM){
                        states[index] = CLEAR;
                        count++;
                    }
                }
            }
        }
        std::cout << count << " clear grid cells were added" << std::endl;

        // update grid cells
        std::cout << "update grid cells" << std::endl;
        for(int j=0;j<GRID_NUM;j++){
            grid_cells[j].update_state(states[j]);
        }
    }
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

void DynamicCloudDetector::get_beam_list(const CloudXYZIPtr& input_cloud, std::vector<double>& _beam_list)
{
    _beam_list = std::vector<double>(BEAM_NUM, WIDTH);
    const double BEAM_ANGLE_RESOLUTION = 2.0 * M_PI / (double)BEAM_NUM;
    for(const auto& pt: input_cloud->points){
        double distance = sqrt(pt.x * pt.x + pt.y * pt.y);
        if(-WIDTH_2 <= pt.x && pt.x <= WIDTH_2 && -WIDTH_2 <= pt.y && pt.y <= WIDTH_2){
            double angle = atan2(pt.y, pt.x);
            int beam_index = (angle + M_PI) / BEAM_ANGLE_RESOLUTION;
            if(0 <= beam_index && beam_index < BEAM_NUM){
                if(_beam_list[beam_index] > distance){
                    _beam_list[beam_index] = distance;
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

#include "obstacle_detector/obstacle_detector.hpp"

ObstacleDetector::ObstacleDetector() : Node("chibi24_c_obstacle_detector")
{
    this->declare_parameter("hz_", 10);
    this->declare_parameter("laser_step", 3);
    this->declare_parameter("ignore_distance", 1.0);
    this->declare_parameter("ignore_angle_range_list", std::vector<double>({0.29, 0.98, 1.96}));
    this->declare_parameter("robot_frame", "base_link");
    this->declare_parameter("min_obstacle_distance", 0.25);

    this->get_parameter("hz_", hz_);
    this->get_parameter("laser_step", laser_step_);
    this->get_parameter("ignore_distance", ignore_distance_);
    this->get_parameter("ignore_angle_range_list", ignore_angle_range_list_);
    this->get_parameter("robot_frame", robot_frame_);
    this->get_parameter("min_obstacle_distance", min_obstacle_distance_);

    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::QoS(1).reliable(), std::bind(&ObstacleDetector::laser_scan_callback, this, std::placeholders::_1));
    obstacle_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/obstacle_pose", rclcpp::QoS(1).reliable());

    obstacle_pose_array_.header.frame_id = robot_frame_;
}

// hz_を返す関数
int ObstacleDetector::getFreq() { return hz_; }

void ObstacleDetector::process()
{
    if(flag_laser_scan_)
    {
        scan_obstacle();
        obstacle_pose_pub_->publish(obstacle_pose_array_);
    }
}

void ObstacleDetector::laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    laser_scan_ = *msg;
    flag_laser_scan_ = true;
}


void ObstacleDetector::scan_obstacle()
{
    obstacle_pose_array_.poses.clear();
    for(int i=0; i<laser_scan_.ranges.size(); i+=laser_step_)
    {
        
         if (laser_scan_.ranges[i] > min_obstacle_distance_) {
            geometry_msgs::msg::Pose obs_pose;
            obs_pose.position.x = laser_scan_.ranges[i] * cos(laser_scan_.angle_min + laser_scan_.angle_increment * i);
            obs_pose.position.y = laser_scan_.ranges[i] * sin(laser_scan_.angle_min + laser_scan_.angle_increment * i);
            obstacle_pose_array_.poses.push_back(obs_pose);
         }
        
    }
}

bool ObstacleDetector::is_ignore_scan(double angle)
{
    for(int i = 0; i < ignore_angle_range_list_.size(); i += 2)
    {
        if(ignore_angle_range_list_[i] < angle && angle < ignore_angle_range_list_[i + 1])
        {
            return true;
        }
    }
    return false;
}



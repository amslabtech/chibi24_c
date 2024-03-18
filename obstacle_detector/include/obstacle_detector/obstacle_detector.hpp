#ifndef OBSTACLR_DETECTOE_HPP
#define OBSTACLR_DETECTOE_HPP

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

class ObstacleDetector : public rclcpp::Node
{
    public:
        ObstacleDetector();     // コンストラクタ
        void process();
        int getFreq();  // hz_を返す関数
    
    private:
        void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        void scan_obstacle();
        bool is_ignore_scan(double angle);
        
        int hz_;
        int laser_step_;
        double ignore_distance_;
        std::string robot_frame_;
        std::vector<double> ignore_angle_range_list_;
        double min_obstacle_distance_;
        bool flag_laser_scan_ = false;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_pose_pub_;

        geometry_msgs::msg::PoseArray obstacle_pose_array_;
        sensor_msgs::msg::LaserScan laser_scan_;
};

#endif // OBSTACLR_DETECTOE_H
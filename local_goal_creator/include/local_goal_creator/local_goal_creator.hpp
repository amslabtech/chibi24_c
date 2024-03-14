#ifndef LOCAL_GOAL_CREATER
#define LOCAL_GOAL_CREATER
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/path.hpp>
using namespace std::chrono_literals;

class LocalGoalCreator : public rclcpp::Node
{
public:
    LocalGoalCreator();

private:
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    std::vector<geometry_msgs::msg::PoseStamped> global_path_;
    void pose_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void path_callback(nav_msgs::msg::Path::SharedPtr msg);
    double calc_distance(geometry_msgs::msg::Pose current, geometry_msgs::msg::Pose goal);
    int index_=0;
};

#endif
#ifndef LOCAL_PATH_PLANNER_HPP
#define LOCAL_PATH_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <memory>
#include "robot_state.hpp" 
#include "v.hpp"
#include <math.h>
#include <limits>
#include "roomba_500driver_meiji/msg/roomba_ctrl.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
using namespace std::chrono_literals;


class LocalPathPlanner : public rclcpp::Node {
public:
    LocalPathPlanner();
private:
    void goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void obstacle_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void timer_callback();
    void roomba_control(const double velocity, const double yawrate);
    void calc_final_input();
    double calc_evaluation(const std::vector<std::shared_ptr<RobotState>>& trajectory);
    double calc_heading_eval(const std::vector<std::shared_ptr<RobotState>>& trajectory);
    double calc_dist_eval(const std::vector<std::shared_ptr<RobotState>>& trajectory);
    double calc_vel_eval(const std::vector<std::shared_ptr<RobotState>>& trajectory);
    void visualize_traj(const std::vector<std::shared_ptr<RobotState>>& traj, const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& pub_local_path, rclcpp::Time now);
    std::vector<std::shared_ptr<RobotState>> calc_trajectory(double v, double y);
    std::shared_ptr<RobotState> motion(std::shared_ptr<RobotState> state, double v, double y, double dt);
    V calc_dynamic_window();
    bool can_move();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr local_goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_predict_path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_optimal_path_;
    rclcpp::Publisher<roomba_500driver_meiji::msg::RoombaCtrl>::SharedPtr roomba_pub_;
    geometry_msgs::msg::PoseArray obstacle_pose_;
    geometry_msgs::msg::PointStamped local_goal_;
    std::shared_ptr<RobotState> robot_state_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool has_local_goal_;
    bool has_obs_poses_;
};

#endif // LOCAL_PATH_PLANNER_HPP

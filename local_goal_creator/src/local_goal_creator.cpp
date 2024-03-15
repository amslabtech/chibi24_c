#include "local_goal_creator/local_goal_creator.hpp"

LocalGoalCreator::LocalGoalCreator() : Node("chibi24_c_local_goal_creator_node")
{
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/global_path",
        rclcpp::QoS(1).reliable(),
        std::bind(&LocalGoalCreator::path_callback, this, std::placeholders::_1));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/estimated_pose",
        rclcpp::QoS(1).reliable(),
        std::bind(&LocalGoalCreator::pose_callback, this, std::placeholders::_1));

    goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/local_goal", rclcpp::QoS(1).reliable());

    // 次の移動指令先を指示する距離感
    this->declare_parameter<double>("target_dist_to_goal", 5.0);
}

void LocalGoalCreator::path_callback(nav_msgs::msg::Path::SharedPtr msg)
{
    global_path_ = msg->poses;
}

void LocalGoalCreator::pose_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // printf("pose_callback \n");
    if (global_path_.empty())
    {
        return;
    }
    auto pose = *msg;
    double target_dist_to_goal= this->get_parameter("target_dist_to_goal").as_double();

    // 最新のゴールを取得
    auto goal = global_path_[index_];
    auto distance = calc_distance(pose.pose, goal.pose);

    // 距離が規定値未満の場合、適切な距離を取得できるまで処理を繰り返す
    while (distance < target_dist_to_goal && global_path_.size() > index_)
    {
        index_++;;
        goal = global_path_[index_];
        distance = calc_distance(pose.pose, goal.pose);
    }

  //  printf("target_dist_to_goal %f \n",target_dist_to_goal);
 //   printf("distance %f \n",distance);
    auto local_goal = geometry_msgs::msg::PointStamped();
    local_goal.header.frame_id = "map";
    local_goal.header.stamp = this->get_clock()->now();
    local_goal.point = goal.pose.position;
    goal_pub_->publish(local_goal);
}

double LocalGoalCreator::calc_distance(geometry_msgs::msg::Pose current, geometry_msgs::msg::Pose goal)
{
   // printf("goal x:%f y:%f  current x:%f y:%f \n",goal.position.x,goal.position.y,current.position.x,current.position.y);
    return std::hypot(goal.position.x - current.position.x, goal.position.y - current.position.y);
}

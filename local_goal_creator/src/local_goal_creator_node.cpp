#include "local_goal_creator/local_goal_creator.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<LocalGoalCreator> local_goal_creator = std::make_shared<LocalGoalCreator>();
    rclcpp::spin(local_goal_creator);
    rclcpp::shutdown();

    return 0;
}

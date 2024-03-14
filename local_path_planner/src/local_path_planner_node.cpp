#include "local_path_planner/local_path_planner.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<LocalPathPlanner> local_path_planner = std::make_shared<LocalPathPlanner>();
    rclcpp::spin(local_path_planner);
    rclcpp::shutdown();

    return 0;
}

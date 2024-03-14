#include "global_path_planner/a_star.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<AStar> astar = std::make_shared<AStar>();
  rclcpp::spin(astar);
  rclcpp::shutdown();

  return 0;
}

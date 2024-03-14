#include "obstacle_detector/obstacle_detector.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleDetector>();
    rclcpp::Rate loop_rate(node->getFreq());
    while(rclcpp::ok())
    {
        node->process();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
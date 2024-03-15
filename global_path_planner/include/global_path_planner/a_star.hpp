#ifndef ASTER
#define ASTER
#include "global_path_planner/node.hpp"
#include "global_path_planner/action.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <memory> 
#include <nav_msgs/msg/path.hpp>
using namespace std::chrono_literals;

class AStar: public rclcpp::Node
{
    public:
        AStar();

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        void create_aster();
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_; 
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_; 
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        nav_msgs::msg::OccupancyGrid map_;
        nav_msgs::msg::OccupancyGrid expand_map_;
        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        void obs_expander();
        void obs_expand(int index);
        bool is_Wall(std::shared_ptr<ANode>  node);
        std::list<Action> create_action_list();
        std::shared_ptr<ANode> create_way_point_node(int x, int y);
        std::shared_ptr<ANode> create_path(std::shared_ptr<ANode> start,std::shared_ptr<ANode> goal);



};

#endif
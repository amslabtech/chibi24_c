#include "global_path_planner/a_star.hpp"


AStar::AStar():Node("chibi24_c_global_path_planner_node"){
    // ロボットの移動方向とコストを定義する
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map",
         rclcpp::QoS(1).reliable(),
        std::bind(&AStar::map_callback,this,std::placeholders::_1)
        );
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map/new",rclcpp::QoS(1).reliable());
    this->declare_parameter<double>("expansion_size",0.2);

    this->declare_parameter<std::vector<int64_t>>("way_points_x", {-1,15,15,-18,-18,-1});
    this->declare_parameter<std::vector<int64_t>>("way_points_y", {0,0,14,14,0,0});
    this->declare_parameter<bool>("is_debug",1);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path",rclcpp::QoS(1).reliable());

}


void AStar::create_aster(){
        std::vector<int64_t> way_points_x =  this->get_parameter("way_points_x").as_integer_array();
        std::vector<int64_t> way_points_y =  this->get_parameter("way_points_y").as_integer_array();

        std::list<std::shared_ptr<ANode>> way_point_node_list;
        for(int i = 0; i < way_points_x.size(); i++){
            int x = way_points_x[i];
            int y = way_points_y[i];
            way_point_node_list.push_back(create_way_point_node(x,y));
        }

        auto start_node = way_point_node_list.front();

        for (const auto& way_point_node : way_point_node_list) {
            start_node = create_path(start_node,way_point_node);
        }

        std::list<std::shared_ptr<ANode>> result_list;
        while(start_node != nullptr){
          //  printf("x:%d y:%d  F:%f \n",start_node->x,start_node->y,start_node->getF());
            result_list.push_back(start_node);
            start_node = start_node->parent;
        }

        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id ="map";
        path_msg.poses.reserve(result_list.size());

        for(const auto& node : result_list ){
            geometry_msgs::msg::PoseStamped msg;
            msg.pose.position.x = node->x * map_.info.resolution + map_.info.origin.position.x;
            msg.pose.position.y = node->y * map_.info.resolution + map_.info.origin.position.y;
            path_msg.poses.push_back(msg);

         }
        std::reverse(path_msg.poses.begin(), path_msg.poses.end());
        path_pub_->publish(path_msg);
        
}

std::shared_ptr<ANode> AStar::create_way_point_node(int x, int y){
    
    int new_x = std::round((x - map_.info.origin.position.x) / map_.info.resolution);
    int new_y = std::round((y - map_.info.origin.position.y) / map_.info.resolution);
    return std::make_shared<ANode>(new_x, new_y, 0.0);
}

void AStar::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    map_ = *msg;
    expand_map_ = map_;
    obs_expander();
    map_pub_->publish(expand_map_);
    create_aster();
}

// マップの拡張
// マップの障害物を拡張する
void AStar::obs_expander(){
    int size = map_.data.size();
    for(int i = 0; i < size; i++){
        if(map_.data[i] == 100){
            obs_expand(i);
        }
    }
    
}

void AStar::obs_expand(int index){
    int x = index % map_.info.width;
    int y = index / map_.info.width;
    double expansion_size=this->get_parameter("expansion_size").as_double();

    // マップの解像度を考慮して拡張サイズを調整し、その後半分のサイズにする
    int half_expansion_size = (expansion_size / map_.info.resolution) / 2;
    // expation_sizeの範囲内で障害物を拡張
    for(int i = -half_expansion_size; i <= half_expansion_size; i++){
        for(int j = -half_expansion_size; j <= half_expansion_size; j++){
            int newX = x + i;
            int newY = y + j;
            // newXやnewYがマップの範囲外にならないようにチェック
            if(newX >= 0 && newX < map_.info.width && newY >= 0 && newY < map_.info.height){
                int grid_index = newY * map_.info.width + newX;
                expand_map_.data[grid_index] = 100; // 障害物を設定
            }
        }
    }
}



// Aスターを実装する
// 上下左右１斜√２で計算する
// 中継点を作成し、way_pointを通りゴールノードを参照するようにする
// つまりこうなる
// ロボ→waypoint1→waypoint2
std::shared_ptr<ANode> AStar::create_path(std::shared_ptr<ANode> start,std::shared_ptr<ANode> goal){
    std::list<std::shared_ptr<ANode>> open_list;
    std::list<std::shared_ptr<ANode>> close_list;

    open_list.push_back(start);

    auto point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/point",rclcpp::QoS(1).reliable());
    
    bool is_debug;
    this->get_parameter<bool>("is_debug",is_debug);
    std::list<Action> actin_list = create_action_list();
    while(rclcpp::ok()){
        if(open_list.empty()){
            break;
        }

        open_list.sort([](const std::shared_ptr<ANode>& a, const std::shared_ptr<ANode>& b) {
            return *a< *b;
        });

      
        std::shared_ptr<ANode> current = open_list.front();
        open_list.pop_front();
    
        if(*current == *goal){
            return current;
        }

      //  printf("current x %d y %d f %f goal x %d y %d \n",current->x,current->y,current->getF(),goal->x,goal->y);
        close_list.push_back(current);

        // デバッグ中は捜査しているポイントをパブリッシュする
        if(is_debug){
            geometry_msgs::msg::PointStamped msg;
            msg.header.frame_id ="map";
            msg.point.x = current->x* map_.info.resolution + map_.info.origin.position.x;
            msg.point.y = current->y * map_.info.resolution + map_.info.origin.position.y;
            point_pub_->publish(msg);
        }


        for (const auto& action : actin_list) {

            int new_x = current->x + action.x;
            int new_y = current->y + action.y;
            double g = current->g + action.cost;
            auto new_node = std::make_shared<ANode>(new_x, new_y, g, goal,current);
    
            if(is_Wall(new_node)){
                continue;
            }

            auto find_it = std::find_if(open_list.begin(), open_list.end(), [new_node](std::shared_ptr<ANode> node) { 
                return *node == *new_node;
            });
            // openリストに含まれていた場合
            if (find_it != open_list.end()) {
                auto foundNode = *find_it; 
                // 取得した結果のF値が大きい場合
                if(*foundNode > *new_node){
                    foundNode->g = new_node->g;
                    foundNode->parent = current;
                }
                continue;
            }

            find_it = std::find_if(close_list.begin(), close_list.end(), [new_node](std::shared_ptr<ANode> node) { 
                return *node == *new_node;
            });
             if (find_it != close_list.end()) {
                auto foundNode = *find_it; 
                if(*foundNode > *new_node){
                    foundNode->g = new_node->g;
                    foundNode->parent = current;
                    close_list.erase(find_it);
                    open_list.push_back(foundNode);
                }
                continue;
            }
            open_list.push_back(new_node);
        }
    }
}

bool AStar::is_Wall(std::shared_ptr<ANode>  node){
    long grid_index = node->y * map_.info.width + node->x;
    return expand_map_.data[grid_index] == 100;
}


std::list<Action> AStar::create_action_list(){
    std::list<Action> actions;
    actions.push_back(Action(1,0,1)); // 右
    actions.push_back(Action(-1,0,1)); // 左
    actions.push_back(Action(0,1,0)); // 上
    actions.push_back(Action(0,-1,1)); // 下
    actions.push_back(Action(1,1,std::sqrt(2))); // 右上
    actions.push_back(Action(1,-1,std::sqrt(2))); // 右下
    actions.push_back(Action(-1,1,std::sqrt(2))); // 左上
    actions.push_back(Action(-1,1,std::sqrt(2))); // 左下

    return actions;
}


#include "local_path_planner/local_path_planner.hpp"
#define MIN_OBSTRACLE_COST -std::numeric_limits<float>::infinity()
#define MIN_SCORE -std::numeric_limits<float>::infinity()

LocalPathPlanner::LocalPathPlanner() : Node("chibi24_c_local_path_planner")
{
    this->declare_parameter<double>("min_vel", 0.0);
    this->declare_parameter<double>("max_yawrate", 0.1);
    this->declare_parameter<double>("max_accel", 1000.0);
    this->declare_parameter<double>("max_dyawrate", 1000.0);
    this->declare_parameter<double>("v_reso", 0.05);
    this->declare_parameter<double>("yawrate_reso", 0.02);
    this->declare_parameter<double>("dt", 0.1);
    this->declare_parameter<double>("speed_cost_gain", 0.5);
    this->declare_parameter<double>("stop_vel_th", 0.1);
    this->declare_parameter<double>("stop_yawrate_th", 0.1);
    this->declare_parameter<double>("weight_heading", 0.7);
    this->declare_parameter<double>("weight_dist", 0.8);
    this->declare_parameter<double>("weight_vel", 0.6);
    this->declare_parameter<double>("roomba_radius", 0.25);
    this->declare_parameter<double>("radius_margin", 0.04);
    this->declare_parameter<double>("search_range", 0.95);
    this->declare_parameter<double>("max_vel", 0.1);
    this->declare_parameter<double>("predict_time", 6.0);
    this->declare_parameter<bool>("is_visible", true);
    this->declare_parameter<double>("goal_tolerance", 0.5);
    this->declare_parameter<std::string>("robot_frame", "base_link");

    local_goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/local_goal",
        rclcpp::QoS(1).reliable(),
        std::bind(&LocalPathPlanner::goal_callback, this, std::placeholders::_1));

    obstacle_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/obstacle_pose",
        rclcpp::QoS(1).reliable(),
        std::bind(&LocalPathPlanner::obstacle_callback, this, std::placeholders::_1));
    pub_predict_path_ = this->create_publisher<nav_msgs::msg::Path>("/predict_local_paths", rclcpp::QoS(1).reliable());
    pub_optimal_path_ = this->create_publisher<nav_msgs::msg::Path>("/optimal_local_path", rclcpp::QoS(1).reliable());

    timer_ = this->create_wall_timer(500ms, std::bind(&LocalPathPlanner::timer_callback, this));
    robot_state_ = std::make_shared<RobotState>();
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    roomba_pub_ = this->create_publisher<roomba_500driver_meiji::msg::RoombaCtrl>("/roomba/control", rclcpp::QoS(1).reliable());
}

void LocalPathPlanner::goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    // ゴールがマップ座標系なので実際の移動と合わせるためにルンバ座標系に置き換える処理
    geometry_msgs::msg::TransformStamped transform;
    try
    {

        transform = tf_buffer_->lookupTransform(
            this->get_parameter("robot_frame").as_string(), 
            "map", 
            tf2::TimePointZero);
        
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "トランスフォーム取得エラー: %s", ex.what());
        has_local_goal_ = false;
        return;
    }

    // 座標を変換する
    tf2::doTransform(*msg, local_goal_, transform);
    has_local_goal_ = true;
}

void LocalPathPlanner::obstacle_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  //  printf("obstacle_callback \n");
    obstacle_pose_ = *msg;
    has_obs_poses_ = true;
}

void LocalPathPlanner::timer_callback()
{
   // printf("timer_callback \n");
    if (can_move())
    {
      //  printf("start \n");
        calc_final_input();
        roomba_control(robot_state_->velocity, robot_state_->yaw_rate);
    }
    else
    {
        roomba_control(0, 0);
    }
}

bool LocalPathPlanner::can_move()
{
    if (!(has_local_goal_ && has_obs_poses_))
    {
        return false; // msg受信済みか
    }

    const double dx = local_goal_.point.x;
    const double dy = local_goal_.point.y;
    const double dist_to_goal = hypot(dx, dy); // 現在位置からゴールまでの距離

    double goal_tolerance=this->get_parameter("goal_tolerance").as_double(); // 回転解像度
    
    if (dist_to_goal > goal_tolerance)
    {
        return true;
    }
    else
    {
        roomba_control(0.0, 0.0);
        exit(0); // ノード終了
    }
}

void LocalPathPlanner::roomba_control(const double velocity, const double yawrate)
{
    roomba_500driver_meiji::msg::RoombaCtrl cmd_vel;
    cmd_vel.mode = 11; // 任意の動作を実行するモード
    cmd_vel.cntl.linear.x = velocity;
    cmd_vel.cntl.angular.z = yawrate;

    roomba_pub_->publish(cmd_vel);
}

void LocalPathPlanner::calc_final_input()
{
    // ロボットの初期値を作成
    V dw = calc_dynamic_window();
    // 速度解像度
    double v_reso = this->get_parameter("v_reso").as_double();
    // 回転解像度
    double yawrate_reso = this->get_parameter("yawrate_reso").as_double();
    double stop_vel_th = this->get_parameter("stop_vel_th").as_double();
    double stop_yawrate_th = this->get_parameter("stop_yawrate_th").as_double();

    std::vector<std::vector<std::shared_ptr<RobotState>>> trajectories;
    double max_velocity = 0.0;
    double max_yawrate = 0.0;
    double max_score = MIN_SCORE;

    // 後々の表示用に利用
    int i = 0;
    int index_of_max_score = 0;
    // 最低速度から最高速度まで速度解像度分移動しながら
    // 最低回転速度から最高回転速度まで回転解像度分くるくる回る

    printf("dw min_vel %f, max_vel %f \n",dw.min_vel,dw.max_vel);
   for (double v = dw.min_vel; v <= dw.max_vel; v += v_reso)
    {
        for (double y = dw.min_yawrate; y <= dw.max_yawrate; y += yawrate_reso)
        {

            auto trajectory = calc_trajectory(v, y);

            double score = calc_evaluation(trajectory);
            trajectories.push_back(trajectory);

            // ほとんど停止の場合最大スコアは最小とする
            if (v < stop_vel_th and abs(y) < stop_yawrate_th)
            {
                score = MIN_SCORE;
            }

            //printf("velocity %f, yawrate %f  score %f  \n",v,y,score);

            // 最大値の更新
            if (max_score < score)
            {
                max_score = score;
                max_velocity = v;
                max_yawrate = y;
                index_of_max_score = i;
            }

            i++;
        }
    }

  //  printf("max_velocity %f, max_yawrate %f \n",max_velocity,max_yawrate);
    // 現在速度の記録
    robot_state_->velocity = max_velocity;
    robot_state_->yaw_rate = max_yawrate;

    // pathの可視化
    if (this->get_parameter("is_visible").as_bool())
    {
        const auto now = this->get_clock()->now();
        for (i = 0; i < trajectories.size(); i++)
        {
            if (i == index_of_max_score)
                visualize_traj(trajectories[i], pub_optimal_path_, now);
            else
                visualize_traj(trajectories[i], pub_predict_path_, now);
        }
    }
}

void LocalPathPlanner::visualize_traj(const std::vector<std::shared_ptr<RobotState>> &traj, const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pub_local_path, rclcpp::Time now)
{

    std::string robot_frame = this->get_parameter("robot_frame").as_string();
    nav_msgs::msg::Path local_path;
    local_path.header.stamp = now;
    local_path.header.frame_id = robot_frame;

    for (const auto &state : traj)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now;
        pose.header.frame_id = robot_frame;
        pose.pose.position.x = state->x;
        pose.pose.position.y = state->y;
        local_path.poses.push_back(pose);
    }

    pub_local_path->publish(local_path);
}

// 評価関数を計算
double LocalPathPlanner::calc_evaluation(const std::vector<std::shared_ptr<RobotState>> &trajectory)
{
    double weight_heading= this->get_parameter("weight_heading").as_double();
   
    const double heading_score = weight_heading * calc_heading_eval(trajectory);

    double weight_dist=this->get_parameter("weight_dist").as_double();
    
    const double distance_score = weight_dist * calc_dist_eval(trajectory);

    double weight_vel= this->get_parameter("weight_vel").as_double();
   
    const double velocity_score = weight_vel * calc_vel_eval(trajectory);

    const double total_score = heading_score + distance_score + velocity_score;

    return total_score;
}

double LocalPathPlanner::calc_heading_eval(const std::vector<std::shared_ptr<RobotState>> &trajectory)
{
    const auto last_trajectory = trajectory.back();
    return last_trajectory->calculate_goal_direction_theta(local_goal_.point.x, local_goal_.point.y);
}

double LocalPathPlanner::calc_dist_eval(const std::vector<std::shared_ptr<RobotState>> &trajectory)
{
    double roomba_radius = this->get_parameter("roomba_radius").as_double();
    double radius_margin = this->get_parameter("radius_margin").as_double();
    double min_dist = this->get_parameter("search_range").as_double();
    // pathの点と障害物のすべての組み合わせを探索
    for (const auto &state : trajectory)
    {
        for (const auto &obs_pose : obstacle_pose_.poses)
        {
            // pathのうちの１点と障害物の距離を計算
            auto dx = obs_pose.position.x - state->x;
            auto dy = obs_pose.position.y - state->y;
            auto dist = hypot(dx, dy);

           // printf("position x %f y %f   state x %f y %f dist %f \n",obs_pose.position.x,obs_pose.position.y,state->x,state->y,dist);
            // 壁に衝突したパスを評価
            if (dist <= roomba_radius + radius_margin)
            {
                return MIN_OBSTRACLE_COST;
            }
		    
            // 最小値の更新
            if (dist < min_dist)
            {
                min_dist = dist;
            }
        }
    }

    double search_range = this->get_parameter("search_range").as_double();

    return min_dist / search_range; // 正規化
}

double LocalPathPlanner::calc_vel_eval(const std::vector<std::shared_ptr<RobotState>> &trajectory)
{

    double max_vel = this->get_parameter("max_vel").as_double();
    

    const auto last_trajectory = trajectory.back();
    if (0.0 < last_trajectory->velocity) // 前進
    {
        return last_trajectory->velocity / max_vel; // 正規化
    }

    return 0.0; // 後退
}

std::vector<std::shared_ptr<RobotState>> LocalPathPlanner::calc_trajectory(double v, double y)
{
    std::vector<std::shared_ptr<RobotState>> trajectory;

    double dt=this->get_parameter("dt").as_double();
    

    double predict_time=   this->get_parameter("predict_time").as_double();

    std::shared_ptr<RobotState> state = std::make_shared<RobotState>();
    for (double t = 0; t <= predict_time; t += dt)
    {
        state = motion(state, v, y, dt);
	 //   printf("state x:%f y:%f theta:%f v:%f yaw:%f \n",state->x,state->y,state->theta,state->velocity,state->yaw_rate);
        trajectory.push_back(state);
    }

    return trajectory;
}

std::shared_ptr<RobotState> LocalPathPlanner::motion(std::shared_ptr<RobotState> state, double v, double y, double dt)
{
    auto local = std::make_shared<RobotState>(*state);
    local->update(v, y, dt);
    return local;
}

V LocalPathPlanner::calc_dynamic_window()
{
    // ロボットの物理的な最大および最小の速度および旋回速度
    V vs;
    this->get_parameter("max_vel", vs.max_vel);
    this->get_parameter("min_vel", vs.min_vel);
    this->get_parameter("max_yawrate", vs.max_yawrate);
    this->get_parameter("max_yawrate", vs.min_yawrate);
    vs.min_yawrate *= -1;

    // 現在の速度および旋回速度に基づいて、最大加速度および最大旋回加速度を考慮した範囲
    double max_accel=this->get_parameter("max_accel").as_double();
    

    double dt= this->get_parameter("dt").as_double();


    double max_dyawrate=this->get_parameter("max_dyawrate").as_double();
    V vd;
    vd.min_vel = robot_state_->velocity - max_accel * dt;
    vd.max_vel = robot_state_->velocity + max_accel * dt;
    vd.min_yawrate = robot_state_->yaw_rate - max_dyawrate * dt;
    vd.max_yawrate = robot_state_->yaw_rate + max_dyawrate * dt;

    V va;
    va.min_vel = std::max(vs.min_vel, vd.min_vel);
    va.max_vel = std::min(vs.max_vel, vd.max_vel);
    va.min_yawrate = std::max(vs.min_yawrate, vd.min_yawrate);
    va.max_yawrate = std::min(vs.max_yawrate, vd.max_yawrate);

    return va;
}

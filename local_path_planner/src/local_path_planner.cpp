#include "local_path_planner/local_path_planner.hpp"
#define MAX_OBSTRACLE_COST std::numeric_limits<float>::infinity()
#define MAX_SCORE std::numeric_limits<float>::infinity()

LocalPathPlanner::LocalPathPlanner() : Node("chibi24_c_local_path_planner")
{
    this->declare_parameter<double>("max_speed", 0.5);
    this->declare_parameter<double>("min_speed", 0.1);
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
    this->declare_parameter<std::string>("robot_frame", "odom");

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

        std::string robot_frame;
        this->get_parameter("robot_frame", robot_frame);

        transform = tf_buffer_->lookupTransform(
            robot_frame, "map", tf2::TimePointZero);
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
    printf("obstacle_callback \n");
    obstacle_pose_ = *msg;
    has_obs_poses_ = true;
}

void LocalPathPlanner::timer_callback()
{
    printf("timer_callback \n");
    if (can_move())
    {
        printf("start \n");
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

    double goal_tolerance; // 回転解像度
    this->get_parameter("goal_tolerance", goal_tolerance);
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

    double v_reso; // 速度解像度
    this->get_parameter("v_reso", v_reso);
    double yawrate_reso; // 回転解像度
    this->get_parameter("yawrate_reso", yawrate_reso);

    double max_speed;
    this->get_parameter("max_speed", max_speed);
    double speed_cost_gain;
    this->get_parameter("speed_cost_gain", speed_cost_gain);

    double stop_vel_th;
    this->get_parameter("stop_vel_th", stop_vel_th);
    double stop_yawrate_th;
    this->get_parameter("stop_yawrate_th", stop_yawrate_th);

    std::vector<std::vector<std::shared_ptr<RobotState>>> trajectories;
    double max_velocity = 0.0;
    double max_yawrate = 0.0;

    double max_score = 0.0;

    // 後々の表示用に利用
    int i = 0;
    int index_of_max_score = 0;
    // 最低速度から最高速度まで速度解像度分移動しながら
    // 最低回転速度から最高回転速度まで回転解像度分くるくる回る
    for (double v = dw.min_speed; v < dw.max_speed; v += v_reso)
    {
        for (double y = dw.min_yawrate; y < dw.max_yawrate; y += yawrate_reso)
        {

            auto trajectory = calc_trajectory(v, y);

            double score = calc_evaluation(trajectory);
            trajectories.push_back(trajectory);

            // ほとんど停止の場合最大スコアは最大とする
            if (v < stop_vel_th and abs(y) < stop_yawrate_th)
            {
                score = MAX_SCORE;
            }

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
    // 現在速度の記録
    robot_state_->velocity = max_velocity;
    robot_state_->yaw_rate = max_yawrate;

    bool is_visible;
    this->get_parameter("is_visible", is_visible);
    // pathの可視化
    if (is_visible)
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
    nav_msgs::msg::Path local_path;
    local_path.header.stamp = now;
    local_path.header.frame_id = "map";

    for (const auto &state : traj)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now;
        pose.header.frame_id = "map";
        pose.pose.position.x = state->x;
        pose.pose.position.y = state->y;
        local_path.poses.push_back(pose);
    }

    pub_local_path->publish(local_path);
}

// 評価関数を計算
double LocalPathPlanner::calc_evaluation(const std::vector<std::shared_ptr<RobotState>> &trajectory)
{
    double weight_heading;
    this->get_parameter("weight_heading", weight_heading);
    const double heading_score = weight_heading * calc_heading_eval(trajectory);

    double weight_dist;
    this->get_parameter("weight_dist", weight_dist);
    const double distance_score = weight_dist * calc_dist_eval(trajectory);

    double weight_vel;
    this->get_parameter("weight_vel", weight_vel);
    const double velocity_score = weight_vel * calc_vel_eval(trajectory);

    const double total_score = heading_score + distance_score + velocity_score;

    return total_score;
}

double LocalPathPlanner::calc_heading_eval(const std::vector<std::shared_ptr<RobotState>> &trajectory)
{
    const auto last_trajectory = trajectory.back();
    // ゴールまでの方位差分
    const double target_theta = last_trajectory->calculate_goal_direction_theta(local_goal_.point.x, local_goal_.point.y);

    // headingの評価値
    // 180から引くことで、結果を最大化する
    // つまり、0度の場合180となって最大パワー
    const double heading_eval = (M_PI - target_theta) / M_PI; // 正規化

    return heading_eval;
}

double LocalPathPlanner::calc_dist_eval(const std::vector<std::shared_ptr<RobotState>> &trajectory)
{
    double roomba_radius;
    this->get_parameter("roomba_radius", roomba_radius);

    double radius_margin;
    this->get_parameter("radius_margin", radius_margin);

    double min_dist = MAX_OBSTRACLE_COST;
    // pathの点と障害物のすべての組み合わせを探索
    for (const auto &state : trajectory)
    {
        for (const auto &obs_pose : obstacle_pose_.poses)
        {
            // pathのうちの１点と障害物の距離を計算
            auto dx = obs_pose.position.x - state->x;
            auto dy = obs_pose.position.y - state->y;
            auto dist = hypot(dx, dy);

            // 壁に衝突したパスを評価
            if (dist <= roomba_radius + radius_margin)
            {
                return MAX_OBSTRACLE_COST;
            }

            // 最小値の更新
            if (dist < min_dist)
            {
                min_dist = dist;
            }
        }
    }

    double search_range;
    this->get_parameter("search_range", search_range);

    return min_dist / search_range; // 正規化
}

double LocalPathPlanner::calc_vel_eval(const std::vector<std::shared_ptr<RobotState>> &trajectory)
{

    double max_vel;
    this->get_parameter("max_vel", max_vel);

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

    double dt;
    this->get_parameter("dt", dt);

    double predict_time;
    this->get_parameter("predict_time", predict_time);

    std::shared_ptr<RobotState> state = std::make_shared<RobotState>();
    for (double t = 0; t <= predict_time; t += dt)
    {
        state = motion(state, v, y, dt);
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
    this->get_parameter("max_speed", vs.max_speed);
    this->get_parameter("min_speed", vs.min_speed);
    this->get_parameter("max_yawrate", vs.max_yawrate);
    this->get_parameter("max_yawrate", vs.min_yawrate);
    vs.min_yawrate *= -1;

    // 現在の速度および旋回速度に基づいて、最大加速度および最大旋回加速度を考慮した範囲
    double max_accel;
    this->get_parameter("max_accel", max_accel);

    double dt;
    this->get_parameter("dt", dt);

    double max_dyawrate;
    this->get_parameter("max_dyawrate", max_dyawrate);
    V vd;
    vd.min_speed = robot_state_->velocity - max_accel * dt;
    vd.max_speed = robot_state_->velocity + max_accel * dt;
    vd.min_yawrate = robot_state_->yaw_rate - max_dyawrate * dt;
    vd.max_yawrate = robot_state_->yaw_rate + max_dyawrate * dt;

    V va;
    va.min_speed = std::max(vs.min_speed, vd.min_speed);
    va.max_speed = std::min(vs.max_speed, vd.max_speed);
    va.min_yawrate = std::max(vs.min_yawrate, vd.min_yawrate);
    va.max_yawrate = std::min(vs.max_yawrate, vd.max_yawrate);

    return va;
}
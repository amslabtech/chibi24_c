#ifndef ROBOT_STATE
#define ROBOT_STATE
#include <math.h>

class RobotState {
    public:
        double x;        // X座標
        double y;        // Y座標
        double theta;    // 向き（ラジアン単位）
        double velocity; // 速度
        double yaw_rate; // 旋回速度

        RobotState() : x(0.0), y(0.0), theta(0.0), velocity(0.0), yaw_rate(0.0) {}

        void update(double vel, double yawrate, double dt) {
            theta = normalize_angle(theta + yawrate * dt);
            x += vel * std::cos(theta) * dt;
            y += vel * std::sin(theta) * dt;
            velocity = vel;
            yaw_rate = yawrate;
        }

        double calculate_goal_direction_theta(double goal_x, double goal_y) {
            double dx = goal_x - x;
            double dy = goal_y - y;
            const double goal_theta = std::atan2(dy, dx);
            return normalize_angle(std::abs(goal_theta -theta));
        }
    private:

    // 適切な角度(-M_PI ~ M_PI)を返す
    double normalize_angle(double angle)
    {
        while(M_PI  < angle) angle -= 2.0*M_PI;
        while(angle < -M_PI) angle += 2.0*M_PI;

        return angle;
    }
};

#endif
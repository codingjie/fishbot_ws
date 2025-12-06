#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

#define KP 1.00
#define KD 0.001
#define KI 0.005
#define DESIRED_DISTANCE 0.15f  // 单位：米
#define LOOK_AHEAD_DIS 0.15
#define PI 3.1415927

class WallFollower : public rclcpp::Node {
public:
    // 构造函数，初始化节点名并创建订阅者和发布者
    WallFollower() : Node("wall_follower") {
        // 创建速度控制指令的发布者，话题为 /cmd_vel，队列大小为10
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // 创建激光雷达数据的订阅者，话题为 /scan，使用回调函数处理
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10,
            std::bind(&WallFollower::scan_callback, this, std::placeholders::_1)
        );
    }

private:
    // 激光雷达数据处理回调函数
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    //     size_t total_ranges = msg->ranges.size(); // 获取总采样数目 360

    //     // 提取机器人右侧和正前方的激光数据索引（角度大致为 270° 和 180°）
    //     size_t right_idx = total_ranges * 3 / 4;   // 右侧角度索引
    //     size_t front_idx = 0;       // 前方角度索引

    //     // 获取对应方向上的距离值
    //     float right_dist = msg->ranges[right_idx];
    //     float front_dist = msg->ranges[front_idx];

    //     // 计算偏差值（右手墙距离偏差）
    //     float error = DESIRED_DISTANCE - right_dist;

    //     // 创建并初始化速度消息
    //     geometry_msgs::msg::Twist cmd;

    //     // 如果前方障碍物距离太近，则左转避障
    //     if (front_dist < DESIRED_DISTANCE) {
    //         cmd.linear.x = 0.0;
    //         cmd.angular.z = 0.5;  // 左转
    //     } else {
    //         // 正常行驶，沿着墙壁，用 P 控制偏差角速度
    //         cmd.linear.x = 0.15;
    //         cmd.angular.z = 1.0 * error;  // 简单比例控制（P 控制）
    //     }

    //     // 发布速度指令
    //     cmd_pub_->publish(cmd);
    //     // 添加日志输出
    //     RCLCPP_INFO(this->get_logger(), "Right dist: %.3f m, Front dist: %.3f m", right_dist, front_dist);
    
        double a_angle = -45.0 / 180.0 * PI; // 右前方向
        double b_angle = -90.0 / 180.0 * PI; // 正右方向
        unsigned int a_index;
        // unsigned int a_index = (unsigned int)(floor((a_angle - msg->angle_min) / msg->angle_increment)); // a点对应的索引
        unsigned int b_index = (unsigned int)(floor((b_angle - msg->angle_min) / msg->angle_increment)); // b点对应的索引（90度方向）


        // 处理当起始角大于 45 度时，避免角度越界
        if (msg->angle_min > 45.0 / 180.0 * PI) {
            a_angle = msg->angle_min;
            a_index = 0;
        } else {
            a_index = (unsigned int)(floor((45.0 / 180.0 * PI - msg->angle_min) / msg->angle_increment));
        }
        
        double a_range = 0.0;
        double b_range = 0.0;
        // 获取 a 点的有效距离，若无效则赋默认值 4
        if (!std::isinf(msg->ranges[a_index]) && !std::isnan(msg->ranges[a_index])) {
            a_range = msg->ranges[a_index];
        }
        else {
            a_range = msg->range_max;
        }
        // 获取 b 点的有效距离，若无效则赋默认值 4
        if (!std::isinf(msg->ranges[b_index]) && !std::isnan(msg->ranges[b_index])) {
            b_range = msg->ranges[b_index];
        }
        else {
            b_range = msg->range_max;
        }

        // 计算几何关系，用于推算机器人到墙的垂直距离及朝向
        double alpha = atan((a_range * cos(b_angle - a_angle) - b_range) / (a_range * sin(b_angle - a_angle))); // alpha角，用于描述车体偏转角度
        double AB = b_range * cos(alpha); // 车体当前与墙之间的实际距离（垂直距离）
        double projected_dis = AB + LOOK_AHEAD_DIS * sin(alpha); // 前瞻预测距离（考虑车辆运动延迟）
        error = DESIRED_DISTANCE - projected_dis; // 与期望墙距的误差

        // 输出调试信息
        RCLCPP_INFO(this->get_logger(), "projected_dis = %f", projected_dis);
        RCLCPP_INFO(this->get_logger(), "error = %f", error);
        RCLCPP_INFO(this->get_logger(), "del_time = %f\n", del_time);

        this->pid_control(); // 执行 PID 控制
    }

    // PID 控制器：根据误差计算转角
    void pid_control() {
        geometry_msgs::msg::Twist cmd; // 创建并初始化速度消息

        double tmoment = this->get_clock()->now().seconds(); // 当前时间戳（秒）
        // 防止第一次除以 0（首次没有上一次时间）
        if (prev_tmoment == 0.0) {
            prev_tmoment = tmoment;
            return;  // 第一次跳过控制
        }

        del_time = tmoment - prev_tmoment; // 当前与上一帧之间的时间差
        integral += prev_error * del_time; // 误差积分
        cmd.angular.z = -(KP * error + KD * (error - prev_error) / del_time + KI * integral); // PID 控制公式

        prev_tmoment = tmoment; // 更新时间戳
        prev_error = error; // 保存当前误差作为下一次的前一误差

        // 根据转向角动态调整小车速度（转弯越大，速度越慢）
        if (abs(cmd.angular.z) > 20.0 / 180.0 * PI) {
            cmd.linear.x = 2.0;
        } 
        else if (abs(cmd.angular.z) > 10.0 / 180.0 * PI) {
            cmd.linear.x = 3.0;
        } 
        else {
            cmd.linear.x = 5.0;
        }

        cmd_pub_->publish(cmd);
    }

    // 发布者：发送速度控制指令
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    // 订阅者：接收激光雷达数据
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    double prev_error = 0.0; // 上一次误差
    double prev_tmoment = 0.0; // 上一帧时间
    double error = 0.0; // 当前误差
    double integral = 0.0; // 积分累加项
    double del_time = 0.0; // 时间差
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);                              // 初始化 ROS2
    rclcpp::spin(std::make_shared<WallFollower>());        // 运行 WallFollower 节点
    rclcpp::shutdown();                                    // 清理并关闭 ROS2
    return 0;
}

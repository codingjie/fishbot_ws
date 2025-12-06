#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include "math.h"

#define KP 1.00                          // 比例增益
#define KD 0.001                         // 微分增益
#define KI 0.005                         // 积分增益
#define DESIRED_DISTANCE_RIGHT 1.0      // 期望车体右侧与墙的距离（AB）m
#define LOOK_AHEAD_DIS 1.0              // 前瞻距离（AC）m
#define PI 3.1415927

class SubscribeAndPublish {
public:
    SubscribeAndPublish() {
        // 1. 初始化话题的发布和订阅
        drive_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1000);  // 控制指令发布器
        scan_sub = nh.subscribe("/scan", 1000, &SubscribeAndPublish::callback, this);     // 激光雷达订阅器
    }

    void callback(const sensor_msgs::LaserScan& lidar_info) {
        // 2. 获取激光雷达在两个方向的测量距离
        unsigned int b_index = (unsigned int)(floor((90.0 / 180.0 * PI - lidar_info.angle_min) / lidar_info.angle_increment));  // b点对应的索引（90度方向）
        double b_angle = 90.0 / 180.0 * PI;    // b点方向角
        double a_angle = 45.0 / 180.0 * PI;    // a点方向角
        unsigned int a_index;

        // 处理当起始角大于 45 度时，避免角度越界
        if (lidar_info.angle_min > 45.0 / 180.0 * PI) {
            a_angle = lidar_info.angle_min;
            a_index = 0;
        } else {
            a_index = (unsigned int)(floor((45.0 / 180.0 * PI - lidar_info.angle_min) / lidar_info.angle_increment));
        }

        double a_range = 0.0;
        double b_range = 0.0;

        // 获取 a 点的有效距离，若无效则赋默认值 100
        if (!std::isinf(lidar_info.ranges[a_index]) && !std::isnan(lidar_info.ranges[a_index])) {
            a_range = lidar_info.ranges[a_index];
        } else {
            a_range = 100.0;
        }

        // 获取 b 点的有效距离，若无效则赋默认值 100
        if (!std::isinf(lidar_info.ranges[b_index]) && !std::isnan(lidar_info.ranges[b_index])) {
            b_range = lidar_info.ranges[b_index];
        } else {
            b_range = 100.0;
        }

        // 3. 计算几何关系，用于推算机器人到墙的垂直距离及朝向
        double alpha = atan((a_range * cos(b_angle - a_angle) - b_range) / (a_range * sin(b_angle - a_angle)));  // alpha角，用于描述车体偏转角度
        double AB = b_range * cos(alpha);                        // 车体当前与墙之间的实际距离（垂直距离）
        double projected_dis = AB + LOOK_AHEAD_DIS * sin(alpha); // 前瞻预测距离（考虑车辆运动延迟）
        error = DESIRED_DISTANCE_RIGHT - projected_dis;          // 与期望墙距的误差

        // 输出调试信息
        ROS_INFO("projected_dis = %f", projected_dis);
        ROS_INFO("error = %f", error);
        ROS_INFO("del_time = %f\n", del_time);

        SubscribeAndPublish::pid_control(); // 执行 PID 控制
    }

    // 4. PID 控制器：根据误差计算转角
    void pid_control() {
        ackermann_msgs::AckermannDriveStamped ackermann_drive_result;
        double tmoment = ros::Time::now().toSec();           // 当前时间戳（秒）
        del_time = tmoment - prev_tmoment;                   // 当前与上一帧之间的时间差
        integral += prev_error * del_time;                   // 误差积分
        ackermann_drive_result.drive.steering_angle =
            -(KP * error + KD * (error - prev_error) / del_time + KI * integral); // PID 控制公式

        prev_tmoment = tmoment;                              // 更新时间戳
        prev_error = error;                                  // 保存当前误差作为下一次的前一误差

        // 根据转向角动态调整小车速度（转弯越大，速度越慢）
        if (abs(ackermann_drive_result.drive.steering_angle) > 20.0 / 180.0 * PI) {
            ackermann_drive_result.drive.speed = 2.0;
        } else if (abs(ackermann_drive_result.drive.steering_angle) > 10.0 / 180.0 * PI) {
            ackermann_drive_result.drive.speed = 3.0;
        } else {
            ackermann_drive_result.drive.speed = 5.0;
        }

        drive_pub.publish(ackermann_drive_result); // 发布控制指令
    }

private:
    ros::NodeHandle nh;                         // ROS 节点句柄
    ros::Publisher drive_pub;                   // 控制指令发布器
    ros::Subscriber scan_sub;                   // 激光数据订阅器

    double prev_error = 0.0;                    // 上一次误差
    double prev_tmoment = ros::Time::now().toSec();  // 上一帧时间
    double error = 0.0;                          // 当前误差
    double integral = 0.0;                       // 积分累加项
    double speed = 0.0;                          // 当前车速
    double del_time = 0.0;                       // 时间差
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_following_pid");  // 初始化节点
    SubscribeAndPublish SAPObject;                // 创建控制类实例
    ros::spin();                                  // 进入循环，等待回调
    return 0;
}

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <thread>
#include <vector>
#include <chrono>

class RotateWheelNode : public rclcpp::Node {
public:
    RotateWheelNode() : Node("rotate_fishbot_wheel"),
                        joint_speeds_{0.0, 0.0},
                        joint_positions_{0.0, 0.0},
                        running_(true)
    {
        RCLCPP_INFO(this->get_logger(), "node rotate_fishbot_wheel init..");

        // 创建 publisher，发布 JointState 消息到 "joint_states" 主题
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // 初始化消息
        joint_state_msg_.name = {"left_wheel_joint", "right_wheel_joint"};
        joint_state_msg_.position = joint_positions_;
        joint_state_msg_.velocity = joint_speeds_;
        joint_state_msg_.effort = {};

        // 启动线程定时发布 joint_states
        pub_thread_ = std::thread(&RotateWheelNode::publish_loop, this);
    }

    ~RotateWheelNode()
    {
        running_ = false;
        if (pub_thread_.joinable())
            pub_thread_.join();
    }

    void update_speed(const std::vector<double>& speeds)
    {
        joint_speeds_ = speeds;
    }

private:
    void publish_loop() {
        rclcpp::Rate rate(30);  // 30 Hz
        auto last_time = std::chrono::steady_clock::now();

        while (rclcpp::ok() && running_) {
            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - last_time).count();
            last_time = now;

            // 积分更新两个轮子的角度（位置）
            joint_positions_[0] += dt * joint_speeds_[0];
            joint_positions_[1] += dt * joint_speeds_[1];

            // 更新消息内容
            joint_state_msg_.header.stamp = this->get_clock()->now();
            joint_state_msg_.position = joint_positions_;
            joint_state_msg_.velocity = joint_speeds_;
            joint_state_msg_.effort.clear();  // 不使用 effort

            // 发布 joint state
            joint_state_publisher_->publish(joint_state_msg_);

            rate.sleep();
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    sensor_msgs::msg::JointState joint_state_msg_;

    std::vector<double> joint_speeds_;
    std::vector<double> joint_positions_;

    std::thread pub_thread_;
    bool running_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // 创建 RotateWheelNode 节点的共享指针实例
    auto node = std::make_shared<RotateWheelNode>();
    node->update_speed({15.0, -15.0});  // 左轮正转15 rad/s，右轮反转15 rad/s

    // 进入ROS 2的事件循环，处理回调（此处用于保持节点运行）
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

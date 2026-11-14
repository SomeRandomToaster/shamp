#ifndef ARTHROPOD_CONTROLLER_H
#define ARTHROPOD_CONTROLLER_H

#include <shamp/arthropod_base/arthropod_components.h>
#include <shamp/common/common_components.h>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class ArthropodController : public rclcpp::Node
{
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cmd_pose_subscription_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_commands_publisher_;

    rclcpp::TimerBase::SharedPtr loop_timer_;

    champ::Velocities req_vel_;
    champ::Pose req_pose_;

    shamp::ArthropodGaitConfig gait_config_;

    bool publish_joint_control_;

    void controlLoop_();

    void cmdVelCallback_(const geometry_msgs::msg::Twist::SharedPtr msg);
    void cmdPoseCallback_(const geometry_msgs::msg::Pose::SharedPtr msg);
public:
    ArthropodController();
};

#endif
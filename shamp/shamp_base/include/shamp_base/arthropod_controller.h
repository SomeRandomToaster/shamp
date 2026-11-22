#ifndef ARTHROPOD_CONTROLLER_H
#define ARTHROPOD_CONTROLLER_H

#include "arthropod_base/arthropod_base.h"
#include "arthropod_base/arthropod_components.h"
#include "body_controller/arthropod_body_controller.h"
#include "common/common_components.h"
#include "kinematics/arthropod_kinematics.h"
#include "leg_controller/arthropod_leg_controller.h"

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

class ArthropodController : public rclcpp::Node
{
    constexpr static char kLoggerPrefix[] = "ArthropodController";

    unsigned n_legs_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cmd_pose_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_stan_rad_subscription_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_commands_publisher_;

    rclcpp::TimerBase::SharedPtr loop_timer_;
    rclcpp::Clock clock_;

    champ::Velocities req_vel_;
    champ::Pose req_pose_;
    
    shamp::ArthropodGaitConfig gait_config_;

    shamp::ArthropodBase base_;
    shamp::ArthropodBodyController body_controller_;
    shamp::ArthropodLegController leg_controller_;
    shamp::ArthropodKinematics kinematics_;

    std::vector<geometry::Transformation> target_foot_positions_;
    std::vector<double> target_joint_positions_;

    bool publish_joint_control_;


    void controlLoop_();

    void cmdVelCallback_(const geometry_msgs::msg::Twist::SharedPtr msg);
    void cmdPoseCallback_(const geometry_msgs::msg::Pose::SharedPtr msg);
    void cmdStanRadCallback_(const std_msgs::msg::Float64::SharedPtr msg);

    void publishJoints_();
public:
    ArthropodController();
};

#endif
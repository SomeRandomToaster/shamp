#include "arthropod_base/arthropod_controller.h"

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>

#include <functional>

ArthropodController::ArthropodController():
    Node(
        "quadruped_controller_node",
            rclcpp::NodeOptions()
                .allow_undeclared_parameters(true)
                .automatically_declare_parameters_from_overrides(true))
{
    std::string joint_control_topic = "servo_controller/commands";
    std::string urdf = "";

    double loop_rate = 50.0;

    this->get_parameter("gait.max_linear_velocity_x",  gait_config_.max_linear_velocity_x);
    this->get_parameter("gait.max_linear_velocity_y",  gait_config_.max_linear_velocity_y);
    this->get_parameter("gait.max_angular_velocity_z", gait_config_.max_angular_velocity_z);
    this->get_parameter("gait.com_x_translation",      gait_config_.com_x_translation);
    this->get_parameter("gait.swing_height",           gait_config_.swing_height);
    this->get_parameter("gait.stance_depth",           gait_config_.stance_depth);
    this->get_parameter("gait.stance_duration",        gait_config_.stance_duration);
    this->get_parameter("gait.nominal_height",         gait_config_.nominal_height);
    this->get_parameter("publish_joint_control",       publish_joint_control_);
    this->get_parameter("joint_controller_topic",      joint_control_topic);
    this->get_parameter("loop_rate",                   loop_rate);
    this->get_parameter("urdf",                        urdf);

    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&ArthropodController::cmdVelCallback_, this,  std::placeholders::_1));
    cmd_pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "cmd_pose", 1,  std::bind(&ArthropodController::cmdPoseCallback_, this,  std::placeholders::_1));

    if(publish_joint_control_)
    {
        joint_commands_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(joint_control_topic, 10);
    }

    std::chrono::milliseconds period(static_cast<int>(1000/loop_rate));
    loop_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period), std::bind(&ArthropodController::controlLoop_, this));
    
    req_pose_.position.z = gait_config_.nominal_height;
}

void ArthropodController::cmdVelCallback_(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    req_vel_.linear.x = msg->linear.x;
    req_vel_.linear.y = msg->linear.y;
    req_vel_.angular.z = msg->angular.z;
}

void ArthropodController::cmdPoseCallback_(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    tf2::Quaternion quat(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    req_pose_.orientation.roll = roll;
    req_pose_.orientation.pitch = pitch;
    req_pose_.orientation.yaw = yaw;

    req_pose_.position.x = msg->position.x;
    req_pose_.position.y = msg->position.y;
    req_pose_.position.z = msg->position.z +  gait_config_.nominal_height;
}

void ArthropodController::controlLoop_()
{
    RCLCPP_INFO(rclcpp::get_logger("HexapopController"), "test");
}

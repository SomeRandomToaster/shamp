#include "shamp_base/arthropod_controller.h"

#include <shamp/utils/urdf_loader.h>

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>

#include <functional>

shamp::ArthropodPhaseGenerator::Time rosTimeToChampTime(const rclcpp::Time& time)
{
  return time.nanoseconds() / 1000ul;
}

ArthropodController::ArthropodController():
    Node(
        "arthropod_controller_node",
            rclcpp::NodeOptions()
                .allow_undeclared_parameters(true)
                .automatically_declare_parameters_from_overrides(true)),
    n_legs_((this->get_parameter("n_legs", n_legs_), n_legs_)),
    clock_(*this->get_clock()),
    base_(n_legs_),
    body_controller_(base_),
    leg_controller_(base_),
    kinematics_(base_),
    target_foot_positions_(n_legs_),
    target_joint_positions_(n_legs_ * (base_.legs[0].joint_chain.size() - 1)),
    publish_joint_control_(true)
{
    std::string joint_control_topic = "servo_controller/commands";
    std::string urdf = "";

    double loop_rate = 50.0;

    this->get_parameter("gait.max_linear_velocity_x",  gait_config_.max_linear_velocity_x);
    this->get_parameter("gait.max_linear_velocity_y",  gait_config_.max_linear_velocity_y);
    this->get_parameter("gait.max_angular_velocity_z", gait_config_.max_angular_velocity_z);
    this->get_parameter("gait.com_x_translation",      gait_config_.com_x_translation);
    this->get_parameter("gait.swing_height",           gait_config_.swing_height);
    this->get_parameter("gait.swing_duration",         gait_config_.swing_duration);
    this->get_parameter("gait.stance_depth",           gait_config_.stance_depth);
    this->get_parameter("gait.stance_radius",          gait_config_.stance_radius);
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
    cmd_stan_rad_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "cmd_stan_rad", 1,  std::bind(&ArthropodController::cmdStanRadCallback_, this,  std::placeholders::_1));

    if(publish_joint_control_)
    {
        joint_commands_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(joint_control_topic, 10);
    }

    base_.setGaitConfig(&gait_config_);
    shamp::URDF::loadFromString(base_, this->get_node_parameters_interface(), urdf);

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
    body_controller_.poseCommand(target_foot_positions_, req_pose_);
    leg_controller_.velocityCommand(target_foot_positions_, req_vel_, rosTimeToChampTime(clock_.now()));
    kinematics_.inverse(target_joint_positions_, target_foot_positions_);
    publishJoints_();
}

void ArthropodController::publishJoints_()
{
    if(publish_joint_control_)
    {
        auto message = std_msgs::msg::Float64MultiArray();
        message.data = target_joint_positions_;
        joint_commands_publisher_->publish(message);
    }
}

 void ArthropodController::cmdStanRadCallback_(const std_msgs::msg::Float64::SharedPtr msg)
 {
    gait_config_.stance_radius = msg->data;
 }
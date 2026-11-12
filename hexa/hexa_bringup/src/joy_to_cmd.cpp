#include "hexa_bringup/joy_to_cmd.h"

JoyToCommandNode* JoyToCommandNode::instance_ = nullptr;


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToCommandNode>());
  rclcpp::shutdown();
  return 0;
}


JoyToCommandNode::JoyToCommandNode()
    : Node("joy_to_cmd")
{
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("servo_controller/commands", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, TopicCallback);
    instance_ = this;
}


void JoyToCommandNode::TopicCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    auto message = std_msgs::msg::Float64MultiArray();
    double coxa_angle = -M_PI_2 * msg->axes[kCoxaAxis];
    double tibia_angle = M_PI_2 * msg->axes[kTibiaAxis];
    double femur_angle = M_PI_2 * msg->axes[kFemurAxis];
    tibia_angle = tibia_angle < kTibiaMin ? kTibiaMin : tibia_angle;
    femur_angle = femur_angle > kFemurMax ? kFemurMax : femur_angle;

    for (size_t i = 0; i < kLegCount; i++) {
        message.data.push_back(coxa_angle);
        message.data.push_back(tibia_angle);
        message.data.push_back(femur_angle);
    }

    instance_->publisher_->publish(message);
}
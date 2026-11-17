#include "hexa_bringup/joy_to_pose.h"

#include <tf2/LinearMath/Quaternion.hpp>

JoyToPoseNode* JoyToPoseNode::instance_ = nullptr;


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToPoseNode>());
  rclcpp::shutdown();
  return 0;
}


JoyToPoseNode::JoyToPoseNode()
    : Node("joy_to_pose_node")
{
    publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("cmd_pose", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, TopicCallback);
    instance_ = this;
}


void JoyToPoseNode::TopicCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    auto message = geometry_msgs::msg::Pose();
    double y_offset = kMaxOffset * msg->axes[kLeftStickX];
    double z_offset = kMaxOffset * msg->axes[kLeftStickY];

    double roll = -kMaxAngleXY * msg->axes[kRightStickX];
    double pitch = kMaxAngleXY * msg->axes[kRightStickY];
    double yaw = kMaxAngleZ * 0.5 * (msg->axes[kLeftTrigger] - msg->axes[kRightTrigger]);

    message.position.y = y_offset;
    message.position.z = z_offset;
    
    tf2::Quaternion rotation = tf2::Quaternion::createFromRPY(roll, pitch, yaw);

    message.orientation.x = rotation.x();
    message.orientation.y = rotation.y();
    message.orientation.z = rotation.z();
    message.orientation.w = rotation.w();

    instance_->publisher_->publish(message);
}
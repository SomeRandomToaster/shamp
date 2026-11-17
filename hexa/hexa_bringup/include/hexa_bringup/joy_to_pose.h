#ifndef JOY_TO_POSE_H
#define JOY_TO_POSE_H

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <functional>
#include <cmath>

class JoyToPoseNode : public rclcpp::Node
{
    static constexpr size_t kLeftStickX = 0;
    static constexpr size_t kLeftStickY = 1;
    static constexpr size_t kLeftTrigger = 2;
    static constexpr size_t kRightStickX = 3;
    static constexpr size_t kRightStickY = 4;
    static constexpr size_t kRightTrigger = 5;

    static constexpr double kMaxAngleXY = M_PI / 36;
    static constexpr double kMaxAngleZ = M_PI / 12;
    static constexpr double kMaxOffset = 0.05;

  public:
    JoyToPoseNode();
  private:
    static void TopicCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    static JoyToPoseNode* instance_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

#endif
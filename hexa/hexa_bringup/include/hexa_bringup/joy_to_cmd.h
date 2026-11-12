#ifndef JOY_TO_CMD_H
#define JOY_TO_CMD_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <functional>
#include <cmath>

class JoyToCommandNode : public rclcpp::Node
{
    static constexpr size_t kLegCount = 6;
    static constexpr size_t kCoxaAxis = 0;
    static constexpr size_t kTibiaAxis = 1;
    static constexpr size_t kFemurAxis = 4;
    static constexpr double kTibiaMin = -M_PI_4;
    static constexpr double kFemurMax = M_PI / 3;
  public:
    JoyToCommandNode();
  private:
    static void TopicCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    static JoyToCommandNode* instance_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

#endif
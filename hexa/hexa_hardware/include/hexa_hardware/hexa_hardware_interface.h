#pragma once

#include "hexa_hardware/leg.h"

#include <hexa_msgs/msg/hexa_servo_pos.hpp>

#include <hardware_interface/system_interface.hpp>

namespace hexa_hardware
{
    class HexaHardwareInterface : public hardware_interface::SystemInterface
    {
        constexpr static size_t kLegsCount = 6;
        constexpr static size_t kJointsPerLeg = 3;
        constexpr static size_t kJointCount = kLegsCount * kJointsPerLeg;

        static constexpr float kFemurFrameAngle = 0;//8.7;
        // Default servo bias values
        static constexpr float kCoxaBias = 90;
        static constexpr float kTibiaBias = 90;
        static constexpr float kFemurBias = 90 - kFemurFrameAngle;
    public:
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    private:
        std::array<double, kJointCount> position_command;
        std::array<double, kJointCount> position_state;
        rclcpp::Publisher<hexa_msgs::msg::HexaServoPos>::SharedPtr publisher;

        std::vector<Leg> legs = {
            kLegsCount, 
            {
                {kCoxaBias, true},
                {kTibiaBias, true},
                {kFemurBias, false}
            }
        };
    };
}
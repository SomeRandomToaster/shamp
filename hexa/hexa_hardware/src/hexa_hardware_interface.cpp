#include "hexa_hardware/hexa_hardware_interface.h"

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <unordered_map>

namespace hexa_hardware
{
    hardware_interface::CallbackReturn HexaHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
    {
        if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        std::string topic_name;
        auto it = params.hardware_info.hardware_parameters.find("topic_name");
        if (it == params.hardware_info.hardware_parameters.cend()) {
            RCLCPP_ERROR(rclcpp::get_logger("HexaHardwareInterface"), "No output topic was provided. Can't proceed.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        topic_name = it->second;
        RCLCPP_INFO(rclcpp::get_logger("HexaHardwareInterface"), "Starting publisher on topic \"%s\"", topic_name.c_str());

        if (!get_node()) {
            RCLCPP_ERROR(rclcpp::get_logger("HexaHardwareInterface"), "Unable to get the base node. Can't proceed.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        publisher = get_node()->create_publisher<hexa_msgs::msg::HexaServoPos>(topic_name, 10);

        return hardware_interface::CallbackReturn::SUCCESS;
    }
    
    hardware_interface::CallbackReturn HexaHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        (void)previous_state;

        RCLCPP_INFO(rclcpp::get_logger("HexaHardwareInterface"), "Activating hardware interface");

        hexa_msgs::msg::HexaServoPos msg;

        for (size_t i = 0; i < msg.leg.size(); i++) {
            auto servo_angles = legs[i].JointAnglesToServoAngles({0, 0, 0});
            msg.leg[i].coxa_angle = servo_angles.x();
            msg.leg[i].tibia_angle = servo_angles.y();
            msg.leg[i].femur_angle = servo_angles.z();
        }
        publisher->publish(msg);
        RCLCPP_INFO(rclcpp::get_logger("HexaHardwareInterface"), "Setting initial servo positions");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HexaHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        (void)previous_state;

        RCLCPP_INFO(rclcpp::get_logger("HexaHardwareInterface"), "Deactivating hardware interface");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> HexaHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_state[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> HexaHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_command[i]));
        }
        return command_interfaces;
    }

    hardware_interface::return_type HexaHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        (void)time;
        (void)period;

        for (size_t i = 0; i < position_state.size(); i++) {
            position_state[i] = position_command[i];
        }
        return hardware_interface::return_type::OK;
    }
    hardware_interface::return_type HexaHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        (void)time;
        (void)period;

        hexa_msgs::msg::HexaServoPos msg;

        for (size_t i = 0; i < msg.leg.size(); i++) {
            tf2::Vector3 joint_angles = {
                position_command[kJointsPerLeg * i],
                position_command[kJointsPerLeg * i + 1],
                position_command[kJointsPerLeg * i + 2]
            };
            joint_angles *= 180 / M_PI;

            auto servo_angles = legs[i].JointAnglesToServoAngles(joint_angles);
            msg.leg[i].coxa_angle = servo_angles.x();
            msg.leg[i].tibia_angle = servo_angles.y();
            msg.leg[i].femur_angle = servo_angles.z();
        }
        publisher->publish(msg);
        return hardware_interface::return_type::OK;
    }
}

// Register hardware interface as plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  hexa_hardware::HexaHardwareInterface, hardware_interface::SystemInterface)
#ifndef BRC_ARM_HARDWARE_INTERFACE
#define BRC_ARM_HARDWARE_INTERFACE
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <memory>
#include <string>
#include <vector>
#include <iostream>

namespace brc_arm_hardware_interface {

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

class BRCArmHWInterface : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(BRCArmHWInterface)

    // LifecycleNodeInterface
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state);
    // CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state);
    // CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state);
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state);
    CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state);

    // SystemInterface overrides
    /// Initialization of the hardware interface from data parsed from the robot's URDF.
    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    /// Exports all state interfaces for this hardware interface.
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    /// Exports all state interfaces for this hardware interface.
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    /// Read the current state values from the actuator.
    return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    /// Write the current command values to the actuator.
    return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // Parameters for the robot simulation
    double hw_start_sec_;
    double hw_stop_sec_;
    double hw_slowdown_;

    // Store the command for the simulated robot
    std::vector<std::vector<double>> hw_commands_;
    std::vector<std::vector<double>> hw_states_;

    const size_t POSITION_INTERFACE_INDEX = 0;
    const size_t VELOCITY_INTERFACE_INDEX = 1;
};

}

#endif /* BRC_ARM_HARDWARE_INTERFACE */

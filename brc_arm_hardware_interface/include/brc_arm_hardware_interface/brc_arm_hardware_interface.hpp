#ifndef BRC_ARM_HARDWARE_INTERFACE
#define BRC_ARM_HARDWARE_INTERFACE
#include "hardware_interface/system_interface.hpp"
#include "mock_components/generic_system.hpp"
#include <vector>
#include <iostream>

namespace brc_arm_hardware_interface {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class BRCArmHWInterface : public mock_components::GenericSystem {
public:
    /// Read the current state values from the actuator.
    //return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    /// Write the current command values to the actuator.
    return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override {
        std::cout << "return" << std::endl;
        return return_type::OK;
    };
};

/*class BRCArmHWInterface : public hardware_interface::SystemInterface {
public:
    BRCArmHWInterface();

    // LifecycleNodeInterface
    CallbackReturn on_configure(const State& previous_state);
    CallbackReturn on_cleanup(const State& previous_state);
    CallbackReturn on_shutdown(const State& previous_state);
    CallbackReturn on_activate(const State& previous_state);
    CallbackReturn on_deactivate(const State& previous_state);
    CallbackReturn on_error(const State& previous_state);

    // SystemInterface overrides
    /// Initialization of the hardware interface from data parsed from the robot's URDF.
    CallbackReturn on_init(const HardwareInfo& hardware_info) override;

    /// Exports all state interfaces for this hardware interface.
    std::vector<StateInterface> export_state_interfaces() override;

    /// Exports all state interfaces for this hardware interface.
    std::vector<CommandInterface> export_command_interfaces() override;

    /// Read the current state values from the actuator.
    return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    /// Write the current command values to the actuator.
    return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
}*/

}

#endif /* BRC_ARM_HARDWARE_INTERFACE */

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "brc_arm_hardware_interface/brc_arm_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace brc_arm_hardware_interface {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn BRCArmHWInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // RCLCPP_INFO(
  //   rclcpp::get_logger("BRCArmHWInterface"), "Init ...please wait...");

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  // hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  // hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // hw_slowdown_ = std::stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = 0;
  hw_stop_sec_ = 3.0;
  hw_slowdown_ = 10;

  // Position, velocity interfaces
  hw_states_.resize(2);
  hw_commands_.resize(2);
  for (auto i = 0u; i < 2; i++) {
    hw_states_[i].resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_[i].resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  }

  // Check expected interface types for each joint (position, velocity)
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("BRCArmHWInterface"),
        "Joint '%s' has %zu command interfaces found. 2 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("BRCArmHWInterface"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("BRCArmHWInterface"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("BRCArmHWInterface"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("BRCArmHWInterface"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("BRCArmHWInterface"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

CallbackReturn BRCArmHWInterface::on_configure(const rclcpp_lifecycle::State& /* previous_state */) {
  // RCLCPP_INFO(
  //   rclcpp::get_logger("BRCArmHWInterface"), "Configuring ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
  //   RCLCPP_INFO(
  //     rclcpp::get_logger("BRCArmHWInterface"), "%.1f seconds left...",
  //     hw_start_sec_ - i);
  }

  // Reset values always when configuring hardware
  for (uint i = 0; i < hw_states_.size(); i++) {
    for (uint j = 0; j < hw_states_[i].size(); j++) {
      hw_states_[i][j] = 0;
      hw_commands_[i][j] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("BRCArmHWInterface"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// CallbackReturn BRCArmHWInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
//
// }
//
// CallbackReturn BRCArmHWInterface::on_shutdown(const rclcpp_lifecycle::State& previous_state) {
//
// }

CallbackReturn BRCArmHWInterface::on_activate(const rclcpp_lifecycle::State& /* previous_state */) {
  RCLCPP_INFO(
    rclcpp::get_logger("BRCArmHWInterface"), "Activating ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    // RCLCPP_INFO(
    //   rclcpp::get_logger("BRCArmHWInterface"), "%.1f seconds left...",
    //   hw_start_sec_ - i);
  }

  // Command and state should be equal when starting
  for (uint i = 0; i < hw_states_.size(); i++) {
    for (uint j = 0; j < hw_states_[i].size(); j++) {
      hw_commands_[i][j] = hw_states_[i][j];
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("BRCArmHWInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

CallbackReturn BRCArmHWInterface::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */) {
  RCLCPP_INFO(
    rclcpp::get_logger("BRCArmHWInterface"), "Deactivating ...please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    // RCLCPP_INFO(
    //   rclcpp::get_logger("BRCArmHWInterface"), "%.1f seconds left...",
    //   hw_stop_sec_ - i);
  }

  RCLCPP_INFO(rclcpp::get_logger("BRCArmHWInterface"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

CallbackReturn BRCArmHWInterface::on_error(const rclcpp_lifecycle::State& /* previous_state */) {
  return hardware_interface::CallbackReturn::ERROR;
}

std::vector<hardware_interface::StateInterface> BRCArmHWInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint j = 0; j < info_.joints.size(); j++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[j].name, hardware_interface::HW_IF_POSITION, &hw_states_[POSITION_INTERFACE_INDEX][j]));
    RCLCPP_INFO(rclcpp::get_logger("state interfaces"), "interface: %s added", info_.joints[j].name.c_str());

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[j].name, hardware_interface::HW_IF_VELOCITY, &hw_states_[VELOCITY_INTERFACE_INDEX][j]));
    RCLCPP_INFO(rclcpp::get_logger("state interfaces"), "interface: %s added", info_.joints[j].name.c_str());
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BRCArmHWInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint j = 0; j < info_.joints.size(); j++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[j].name, hardware_interface::HW_IF_POSITION, &hw_commands_[POSITION_INTERFACE_INDEX][j]));
    RCLCPP_INFO(rclcpp::get_logger("state interfaces"), "interface: %s added", info_.joints[j].name.c_str());

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[j].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[VELOCITY_INTERFACE_INDEX][j]));
    RCLCPP_INFO(rclcpp::get_logger("state interfaces"), "interface: %s added", info_.joints[j].name.c_str());
  }

  return command_interfaces;
}

return_type BRCArmHWInterface::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */) {
  auto mirror_command_to_state = [](auto & states_, auto commands_, size_t start_index = 0)
  {
    for (size_t i = start_index; i < states_.size(); ++i)
    {
      for (size_t j = 0; j < states_[i].size(); ++j)
      {
        if (!std::isnan(commands_[i][j]))
        {
          states_[i][j] = commands_[i][j];
        }
      }
    }
  };

  for (uint j = 0; j < hw_states_[POSITION_INTERFACE_INDEX].size(); j++)
  {
    // Simulate movement
    hw_states_[POSITION_INTERFACE_INDEX][j] = hw_states_[POSITION_INTERFACE_INDEX][j] +
      (hw_commands_[POSITION_INTERFACE_INDEX][j] - hw_states_[POSITION_INTERFACE_INDEX][j]) / hw_slowdown_;
    // RCLCPP_INFO(
    //   rclcpp::get_logger("BRCArmHWInterface"), "Got state %.5f for joint %d!",
    //   hw_states_[i], i);
  }

  // Mirror remaining interface types (velocity)
  mirror_command_to_state(hw_states_, hw_commands_, 1);

  return hardware_interface::return_type::OK;
}

return_type BRCArmHWInterface::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */) {
  for (uint i = 0; i < hw_commands_[POSITION_INTERFACE_INDEX].size(); i++)
  {
    // Simulate sending commands to the hardware
    // RCLCPP_INFO(
    //   rclcpp::get_logger("BRCArmHWInterface"), "Got command %.5f for joint %d!",
    //   hw_commands_[i], i);
  }

  return hardware_interface::return_type::OK;
}

}

PLUGINLIB_EXPORT_CLASS(brc_arm_hardware_interface::BRCArmHWInterface, hardware_interface::SystemInterface)

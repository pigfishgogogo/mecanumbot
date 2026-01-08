#pragma once
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "mecanumbot_hardware/motor_controller.hpp"

// 添加包含 pluginlib 宏
#include <pluginlib/class_list_macros.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace mecanumbot_hardware
{
class MecanumBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MecanumBotSystemHardware)

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Motor controllers for each wheel
  std::vector<std::unique_ptr<MotorController>> motor_controllers_;
  
  // CAN interface
  std::string can_interface_;
  uint8_t node_id_base_;
  
  // Store the command and state for each joint
  std::vector<double> velocity_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  // std::vector<double> effort_states_;
  
  // Joint names
  std::vector<std::string> joint_names_;
  
  // Wheel parameters
  std::vector<double> wheel_radius_;
  std::vector<double> wheel_separation_x_;
  std::vector<double> wheel_separation_y_;
  
  // Conversion factors
  double rpm_to_rad_per_sec_;
  double rad_per_sec_to_rpm_;
  
  // Initialize motor controllers
  bool initialize_motors();
  
  // Convert wheel velocities to motor RPM
  double wheel_velocity_to_rpm(double wheel_velocity, double wheel_radius);
  double rpm_to_wheel_velocity(double rpm, double wheel_radius);
  
  // Mecanum kinematics
  void calculate_mecanum_kinematics(
    const double linear_x, const double linear_y, const double angular_z,
    std::vector<double> & wheel_velocities);
  
  void calculate_inverse_kinematics(
    const std::vector<double> & wheel_velocities,
    double & linear_x, double & linear_y, double & angular_z);
};
}  // namespace mecanumbot_hardware


// 添加在头文件末尾添加插件导出宏 命令空间:: 类名或 基类名
PLUGINLIB_EXPORT_CLASS(
  mecanumbot_hardware::MecanumBotSystemHardware, 
  hardware_interface::SystemInterface
)

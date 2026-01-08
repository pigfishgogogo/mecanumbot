#include "mecanumbot_hardware/mecanumbot_hardware.hpp"
#include <cmath>
#include <limits>
#include <vector>
#include <iostream>

namespace mecanumbot_hardware
{

CallbackReturn MecanumBotSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  std::cout << "Initializing MecanumBotSystemHardware..." << std::endl;

  // Initialize default values
  velocity_commands_.resize(info_.joints.size(), 0.0);
  position_states_.resize(info_.joints.size(), 0.0);
  velocity_states_.resize(info_.joints.size(), 0.0);
  // effort_states_.resize(info_.joints.size(), 0.0);
  
  joint_names_.resize(info_.joints.size());
  wheel_radius_.resize(info_.joints.size(), 0.05);  // default 5cm radius
  wheel_separation_x_.resize(info_.joints.size(), 0.3); // default 30cm
  wheel_separation_y_.resize(info_.joints.size(), 0.2); // default 20cm
  motor_controllers_.resize(info_.joints.size());
  
  // Conversion factors
  rpm_to_rad_per_sec_ = 2.0 * M_PI / 60.0;  // RPM to rad/s
  rad_per_sec_to_rpm_ = 60.0 / (2.0 * M_PI); // rad/s to RPM

  // Get parameters from URDF
  can_interface_ = info_.hardware_parameters["can_interface"];
  node_id_base_ = std::stoi(info_.hardware_parameters["node_id_base"]);
  
  // Initialize each joint
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    joint_names_[i] = info_.joints[i].name;
    
    // Get wheel radius from parameters if specified
    auto it = info_.joints[i].parameters.find("wheel_radius");
    if (it != info_.joints[i].parameters.end())
    {
      wheel_radius_[i] = std::stod(it->second);
    }
    
    std::cout << "Joint " << i << ": " << joint_names_[i] << ", Wheel radius: " << wheel_radius_[i] << std::endl;
  }

  // Initialize motor controllers
  if (!initialize_motors())
  {
    std::cerr << "Failed to initialize motor controllers" << std::endl;
    return CallbackReturn::ERROR;
  }

  std::cout << "MecanumBotSystemHardware initialized successfully" << std::endl;
  return CallbackReturn::SUCCESS;
}

bool MecanumBotSystemHardware::initialize_motors()
{
  std::cout << "Initializing motor controllers..." << std::endl;
  
  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    uint8_t node_id = node_id_base_ + static_cast<uint8_t>(i);
    
    std::cout << "Initializing motor for joint " << joint_names_[i] 
              << " with node ID: " << static_cast<int>(node_id) << std::endl;
    
    motor_controllers_[i] = std::make_unique<MotorController>(node_id);
    
    if (!motor_controllers_[i]->initialize(can_interface_))
    {
      std::cerr << "Failed to initialize motor controller for joint " 
                << joint_names_[i] << std::endl;
      return false;
    }
    
    // Enable motor
    if (!motor_controllers_[i]->enableMotor())
    {
      std::cerr << "Failed to enable motor for joint " << joint_names_[i] << std::endl;
      return false;
    }
    
    // Set to velocity mode
    if (!motor_controllers_[i]->setModeOfOperation(3))  // 3 = velocity mode
    {
      std::cerr << "Failed to set velocity mode for joint " << joint_names_[i] << std::endl;
      return false;
    }
  }
  
  std::cout << "All motor controllers initialized successfully" << std::endl;
  return true;
}

std::vector<hardware_interface::StateInterface> MecanumBotSystemHardware::export_state_interfaces()
{
  // 告诉ros2控制器管理器可以读取哪些状态（位置、速度、 力矩）
  std::vector<hardware_interface::StateInterface> state_interfaces;
  // 为每个关节导出状态接口
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &position_states_[i]));
    
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &velocity_states_[i]));
    
    // state_interfaces.emplace_back(
    //   hardware_interface::StateInterface(
    //     info_.joints[i].name,
    //     hardware_interface::HW_IF_EFFORT,
    //     &effort_states_[i]));
  }
  
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MecanumBotSystemHardware::export_command_interfaces()
{
  // 告诉ros2控制器管理器可以写入哪些命令（速度）
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // 为每个关节导出命令接口
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &velocity_commands_[i]));
  }
  
  return command_interfaces;
}

CallbackReturn MecanumBotSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::cout << "Activating MecanumBotSystemHardware..." << std::endl;
  
  // Reset commands and states
  std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
  std::fill(position_states_.begin(), position_states_.end(), 0.0);
  std::fill(velocity_states_.begin(), velocity_states_.end(), 0.0);
  // std::fill(effort_states_.begin(), effort_states_.end(), 0.0);
  
  // Send zero velocity command to all motors
  for (size_t i = 0; i < motor_controllers_.size(); i++)
  {
    if (!motor_controllers_[i]->setVelocityZero())
    {
      std::cerr << "Failed to set zero velocity for joint " << joint_names_[i] << std::endl;
      return CallbackReturn::ERROR;
    }
  }
  
  std::cout << "MecanumBotSystemHardware activated successfully" << std::endl;
  return CallbackReturn::SUCCESS;
}

CallbackReturn MecanumBotSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::cout << "Deactivating MecanumBotSystemHardware..." << std::endl;
  
  // Send zero velocity command to all motors
  for (size_t i = 0; i < motor_controllers_.size(); i++)
  {
    motor_controllers_[i]->setVelocityZero();
    motor_controllers_[i]->disableMotor();
  }
  
  std::cout << "MecanumBotSystemHardware deactivated" << std::endl;
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type MecanumBotSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read actual velocity from motors and update states
  for (size_t i = 0; i < motor_controllers_.size(); i++)
  {
    int32_t can_velocity;
    if (motor_controllers_[i]->getActualVelocity(can_velocity))
    {
      // Convert CAN velocity to RPM
      double rpm = static_cast<double>(can_velocity) * 1875.0 / 
                   (512.0 * motor_controllers_[i]->getEncoderResolution());
      
      // Convert RPM to rad/s for state interface
      velocity_states_[i] = rpm * rpm_to_rad_per_sec_; 
      
      // Integrate velocity to get position (simplified)
      position_states_[i] += velocity_states_[i] * 0.01;  // assuming 100Hz update
    }
    // 此外还可以读取力矩等其他状态，读取的的数据可存入 effort_states_[i]. 可选
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MecanumBotSystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 根据命令接口（velocity_commands_中的） 速度命令 发送给电机
  for (size_t i = 0; i < motor_controllers_.size(); i++)
  {
    // Convert rad/s to RPM
    double rpm = velocity_commands_[i] * rad_per_sec_to_rpm_;
    
    // Send RPM command to motor
    if (!motor_controllers_[i]->setTargetVelocity(static_cast<int32_t>(rpm)))
    {
      std::cerr << "Failed to set velocity for joint " << joint_names_[i] << std::endl;
      return hardware_interface::return_type::ERROR;
    }
  }
  
  return hardware_interface::return_type::OK;
}

double MecanumBotSystemHardware::wheel_velocity_to_rpm(double wheel_velocity, double wheel_radius)
{
  // Convert wheel linear velocity (m/s) to angular velocity (rad/s)
  double wheel_angular_velocity = wheel_velocity / wheel_radius;
  
  // Convert rad/s to RPM
  return wheel_angular_velocity * rad_per_sec_to_rpm_;
}

double MecanumBotSystemHardware::rpm_to_wheel_velocity(double rpm, double wheel_radius)
{
  // Convert RPM to rad/s
  double wheel_angular_velocity = rpm * rpm_to_rad_per_sec_;
  
  // Convert angular velocity to linear velocity
  return wheel_angular_velocity * wheel_radius;
}

void MecanumBotSystemHardware::calculate_mecanum_kinematics(
  const double linear_x, const double linear_y, const double angular_z,
  std::vector<double> & wheel_velocities)
{
  // Assuming 4-wheel mecanum configuration:
  // Index 0: front_left
  // Index 1: front_right
  // Index 2: rear_left
  // Index 3: rear_right
  
  if (wheel_velocities.size() < 4)
  {
    std::cerr << "Not enough wheels for mecanum kinematics" << std::endl;
    return;
  }
  
  // Mecanum wheel kinematics matrix
  // For standard 45-degree mecanum wheels
  const double l = wheel_separation_x_[0];  // distance from center to wheel along x (adjust as needed)
  const double w = wheel_separation_y_[0];  // distance from center to wheel along y (adjust as needed)
  
  wheel_velocities[0] = linear_x - linear_y - (l + w) * angular_z;  // front_left
  wheel_velocities[1] = linear_x + linear_y + (l + w) * angular_z;  // front_right
  // wheel_velocities[2] = linear_x + linear_y - (l + w) * angular_z;  // rear_left
  // wheel_velocities[3] = linear_x - linear_y + (l + w) * angular_z;  // rear_right // 麦轮计算顺序
  // ros2_control 中的mecanum_drive_controller使用的顺序 是 front_left, front_right, rear_right, rear_left
  wheel_velocities[2] = linear_x - linear_y + (l + w) * angular_z;  // rear_right
  wheel_velocities[3] = linear_x + linear_y - (l + w) * angular_z;  // rear_left
}

void MecanumBotSystemHardware::calculate_inverse_kinematics(
  const std::vector<double> & wheel_velocities,
  double & linear_x, double & linear_y, double & angular_z)
{
  if (wheel_velocities.size() < 4)
  {
    std::cerr << "Not enough wheels for inverse kinematics" << std::endl;
    return;
  }
  
  // Inverse kinematics for mecanum wheels
  const double l = wheel_separation_x_[0];  // distance from center to wheel along x
  const double w = wheel_separation_y_[0];  // distance from center to wheel along y
  
  linear_x = (wheel_velocities[0] + wheel_velocities[1] + 
              wheel_velocities[2] + wheel_velocities[3]) / 4.0;
  
  // linear_y = (-wheel_velocities[0] + wheel_velocities[1] + 
  //              wheel_velocities[2] - wheel_velocities[3]) / 4.0; // 适配ros2_control的wheel顺序
    linear_y = (-wheel_velocities[0] + wheel_velocities[1] + 
               wheel_velocities[3] - wheel_velocities[2]) / 4.0;
  
  angular_z = (-wheel_velocities[0] + wheel_velocities[1] - 
                wheel_velocities[2] + wheel_velocities[3]) / (4.0 * (l + w));
}

}  // namespace mecanumbot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mecanumbot_hardware::MecanumBotSystemHardware,
  hardware_interface::SystemInterface)
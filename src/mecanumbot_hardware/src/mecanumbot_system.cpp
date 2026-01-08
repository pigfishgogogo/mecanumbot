#include "mecanumbot_hardware/mecanumbot_hardware.hpp"
#include <controller_manager/controller_manager.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>

class MecanumBotSystemNode : public rclcpp::Node
{
public:
  explicit MecanumBotSystemNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("mecanumbot_system_node", options)
  {
    RCLCPP_INFO(get_logger(), "Starting MecanumBotSystemNode...");
    
    // // Create hardware interface
    // hardware_interface_ = std::make_shared<mecanumbot_hardware::MecanumBotSystemHardware>();
    
    // // Initialize hardware interface
    // hardware_interface::HardwareInfo hardware_info;
    
    // // Configure hardware info from parameters
    // hardware_info.name = "mecanumbot_system";
    
    // // Load joint configuration from parameters
    // this->declare_parameter("joints", std::vector<std::string>{
    //   "front_left_wheel_joint", 
    //   "front_right_wheel_joint",
    //   "rear_right_wheel_joint",
    //   "rear_left_wheel_joint",
    // });
    
    // auto joint_names = this->get_parameter("joints").as_string_array();
    
    // for (const auto& joint_name : joint_names)
    // {
    //   hardware_interface::ComponentInfo joint_info;
    //   joint_info.name = joint_name;
    //   joint_info.type = "joint";
      
    //   // State interfaces
    //   joint_info.state_interfaces.push_back({hardware_interface::HW_IF_POSITION});
    //   joint_info.state_interfaces.push_back({hardware_interface::HW_IF_VELOCITY});
    //   joint_info.state_interfaces.push_back({hardware_interface::HW_IF_EFFORT});
      
    //   // Command interfaces
    //   joint_info.command_interfaces.push_back({hardware_interface::HW_IF_VELOCITY});
      
    //   hardware_info.joints.push_back(joint_info);
    // }
    
    // // Hardware parameters
    // // this->declare_parameter<std::string>("can_interface", "can0");
    // this->declare_parameter<std::string>("can_interface", "vcan0");
    // this->declare_parameter<int>("node_id_base", 600);
    // this->declare_parameter<int>("update_rate", 100);
    
    // hardware_info.hardware_parameters["can_interface"] = this->get_parameter("can_interface").as_string();
    // hardware_info.hardware_parameters["node_id_base"] = std::to_string(this->get_parameter("node_id_base").as_int());
    
    // update_rate_ = this->get_parameter("update_rate").as_int();
    
    // // Initialize hardware interface
    // if (hardware_interface_->on_init(hardware_info) != 
    //     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
    // {
    //   RCLCPP_ERROR(get_logger(), "Failed to initialize hardware interface");
    //   rclcpp::shutdown();
    //   return;
    // }

    // resource_manager_unique_ptr_ = std::make_unique<hardware_interface::ResourceManager>();

    

    // 从参数获取机器人描述
    this->declare_parameter("robot_description", "");
    auto robot_description = this->get_parameter("robot_description").as_string();
    
    if (robot_description.empty()) {
      RCLCPP_ERROR(get_logger(), "robot_description parameter is empty!");
      rclcpp::shutdown();
      return;
    }
    
    // 创建资源管理器，传入机器人描述
    resource_manager_unique_ptr_ = std::make_unique<hardware_interface::ResourceManager>(
      robot_description
    );


    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    std::string cm_node_name = std::string(this->get_name()) + "_cm";

    controller_manager_ = std::make_shared<controller_manager::ControllerManager>(
      std::move(resource_manager_unique_ptr_),  
      executor_,
      "controller_manager",
      cm_node_name
    );

    // 将控制器管理器添加到执行器
    executor_->add_node(controller_manager_);
    
    // Create timer for control loop
    double period = 1.0 / update_rate_;
    control_loop_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(period),
      std::bind(&MecanumBotSystemNode::update, this));
    
    RCLCPP_INFO(get_logger(), "MecanumBotSystemNode started successfully");
  }
  
  void update()
  {
    auto current_time = this->now();
    static auto previous_time = current_time;
    rclcpp::Duration period = current_time - previous_time;
    previous_time = current_time;
    
    // Update hardware interface
    hardware_interface_->read(rclcpp::Time(current_time), period);
    controller_manager_->update(rclcpp::Time(current_time), period);
    hardware_interface_->write(rclcpp::Time(current_time), period);
  }
  
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;

private:
  std::shared_ptr<mecanumbot_hardware::MecanumBotSystemHardware> hardware_interface_;
  std::unique_ptr<hardware_interface::ResourceManager> resource_manager_unique_ptr_;
  rclcpp::TimerBase::SharedPtr control_loop_timer_;
  int update_rate_;

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<MecanumBotSystemNode>();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
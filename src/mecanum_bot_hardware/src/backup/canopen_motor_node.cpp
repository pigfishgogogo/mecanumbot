#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "motor_controller.hpp"

class CanopenMotorNode : public rclcpp::Node {
public:
    CanopenMotorNode() : Node("canopen_motor_node") {
        // 参数声明
        this->declare_parameter<std::string>("can_interface", "can0");
        this->declare_parameter<int>("node_id", 1);
        this->declare_parameter<double>("max_velocity", 100.0);
        
        // 获取参数
        std::string can_interface = this->get_parameter("can_interface").as_string();
        int node_id = this->get_parameter("node_id").as_int();
        max_velocity_ = this->get_parameter("max_velocity").as_double();
        
        // 初始化电机控制器
        RCLCPP_INFO(this->get_logger(), "Initializing motor controller...");
        RCLCPP_INFO(this->get_logger(), "CAN Interface: %s", can_interface.c_str());
        RCLCPP_INFO(this->get_logger(), "Node ID: %d", node_id);
        
        motor_controller_ = std::make_unique<MotorController>(node_id);
        
        if (!motor_controller_->initialize(can_interface)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize motor controller");
            rclcpp::shutdown();
            return;
        }
        
        // 创建订阅者
        velocity_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "target_velocity", 10,
            std::bind(&CanopenMotorNode::velocityCallback, this, std::placeholders::_1));
        
        enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "enable_motor", 10,
            std::bind(&CanopenMotorNode::enableCallback, this, std::placeholders::_1));
        
        // 创建命令订阅者
        command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "motor_command", 10,
            std::bind(&CanopenMotorNode::commandCallback, this, std::placeholders::_1));
        
        // 创建发布者
        status_pub_ = this->create_publisher<std_msgs::msg::Int32>("motor_status", 10);
        actual_vel_pub_ = this->create_publisher<std_msgs::msg::Int32>("actual_velocity", 10);
        log_pub_ = this->create_publisher<std_msgs::msg::String>("motor_log", 10);
        
        // 创建定时器
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&CanopenMotorNode::statusTimerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "CanopenMotorNode initialized successfully");
        
        // 发布初始化完成消息
        auto log_msg = std_msgs::msg::String();
        log_msg.data = "Motor controller initialized";
        log_pub_->publish(log_msg);
    }
    
    ~CanopenMotorNode() {
        if (motor_controller_) {
            RCLCPP_INFO(this->get_logger(), "Shutting down motor controller...");
            motor_controller_->shutdown();
        }
    }

private:
    void velocityCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        int32_t velocity = msg->data;
        
        // 限制速度范围
        if (velocity > max_velocity_) velocity = max_velocity_;
        if (velocity < -max_velocity_) velocity = -max_velocity_;
        
        RCLCPP_INFO(this->get_logger(), "Received target velocity: %d", velocity);
        
        if (motor_controller_->setTargetVelocity(velocity)) {
            RCLCPP_INFO(this->get_logger(), "Successfully set target velocity: %d", velocity);
            
            auto log_msg = std_msgs::msg::String();
            log_msg.data = "Velocity set to " + std::to_string(velocity);
            log_pub_->publish(log_msg);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to set target velocity");
            
            auto log_msg = std_msgs::msg::String();
            log_msg.data = "Failed to set velocity";
            log_pub_->publish(log_msg);
        }
    }
    
    void enableCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received enable command: %s", msg->data ? "true" : "false");
        
        if (msg->data) {
            // 启用电机
            if (motor_controller_->enableMotor()) {
                // 设置为速度模式
                if (motor_controller_->setModeOfOperation(3)) {  // 3 = 速度模式
                    RCLCPP_INFO(this->get_logger(), "Motor enabled and set to velocity mode");
                    
                    auto log_msg = std_msgs::msg::String();
                    log_msg.data = "Motor enabled (velocity mode)";
                    log_pub_->publish(log_msg);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Motor enabled but failed to set velocity mode");
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to enable motor");
            }
        } else {
            // 禁用电机
            motor_controller_->disableMotor();
            RCLCPP_INFO(this->get_logger(), "Motor disabled");
            
            auto log_msg = std_msgs::msg::String();
            log_msg.data = "Motor disabled";
            log_pub_->publish(log_msg);
        }
    }
    
    void commandCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string command = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received command: %s", command.c_str());
        
        if (command == "start") {
            // 修复这里：正确创建Bool消息
            auto bool_msg = std::make_shared<std_msgs::msg::Bool>();
            bool_msg->data = true;
            enableCallback(bool_msg);
        } else if (command == "stop") {
            auto bool_msg = std::make_shared<std_msgs::msg::Bool>();
            bool_msg->data = false;
            enableCallback(bool_msg);
        } else if (command == "zero") {
            if (motor_controller_->setVelocityZero()) {
                RCLCPP_INFO(this->get_logger(), "Velocity set to zero");
                
                auto log_msg = std_msgs::msg::String();
                log_msg.data = "Velocity set to zero";
                log_pub_->publish(log_msg);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to set zero velocity");
            }
        } else if (command == "status") {
            // 查询状态
            uint16_t status;
            if (motor_controller_->getStatusWord(status)) {
                RCLCPP_INFO(this->get_logger(), "Motor status word: 0x%04X", status);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to get motor status");
            }
        }
    }
    
    void statusTimerCallback() {
        // 读取状态字
        uint16_t status_word;
        if (motor_controller_->getStatusWord(status_word)) {
            auto status_msg = std_msgs::msg::Int32();
            status_msg.data = static_cast<int32_t>(status_word);
            status_pub_->publish(status_msg);
        }
        
        // 读取实际速度
        int32_t actual_velocity;
        if (motor_controller_->getActualVelocity(actual_velocity)) {
            auto vel_msg = std_msgs::msg::Int32();
            vel_msg.data = actual_velocity;
            actual_vel_pub_->publish(vel_msg);
        }
    }
    
    std::unique_ptr<MotorController> motor_controller_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr actual_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    double max_velocity_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanopenMotorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
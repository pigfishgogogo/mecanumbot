#include <iostream>
#include <thread>
#include <chrono>
#include "motor_controller.hpp"

int main() {
    std::cout << "CANopen Motor Control Test" << std::endl;
    
    MotorController motor(1);  // 节点ID为1
    
    if (!motor.initialize("can0")) {
        std::cerr << "Failed to initialize motor controller" << std::endl;
        return 1;
    }
    
    try {
        // 启用电机
        std::cout << "Enabling motor..." << std::endl;
        if (!motor.enableMotor()) {
            std::cerr << "Failed to enable motor" << std::endl;
            return 1;
        }
        
        // 设置为速度模式
        std::cout << "Setting velocity mode..." << std::endl;
        if (!motor.setModeOfOperation(3)) {  // 3 = 速度模式
            std::cerr << "Failed to set velocity mode" << std::endl;
            return 1;
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 测试正转
        std::cout << "Setting positive velocity..." << std::endl;
        motor.setTargetVelocity(500);  // 500 RPM
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        // 测试反转
        std::cout << "Setting negative velocity..." << std::endl;
        motor.setTargetVelocity(-500);  // -500 RPM
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        // 停止电机
        std::cout << "Stopping motor..." << std::endl;
        motor.setVelocityZero();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 禁用电机
        std::cout << "Disabling motor..." << std::endl;
        motor.disableMotor();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "Test completed successfully" << std::endl;
    return 0;
}
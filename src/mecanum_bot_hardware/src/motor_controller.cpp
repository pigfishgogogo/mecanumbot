#include "mecanum_bot_hardware/motor_controller.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>

MotorController::MotorController(uint8_t node_id) 
    : node_id_(node_id) {
    std::cout << "MotorController created with node ID: " << static_cast<int>(node_id_) << std::endl;
}

MotorController::~MotorController() {
    shutdown();
}

bool MotorController::initialize(const std::string& can_interface) {
    try {
        can_interface_ = std::make_unique<CanInterface>(can_interface);
        bool success = can_interface_->initialize();
        
        if (success) {
            std::cout << "Motor controller initialized successfully" << std::endl;
        } else {
            std::cerr << "Failed to initialize motor controller" << std::endl;
        }
        
        return success;
    } catch (const std::exception& e) {
        std::cerr << "Exception initializing motor controller: " << e.what() << std::endl;
        return false;
    }
}

bool MotorController::shutdown() {
    if (can_interface_) {
        disableMotor();
        can_interface_->closeInterface();
    }
    return true;
}

bool MotorController::enableMotor() {
    if (!can_interface_) {
        std::cerr << "CAN interface not initialized" << std::endl;
        return false;
    }
    
    std::cout << "Starting motor enable sequence..." << std::endl;
    
    // 发送上电指令
    std::cout << "Sending power on command..." << std::endl;
    
    // 根据命令：cansend can0 601#2B4060000F000000
    // 这个是SDO下载命令：2B = 下载4字节，40 = 索引低字节，60 = 索引高字节，00 = 子索引，0F000000 = 数据
    // 对应控制字 0x000F (Enable Operation)
    
    uint8_t enable_cmd[] = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};
    if (!can_interface_->sendFrame(getCobIdTx(), enable_cmd, sizeof(enable_cmd))) {
        std::cerr << "Failed to send enable command" << std::endl;
        return false;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::cout << "Motor enable sequence completed" << std::endl;
    return true;
}

bool MotorController::disableMotor() {
    if (!can_interface_) {
        return false;
    }
    
    std::cout << "Disabling motor..." << std::endl;
    
    // 发送停止指令
    uint8_t disable_cmd[] = {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};  // Shutdown
    can_interface_->sendFrame(getCobIdTx(), disable_cmd, sizeof(disable_cmd));
    
    return true;
}

bool MotorController::setModeOfOperation(uint8_t mode) {
    if (!can_interface_) {
        return false;
    }
    
    std::cout << "Setting mode of operation to: " << static_cast<int>(mode) << std::endl;
    
    // 根据你之前成功的命令：cansend can0 601#2F60600003000000
    // 2F = 下载1字节，60 = 索引低字节，60 = 索引高字节，00 = 子索引，03 = 数据 (profile velocity mode)
    
    uint8_t mode_cmd[] = {0x2F, 0x60, 0x60, 0x00, mode, 0x00, 0x00, 0x00};
    bool success = can_interface_->sendFrame(getCobIdTx(), mode_cmd, sizeof(mode_cmd));
    
    if (success) {
        std::cout << "Mode set successfully" << std::endl;
    } else {
        std::cerr << "Failed to set mode" << std::endl;
    }
    
    return success;
}

bool MotorController::setTargetVelocity(int32_t rpm) {
    if (!can_interface_) {
        return false;
    }
    
    std::cout << "Setting target rpm_velocity to: " << rpm << std::endl;
    
    // 根据你之前成功的命令：cansend can0 601#23ff60007eb1e4ff
    // 23 = 下载4字节，FF = 索引低字节，60 = 索引高字节，00 = 子索引，7eb1e4ff = 数据

    // 使用CANopen标准公式转换：CAN_vel_value(dec) = (rpm * 512 * encoder_resolution) / 1875
    // encoder_resolution 应该从配置中获取，这里假设为65536
    // const int32_t encoder_resolution_ = 65536; // 编码器分辨率，应该配置化
    
    // 计算CANopen速度值（注意处理溢出和精度）
    int64_t temp = static_cast<int64_t>(rpm) * 512 * encoder_resolution_;
    int32_t can_velocity = static_cast<int32_t>(temp / 1875);
    
    std::cout << "CAN velocity value: " << can_velocity 
              << " (hex: 0x" << std::hex << can_velocity << std::dec << ")" << std::endl;
    
    uint8_t velocity_cmd[8];
    velocity_cmd[0] = 0x23;  // 下载4字节
    velocity_cmd[1] = 0xFF;  // 目标速度索引低字节 (0x60FF)
    velocity_cmd[2] = 0x60;  // 目标速度索引高字节
    velocity_cmd[3] = 0x00;  // 子索引
    
    // 将速度值转换为字节（小端序）
    velocity_cmd[4] = can_velocity & 0xFF;
    velocity_cmd[5] = (can_velocity >> 8) & 0xFF;
    velocity_cmd[6] = (can_velocity >> 16) & 0xFF;
    velocity_cmd[7] = (can_velocity >> 24) & 0xFF;
    
    // 打印调试信息
    std::cout << "Velocity command bytes: ";
    for (int i = 0; i < 8; i++) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(velocity_cmd[i]) << " ";
    }
    std::cout << std::dec << std::endl;
    
    bool success = can_interface_->sendFrame(getCobIdTx(), velocity_cmd, sizeof(velocity_cmd));
    
    if (success) {
        std::cout << "Velocity command sent successfully" << std::endl;
    } else {
        std::cerr << "Failed to send velocity command" << std::endl;
    }
    
    return success;
}

bool MotorController::setVelocityZero() {
    std::cout << "Setting velocity to zero" << std::endl;
    return setTargetVelocity(0);
}

bool MotorController::setControlWord(uint16_t control_word) {
    if (!can_interface_) return false;
    
    uint8_t data[4];
    data[0] = control_word & 0xFF;
    data[1] = (control_word >> 8) & 0xFF;
    data[2] = 0x00;
    data[3] = 0x00;
    
    return sdoDownload(INDEX_CONTROL_WORD, 0x00, data, 4);
}

bool MotorController::sdoDownload(uint16_t index, uint8_t subindex, const uint8_t* data, uint8_t length, bool expedited) {
    if (!can_interface_ || length > 4) return false;
    
    // 构建SDO下载帧
    uint8_t sdo_data[8] = {0};
    
    if (expedited) {
        // 加速传输
        sdo_data[0] = 0x23;  // 加速下载，4字节数据
        if (length < 4) {
            sdo_data[0] = 0x23 - (4 - length);  // 调整命令字
        }
    } else {
        // 分段传输（简化实现）
        sdo_data[0] = 0x21;  // 分段下载开始
    }
    
    sdo_data[1] = static_cast<uint8_t>(index & 0xFF);
    sdo_data[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
    sdo_data[3] = subindex;
    
    // 拷贝数据
    for (uint8_t i = 0; i < length; i++) {
        sdo_data[4 + i] = data[i];
    }
    
    return can_interface_->sendFrame(getCobIdTx(), sdo_data, 8);
}

bool MotorController::sdoUpload(uint16_t index, uint8_t subindex, uint8_t* data, uint8_t& length) {
    if (!can_interface_) return false;
    
    // 1. 送SDO上传请求
    uint8_t sdo_data[8];
    sdo_data[0] = 0x40;// 上传请求
    sdo_data[1] = static_cast<uint8_t>(index & 0xFF);
    sdo_data[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
    sdo_data[3] = subindex;
    sdo_data[4] = 0;
    sdo_data[5] = 0;
    sdo_data[6] = 0;
    sdo_data[7] = 0;
    
    if (!can_interface_->sendFrame(getCobIdTx(), sdo_data, 8)) {
        return false;
    }

    // 2. 等待并接收响应
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count() < 100) {
        // 接收响应
        struct can_frame frame;
        if (can_interface_->receiveFrame(frame)) {
            if (frame.can_dlc < 4) {
                continue;  // 帧太短，无效
            }

            // 检查索引和子索引是否匹配
            uint16_t resp_index = frame.data[1] | (frame.data[2] << 8);
            uint8_t resp_subindex = frame.data[3];
            if (resp_index != index || resp_subindex != subindex) {
                continue;  // 不是我们要的响应
            }

            // 检查命令字
            uint8_t cmd = frame.data[0];

            // 处理错误响应
            if ((cmd & 0xF0) == 0x80) {
                uint32_t error_code = frame.data[4] | (frame.data[5] << 8) | (frame.data[6] << 16) | (frame.data[7] << 24);
                std::cerr << "SDO Upload Error (index 0x" << std::hex << index << "): 0x" << error_code << std::dec << std::endl;
                return false;
            }
            // 处理成功响应
            if ((cmd & 0xF0) == 0x40 || (cmd & 0xF0) == 0x00) {  // 上传响应
                length = frame.can_dlc - 4;
                if (length > 0) {
                    std::memcpy(data, &frame.data[4], length);
                }
                std::cout << "SDO Upload successful: index=0x" << std::hex << index << ", data=";
                for (uint8_t i = 0; i < length; i++) {
                    std::cout << std::setw(2) << std::setfill('0') << static_cast<int>(data[i]) << " ";
                }
                std::cout << std::dec << std::endl;
                return true;
            }
        }
        // 短暂休眠避免CPU占用过高
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::cerr << "SDO upload timeout for index 0x" << std::hex << index << std::dec << std::endl;
    return false;

    
    // // 接收响应
    // struct can_frame frame;
    // if (can_interface_->receiveFrame(frame)) {
    //     if (frame.can_dlc >= 4) {
    //         uint8_t cmd = frame.data[0];
    //         if ((cmd & 0xF0) == 0x40 || (cmd & 0xF0) == 0x00) {  // 上传响应
    //             length = frame.can_dlc - 4;
    //             if (length > 0) {
    //                 std::memcpy(data, &frame.data[4], length);
    //             }
    //             return true;
    //         }
    //     }
    // }
    // return false;
}

bool MotorController::getStatusWord(uint16_t& status_word) {
    uint8_t data[4] = {0};
    uint8_t length = 0;
    
    if (sdoUpload(INDEX_STATUS_WORD, 0x00, data, length) && length >= 2) {
        status_word = data[0] | (data[1] << 8);
        std::cout << "Status word: 0x" << std::hex << status_word << std::dec << std::endl;
        return true;
    }
    
    std::cerr << "Failed to get status word" << std::endl;
    return false;
}

bool MotorController::getActualVelocity(int32_t& velocity) {
    uint8_t data[4] = {0};
    uint8_t length = 0;
    
    if (sdoUpload(INDEX_ACTUAL_VELOCITY, 0x00, data, length) && length == 4) {
        // 获取CAN原始值（小端序）
        velocity = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
        
        // 反向转换公式：rpm = (can_value * 1875) / (512 * encoder_resolution)
        double rpm = static_cast<double>(velocity) * 1875.0 / 
                     (512.0 * encoder_resolution_);
        
        std::cout << "Actual velocity [CAN value]: " << velocity 
                  << " (0x" << std::hex << std::setw(8) << std::setfill('0') 
                  << velocity << ")" << std::dec << "\t"
                  << "[RPM value]: " << std::fixed 
                  << std::setprecision(2) << rpm << " rpm" << std::endl;
        
        return true;
    }
    
    std::cerr << "Failed to get actual velocity" << std::endl;
    return false;
}
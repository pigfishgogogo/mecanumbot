#pragma once
#include <cstdint>
#include <memory>
#include "mecanumbot_hardware/can_interface.hpp"

class MotorController {
public:
    MotorController(uint8_t node_id = 1);  // 默认节点ID为1
    ~MotorController();

    bool initialize(const std::string& can_interface = "can0");
    bool shutdown();
    
    // CANopen控制命令
    bool enableMotor();
    bool disableMotor();
    bool setModeOfOperation(uint8_t mode);
    bool setTargetVelocity(int32_t velocity);
    bool setVelocityZero();
    bool setControlWord(uint16_t control_word);
    
    // 状态查询
    bool getStatusWord(uint16_t& status_word);
    bool getActualVelocity(int32_t& velocity);

    int32_t getEncoderResolution() const {
        return encoder_resolution_;
    }
    
private:
    const int32_t encoder_resolution_ = 65536; // 编码器分辨率，应该配置化

    // CANopen SDO上传命令
    bool sdoUpload(uint16_t index, uint8_t subindex, uint8_t* data, uint8_t& length);
    
    // CANopen SDO下载命令
    bool sdoDownload(uint16_t index, uint8_t subindex, const uint8_t* data, uint8_t length, bool expedited = true);
    
    // PDO相关（如果需要）
    bool configurePdo();
    
    uint8_t node_id_;
    std::unique_ptr<CanInterface> can_interface_;
    
    // CANopen地址计算
    uint32_t getCobIdTx() const { return 0x600 + node_id_; }  // 主站（你的程序）发送到节点的COB-ID
    uint32_t getCobIdRx() const { return 0x580 + node_id_; }  // 主站（你的程序）从节点接收的COB-ID
    
    // 控制字定义
    static constexpr uint16_t CONTROL_SHUTDOWN = 0x0006;
    static constexpr uint16_t CONTROL_SWITCH_ON = 0x0007;
    static constexpr uint16_t CONTROL_ENABLE_OPERATION = 0x000F;
    static constexpr uint16_t CONTROL_FAULT_RESET = 0x0080;
    
    // 运行模式定义
    static constexpr uint8_t MODE_PROFILE_VELOCITY = 3;
    
    // 对象字典索引
    static constexpr uint16_t INDEX_CONTROL_WORD = 0x6040;
    static constexpr uint16_t INDEX_STATUS_WORD = 0x6041;
    static constexpr uint16_t INDEX_MODE_OF_OPERATION = 0x6060;
    static constexpr uint16_t INDEX_TARGET_VELOCITY = 0x6042;
    static constexpr uint16_t INDEX_ACTUAL_VELOCITY = 0x606C;
};
#include "mecanum_bot_hardware/can_interface.hpp"

CanInterface::CanInterface(const std::string& interface) 
    : interface_name_(interface), socket_fd_(-1) {}

CanInterface::~CanInterface() {
    closeInterface();
}

bool CanInterface::initialize() {
    // 创建socket
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        throw std::runtime_error("Failed to create CAN socket");
    }

    // 获取接口索引
    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, interface_name_.c_str());
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        close(socket_fd_);
        socket_fd_ = -1;
        throw std::runtime_error("Failed to get CAN interface index");
    }

    // 绑定socket
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(socket_fd_);
        socket_fd_ = -1;
        throw std::runtime_error("Failed to bind CAN socket");
    }

    return true;
}

bool CanInterface::sendFrame(uint32_t can_id, const uint8_t* data, uint8_t length) {
    if (socket_fd_ < 0 || length > 8) {
        return false;
    }

    struct can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = length;
    std::memcpy(frame.data, data, length);

    int bytes_sent = write(socket_fd_, &frame, sizeof(frame));
    return bytes_sent == sizeof(frame);
}

bool CanInterface::receiveFrame(struct can_frame& frame, int timeout_ms) {
    if (socket_fd_ < 0) {
        return false;
    }

    // 设置超时
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    int bytes_read = read(socket_fd_, &frame, sizeof(frame));
    return bytes_read == sizeof(frame);
}

void CanInterface::closeInterface() {
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
}
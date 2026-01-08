#pragma once
#include <string>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>

class CanInterface {
public:
    CanInterface(const std::string& interface = "can0");
    ~CanInterface();

    bool initialize();
    bool sendFrame(uint32_t can_id, const uint8_t* data, uint8_t length);
    bool receiveFrame(struct can_frame& frame, int timeout_ms = 1000);
    void closeInterface();

private:
    std::string interface_name_;
    int socket_fd_;
};
#include "lidar_lite_ros/lidar_lite.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <iostream>
#include <thread>
#include <chrono>

namespace lidar_lite_ros {

const std::map<std::string, LidarSettings> LidarLite::PRESETS = {
    {"balanced", {128, 8, 3, 0}},
    {"high_speed", {29, 0, 3, 0}},
    {"high_accuracy", {255, 8, 5, 0}},
    {"long_range", {192, 8, 3, 0}}
};

LidarLite::LidarLite(int bus_id, uint8_t address)
    : bus_id_(bus_id), address_(address), fd_(-1), connected_(false) {
    
    std::string device = "/dev/i2c-" + std::to_string(bus_id_);
    fd_ = open(device.c_str(), O_RDWR);
    
    if (fd_ < 0) {
        std::cerr << "Failed to open I2C bus " << device << std::endl;
        return;
    }

    if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
        std::cerr << "Failed to acquire bus access and/or talk to slave 0x" 
                  << std::hex << (int)address_ << std::dec << std::endl;
        ::close(fd_);
        fd_ = -1;
        return;
    }

    // Check connection by performing a dummy write (equivalent to Python's write_quick)
    // For I2C, a simple write of 0 bytes or a read can check for ACK.
    // Lidar Lite might not like a 0-byte write, let's try reading the status register.
    uint8_t status;
    if (readRegister(STATUS_REG, status)) {
        connected_ = true;
    } else {
        std::cerr << "Lidar Lite not responding on address 0x" 
                  << std::hex << (int)address_ << std::dec << std::endl;
        ::close(fd_);
        fd_ = -1;
    }
}

LidarLite::~LidarLite() {
    close();
}

bool LidarLite::writeRegister(uint8_t reg, uint8_t val) {
    if (!connected_) return false;
    
    uint8_t buffer[2] = {reg, val};
    if (write(fd_, buffer, 2) != 2) {
        std::cerr << "Write error to 0x" << std::hex << (int)reg << std::dec << std::endl;
        return false;
    }
    return true;
}

bool LidarLite::readRegister(uint8_t reg, uint8_t& val) {
    if (fd_ < 0) return false;

    if (write(fd_, &reg, 1) != 1) {
        return false;
    }
    if (read(fd_, &val, 1) != 1) {
        return false;
    }
    return true;
}

bool LidarLite::readBlock(uint8_t reg, uint8_t* data, uint8_t len) {
    if (fd_ < 0) return false;

    if (write(fd_, &reg, 1) != 1) {
        return false;
    }
    if (read(fd_, data, len) != len) {
        return false;
    }
    return true;
}

bool LidarLite::configure(const LidarSettings& settings) {
    if (!connected_) return false;
    
    bool success = true;
    success &= writeRegister(SIG_COUNT_VAL, settings.sig_count_val);
    success &= writeRegister(ACQ_CONFIG_REG, settings.acq_config_reg);
    success &= writeRegister(THRESHOLD_BYPASS, settings.threshold_bypass);
    success &= writeRegister(REF_COUNT_VAL, settings.ref_count_val);
    
    return success;
}

std::optional<float> LidarLite::readDistance() {
    if (!connected_) return std::nullopt;

    // 1. Trigger measurement
    if (!writeRegister(ACQ_COMMAND, 0x04)) {
        return std::nullopt;
    }

    // 2. Wait for busy flag to go low
    for (int i = 0; i < 100; ++i) {
        uint8_t status;
        if (readRegister(STATUS_REG, status)) {
            if (!(status & 0x01)) { // Busy bit is 0
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (i == 99) {
            // std::cerr << "Lidar read timeout" << std::endl;
            return std::nullopt;
        }
    }

    // 3. Read distance
    uint8_t val[2];
    if (readBlock(FULL_DELAY_HIGH, val, 2)) {
        uint16_t dist_cm = (static_cast<uint16_t>(val[0]) << 8) | val[1];
        return static_cast<float>(dist_cm) / 100.0f;
    }

    return std::nullopt;
}

void LidarLite::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    connected_ = false;
}

} // namespace lidar_lite_ros

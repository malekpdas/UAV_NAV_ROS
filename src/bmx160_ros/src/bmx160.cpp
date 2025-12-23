#include "bmx160_ros/bmx160.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

namespace bmx160_ros {

const std::map<GyroRange, float> BMX160::GYRO_SENSITIVITIES = {
    {GyroRange::DPS_125, 0.0038110f},
    {GyroRange::DPS_250, 0.0076220f},
    {GyroRange::DPS_500, 0.0152439f},
    {GyroRange::DPS_1000, 0.0304878f},
    {GyroRange::DPS_2000, 0.0609756f}
};

const std::map<AccelRange, float> BMX160::ACCEL_SENSITIVITIES = {
    {AccelRange::G_2, 0.000061035f},
    {AccelRange::G_4, 0.000122070f},
    {AccelRange::G_8, 0.000244141f},
    {AccelRange::G_16, 0.000488281f}
};

const std::map<AccelRange, uint8_t> BMX160::ACCEL_RANGE_REG_VALUES = {
    {AccelRange::G_2, 0x03},
    {AccelRange::G_4, 0x05},
    {AccelRange::G_8, 0x08},
    {AccelRange::G_16, 0x0C}
};

BMX160::BMX160(int bus, uint8_t i2c_addr)
    : bus_(bus), i2c_addr_(i2c_addr), fd_(-1), initialized_(false) {
    
    std::string device = "/dev/i2c-" + std::to_string(bus_);
    fd_ = open(device.c_str(), O_RDWR);
    
    if (fd_ < 0) {
        std::cerr << "Failed to open I2C bus " << device << std::endl;
        return;
    }

    if (ioctl(fd_, I2C_SLAVE, i2c_addr_) < 0) {
        std::cerr << "Failed to acquire bus access and/or talk to slave 0x" 
                  << std::hex << (int)i2c_addr_ << std::dec << std::endl;
        ::close(fd_);
        fd_ = -1;
        return;
    }

    // Default settings
    gyro_sensitivity_ = GYRO_SENSITIVITIES.at(GyroRange::DPS_250);
    accel_sensitivity_ = ACCEL_SENSITIVITIES.at(AccelRange::G_2);

    std::this_thread::sleep_for(std::chrono::milliseconds(160));
}

BMX160::~BMX160() {
    if (fd_ >= 0) {
        ::close(fd_);
    }
}

bool BMX160::begin() {
    if (fd_ < 0) return false;

    // Check if sensor is present
    uint8_t chip_id;
    if (!readRegisters(CHIP_ID_ADDR, &chip_id, 1) || chip_id != CHIP_ID_EXPECTED) {
        std::cerr << "BMX160 not found or wrong chip ID: 0x" << std::hex << (int)chip_id << std::dec << std::endl;
        return false;
    }

    if (!softReset()) return false;

    // Enable sensors
    writeRegister(COMMAND_REG_ADDR, ACCEL_NORMAL_MODE_CMD);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    writeRegister(COMMAND_REG_ADDR, GYRO_NORMAL_MODE_CMD);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    writeRegister(COMMAND_REG_ADDR, MAG_NORMAL_MODE_CMD);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    configureMagnetometer();

    initialized_ = true;
    return true;
}

void BMX160::setGyroRange(GyroRange range) {
    if (fd_ < 0) return;
    gyro_sensitivity_ = GYRO_SENSITIVITIES.at(range);
    writeRegister(GYRO_RANGE_ADDR, static_cast<uint8_t>(range));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void BMX160::setAccelRange(AccelRange range) {
    if (fd_ < 0) return;
    accel_sensitivity_ = ACCEL_SENSITIVITIES.at(range);
    writeRegister(ACCEL_RANGE_ADDR, ACCEL_RANGE_REG_VALUES.at(range));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

std::optional<BMX160Data> BMX160::getAllData() {
    if (!initialized_) return std::nullopt;

    uint8_t raw_data[20];
    if (!readRegisters(MAG_DATA_ADDR, raw_data, 20)) {
        return std::nullopt;
    }

    BMX160Data data;

    // Parse magnetometer (bytes 0-5)
    data.mag[0] = static_cast<float>(parseSignedInt16(raw_data[1], raw_data[0])) * MAG_SENSITIVITY;
    data.mag[1] = static_cast<float>(parseSignedInt16(raw_data[3], raw_data[2])) * MAG_SENSITIVITY;
    data.mag[2] = static_cast<float>(parseSignedInt16(raw_data[5], raw_data[4])) * MAG_SENSITIVITY;

    // Parse gyroscope (bytes 8-13)
    data.gyro[0] = static_cast<float>(parseSignedInt16(raw_data[9], raw_data[8])) * gyro_sensitivity_;
    data.gyro[1] = static_cast<float>(parseSignedInt16(raw_data[11], raw_data[10])) * gyro_sensitivity_;
    data.gyro[2] = static_cast<float>(parseSignedInt16(raw_data[13], raw_data[12])) * gyro_sensitivity_;

    // Parse accelerometer (bytes 14-19)
    data.accel[0] = static_cast<float>(parseSignedInt16(raw_data[15], raw_data[14])) * accel_sensitivity_ * GRAVITY_MS2;
    data.accel[1] = static_cast<float>(parseSignedInt16(raw_data[17], raw_data[16])) * accel_sensitivity_ * GRAVITY_MS2;
    data.accel[2] = static_cast<float>(parseSignedInt16(raw_data[19], raw_data[18])) * accel_sensitivity_ * GRAVITY_MS2;

    return data;
}

bool BMX160::softReset() {
    if (fd_ < 0) return false;
    if (!writeRegister(COMMAND_REG_ADDR, SOFT_RESET_CMD)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    initialized_ = false;
    return true;
}

void BMX160::setLowPower() {
    softReset();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    configureMagnetometer();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    writeRegister(COMMAND_REG_ADDR, ACCEL_SUSPEND_MODE_CMD);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    writeRegister(COMMAND_REG_ADDR, GYRO_SUSPEND_MODE_CMD);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    writeRegister(COMMAND_REG_ADDR, MAG_SUSPEND_MODE_CMD);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void BMX160::wakeUp() {
    softReset();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    configureMagnetometer();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    writeRegister(COMMAND_REG_ADDR, ACCEL_NORMAL_MODE_CMD);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    writeRegister(COMMAND_REG_ADDR, GYRO_NORMAL_MODE_CMD);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    writeRegister(COMMAND_REG_ADDR, MAG_NORMAL_MODE_CMD);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void BMX160::configureMagnetometer() {
    writeRegister(MAGN_IF_0_ADDR, 0x80);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    writeRegister(MAGN_IF_3_ADDR, 0x01);
    writeRegister(MAGN_IF_2_ADDR, 0x4B);
    writeRegister(MAGN_IF_3_ADDR, 0x04);
    writeRegister(MAGN_IF_2_ADDR, 0x51);
    writeRegister(MAGN_IF_3_ADDR, 0x0E);
    writeRegister(MAGN_IF_2_ADDR, 0x52);
    writeRegister(MAGN_IF_3_ADDR, 0x02);
    writeRegister(MAGN_IF_2_ADDR, 0x4C);
    writeRegister(MAGN_IF_1_ADDR, 0x42);
    writeRegister(MAGN_CONFIG_ADDR, 0x08);
    writeRegister(MAGN_IF_0_ADDR, 0x03);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

int16_t BMX160::parseSignedInt16(uint8_t msb, uint8_t lsb) {
    int16_t value = static_cast<int16_t>((msb << 8) | lsb);
    return value;
}

bool BMX160::writeRegister(uint8_t reg, uint8_t value) {
    if (fd_ < 0) return false;
    uint8_t buffer[2] = {reg, value};
    return write(fd_, buffer, 2) == 2;
}

bool BMX160::readRegisters(uint8_t reg, uint8_t* data, uint8_t len) {
    if (fd_ < 0) return false;
    if (write(fd_, &reg, 1) != 1) return false;
    return read(fd_, data, len) == len;
}

} // namespace bmx160_ros

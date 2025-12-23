#include "zoe_m8q_ros/zoe_m8q.hpp"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <unistd.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>
#include <algorithm>
#include <cmath>

namespace zoe_m8q_ros {

ZoeM8Q::ZoeM8Q(int bus, uint8_t i2c_addr)
    : bus_fd_(-1), i2c_addr_(i2c_addr) {
    std::string device = "/dev/i2c-" + std::to_string(bus);
    bus_fd_ = open(device.c_str(), O_RDWR);
    if (bus_fd_ < 0) {
        std::cerr << "Failed to open I2C bus " << device << std::endl;
        return;
    }
    if (ioctl(bus_fd_, I2C_SLAVE, i2c_addr_) < 0) {
        std::cerr << "Failed to acquire bus access and/or talk to slave 0x"
                  << std::hex << (int)i2c_addr_ << std::dec << std::endl;
        ::close(bus_fd_);
        bus_fd_ = -1;
    }
}

ZoeM8Q::~ZoeM8Q() {
    if (bus_fd_ >= 0) {
        ::close(bus_fd_);
    }
}

bool ZoeM8Q::begin() {
    return bus_fd_ >= 0;
}

bool ZoeM8Q::configure(double rate_hz) {
    if (bus_fd_ < 0) return false;

    // 1. Set Rate (UBX-CFG-RATE)
    uint16_t meas_rate = static_cast<uint16_t>(std::max(10.0, std::round(1000.0 / rate_hz)));
    std::vector<uint8_t> rate_payload(6);
    rate_payload[0] = meas_rate & 0xFF;
    rate_payload[1] = (meas_rate >> 8) & 0xFF;
    rate_payload[2] = 0x01; // navRate = 1
    rate_payload[3] = 0x00;
    rate_payload[4] = 0x01; // timeRef = GPS
    rate_payload[5] = 0x00;
    
    if (!writeUbx(0x06, 0x08, rate_payload)) {
        std::cerr << "Failed to set Rate (UBX-CFG-RATE)" << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 1b. GNSS Config (UBX-CFG-GNSS) - GPS + QZSS only for high rate (15Hz+)
    // Concurrent mode with GLONASS/BeiDou/Galileo usually limits M8 series to 10Hz.
    std::vector<uint8_t> gnss_payload = {
        0x00, 0x00, 0x20, 0x07, // version, res, res, numBlocks
        0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, // GPS (Enable)
        0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, // SBAS (Disable)
        0x02, 0x04, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, // Galileo (Disable)
        0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01, // BeiDou (Disable)
        0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, // IMES (Disable)
        0x05, 0x00, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01, // QZSS (Enable)
        0x06, 0x08, 0x0E, 0x00, 0x00, 0x00, 0x01, 0x01  // GLONASS (Disable)
    };
    if (!writeUbx(0x06, 0x3E, gnss_payload)) {
        std::cerr << "Failed to set GNSS Config (UBX-CFG-GNSS)" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 2. Disable NMEA messages (Class 0xF0)
    uint8_t nmea_ids[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0D, 0x0E, 0x0F};
    for (uint8_t id : nmea_ids) {
        writeUbx(0x06, 0x01, {0xF0, id, 0x00});
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // 3. Enable NAV-PVT (Class 0x01 ID 0x07)
    if (!writeUbx(0x06, 0x01, {0x01, 0x07, 0x01})) {
        std::cerr << "Failed to enable NAV-PVT" << std::endl;
        return false;
    }
    
    return true;
}

bool ZoeM8Q::writeUbx(uint8_t msgClass, uint8_t msgId, const std::vector<uint8_t>& payload) {
    if (bus_fd_ < 0) return false;
    
    size_t len = payload.size();
    std::vector<uint8_t> frame;
    frame.push_back(ubx::SYNC1);
    frame.push_back(ubx::SYNC2);
    frame.push_back(msgClass);
    frame.push_back(msgId);
    frame.push_back(len & 0xFF);
    frame.push_back((len >> 8) & 0xFF);
    frame.insert(frame.end(), payload.begin(), payload.end());
    
    uint8_t cka, ckb;
    ubx::calculateChecksum(frame.data() + 2, frame.size() - 2, cka, ckb);
    frame.push_back(cka);
    frame.push_back(ckb);
    
    // UBX via I2C: Write payload to register 0xFF
    for (size_t i = 0; i < frame.size(); i += 32) {
        size_t chunk_len = std::min((size_t)32, frame.size() - i);
        uint8_t buf[33];
        buf[0] = 0xFF;
        std::memcpy(buf + 1, frame.data() + i, chunk_len);
        if (write(bus_fd_, buf, chunk_len + 1) != (ssize_t)(chunk_len + 1)) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    
    return true;
}

bool ZoeM8Q::writeRegister(uint8_t reg, uint8_t value) {
    if (bus_fd_ < 0) return false;
    uint8_t buffer[2] = {reg, value};
    return write(bus_fd_, buffer, 2) == 2;
}

bool ZoeM8Q::readRegisters(uint8_t reg, uint8_t* data, size_t len) {
    if (bus_fd_ < 0) return false;
    struct i2c_msg msgs[2];
    uint8_t reg_addr = reg;
    msgs[0].addr = i2c_addr_;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = &reg_addr;
    msgs[1].addr = i2c_addr_;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = len;
    msgs[1].buf = data;
    struct i2c_rdwr_ioctl_data msg_data;
    msg_data.msgs = msgs;
    msg_data.nmsgs = 2;
    return ioctl(bus_fd_, I2C_RDWR, &msg_data) >= 0;
}

void ZoeM8Q::readBus() {
    if (bus_fd_ < 0) return;
    
    // 1. Check how many bytes are available (registers 0xFD/0xFE)
    uint8_t count_buf[2];
    if (!readRegisters(0xFD, count_buf, 2)) return;
    
    uint16_t available = (static_cast<uint16_t>(count_buf[0]) << 8) | count_buf[1];
    if (available == 0 || available > 0x1FFF) return;

    // 2. Read available data (register 0xFF)
    // Read up to 128 bytes. (100 bytes is roughly one NAV-PVT packet)
    uint16_t to_read = std::min((uint16_t)128, available);
    uint8_t read_buf[128];
    
    if (readRegisters(0xFF, read_buf, to_read)) {
        buffer_.insert(buffer_.end(), read_buf, read_buf + to_read);
    }
}

std::vector<ubx::NavPvt> ZoeM8Q::poll() {
    readBus();
    if (buffer_.empty()) return {};
    return parseBuffer();
}

std::vector<ubx::NavPvt> ZoeM8Q::parseBuffer() {
    std::vector<ubx::NavPvt> packets;
    size_t i = 0;
    while (i + 8 <= buffer_.size()) {
        if (buffer_[i] != ubx::SYNC1 || buffer_[i+1] != ubx::SYNC2) {
            i++;
            continue;
        }
        
        uint8_t cls = buffer_[i+2];
        uint8_t id = buffer_[i+3];
        uint16_t len = buffer_[i+4] | (buffer_[i+5] << 8);
        
        size_t end = i + 6 + len + 2;
        if (end > buffer_.size()) {
            break; // Wait for more data
        }
        
        uint8_t cka, ckb;
        ubx::calculateChecksum(buffer_.data() + i + 2, len + 4, cka, ckb);
        
        if (cka == buffer_[i + 6 + len] && ckb == buffer_[i + 6 + len + 1]) {
            if (cls == 0x01 && id == 0x07 && len == sizeof(ubx::NavPvt)) {
                ubx::NavPvt pvt;
                std::memcpy(&pvt, buffer_.data() + i + 6, sizeof(ubx::NavPvt));
                packets.push_back(pvt);
            }
            i = end;
        } else {
            i++; // Checksum failed
        }
    }
    
    if (i > 0) {
        buffer_.erase(buffer_.begin(), buffer_.begin() + i);
    }
    
    return packets;
}

} // namespace zoe_m8q_ros

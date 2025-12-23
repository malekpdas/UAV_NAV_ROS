#ifndef ZOE_M8Q_ROS__ZOE_M8Q_HPP_
#define ZOE_M8Q_ROS__ZOE_M8Q_HPP_

#include <string>
#include <vector>
#include <optional>
#include <cstdint>
#include "zoe_m8q_ros/ubx.hpp"

namespace zoe_m8q_ros {

class ZoeM8Q {
public:
    static constexpr uint8_t DEFAULT_I2C_ADDR = 0x42;

    ZoeM8Q(int bus, uint8_t i2c_addr = DEFAULT_I2C_ADDR);
    ~ZoeM8Q();

    bool begin();
    bool configure(double rate_hz);
    std::vector<ubx::NavPvt> poll();

private:
    int bus_fd_;
    uint8_t i2c_addr_;
    std::vector<uint8_t> buffer_;

    bool writeUbx(uint8_t msgClass, uint8_t msgId, const std::vector<uint8_t>& payload);
    void readBus();
    std::vector<ubx::NavPvt> parseBuffer();
};

} // namespace zoe_m8q_ros

#endif // ZOE_M8Q_ROS__ZOE_M8Q_HPP_

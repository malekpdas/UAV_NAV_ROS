#ifndef LIDAR_LITE_ROS__LIDAR_LITE_HPP_
#define LIDAR_LITE_ROS__LIDAR_LITE_HPP_

#include <cstdint>
#include <string>
#include <map>
#include <optional>

namespace lidar_lite_ros {

struct LidarSettings {
    uint8_t sig_count_val;
    uint8_t acq_config_reg;
    uint8_t ref_count_val;
    uint8_t threshold_bypass;
};

class LidarLite {
public:
    static const std::map<std::string, LidarSettings> PRESETS;

    explicit LidarLite(int bus_id = 1, uint8_t address = 0x62);
    ~LidarLite();

    bool isConnected() const { return connected_; }
    bool configure(const LidarSettings& settings);
    std::optional<float> readDistance();
    void close();

private:
    bool writeRegister(uint8_t reg, uint8_t val);
    bool readRegister(uint8_t reg, uint8_t& val);
    bool readBlock(uint8_t reg, uint8_t* data, uint8_t len);

    int bus_id_;
    uint8_t address_;
    int fd_;
    bool connected_;

    static constexpr uint8_t ACQ_COMMAND = 0x00;
    static constexpr uint8_t STATUS_REG = 0x01;
    static constexpr uint8_t SIG_COUNT_VAL = 0x02;
    static constexpr uint8_t ACQ_CONFIG_REG = 0x04;
    static constexpr uint8_t THRESHOLD_BYPASS = 0x1c;
    static constexpr uint8_t REF_COUNT_VAL = 0x12;
    static constexpr uint8_t FULL_DELAY_HIGH = 0x8f;
};

} // namespace lidar_lite_ros

#endif // LIDAR_LITE_ROS__LIDAR_LITE_HPP_

#ifndef BMX160_ROS__BMX160_HPP_
#define BMX160_ROS__BMX160_HPP_

#include <vector>
#include <map>
#include <string>
#include <cstdint>
#include <optional>

namespace bmx160_ros {

enum class GyroRange : uint8_t {
    DPS_125 = 0,
    DPS_250 = 1,
    DPS_500 = 2,
    DPS_1000 = 3,
    DPS_2000 = 4
};

enum class AccelRange : uint8_t {
    G_2 = 0,
    G_4 = 1,
    G_8 = 2,
    G_16 = 3
};

struct BMX160Data {
    float mag[3];
    float gyro[3];
    float accel[3];
};

class BMX160 {
public:
    static constexpr uint8_t DEFAULT_I2C_ADDR = 0x68;

    BMX160(int bus, uint8_t i2c_addr = DEFAULT_I2C_ADDR);
    ~BMX160();

    bool begin();
    void setGyroRange(GyroRange range);
    void setAccelRange(AccelRange range);
    std::optional<BMX160Data> getAllData();
    bool softReset();
    void setLowPower();
    void wakeUp();

private:
    int bus_;
    uint8_t i2c_addr_;
    int fd_;
    bool initialized_;

    float gyro_sensitivity_;
    float accel_sensitivity_;

    // Register Addresses
    static constexpr uint8_t CHIP_ID_ADDR = 0x00;
    static constexpr uint8_t ERROR_REG_ADDR = 0x02;
    static constexpr uint8_t MAG_DATA_ADDR = 0x04;
    static constexpr uint8_t GYRO_DATA_ADDR = 0x0C;
    static constexpr uint8_t ACCEL_DATA_ADDR = 0x12;
    static constexpr uint8_t STATUS_ADDR = 0x1B;
    static constexpr uint8_t ACCEL_CONFIG_ADDR = 0x40;
    static constexpr uint8_t ACCEL_RANGE_ADDR = 0x41;
    static constexpr uint8_t GYRO_CONFIG_ADDR = 0x42;
    static constexpr uint8_t GYRO_RANGE_ADDR = 0x43;
    static constexpr uint8_t MAGN_CONFIG_ADDR = 0x44;
    static constexpr uint8_t MAGN_IF_0_ADDR = 0x4C;
    static constexpr uint8_t MAGN_IF_1_ADDR = 0x4D;
    static constexpr uint8_t MAGN_IF_2_ADDR = 0x4E;
    static constexpr uint8_t MAGN_IF_3_ADDR = 0x4F;
    static constexpr uint8_t COMMAND_REG_ADDR = 0x7E;

    // Commands
    static constexpr uint8_t SOFT_RESET_CMD = 0xB6;
    static constexpr uint8_t ACCEL_NORMAL_MODE_CMD = 0x11;
    static constexpr uint8_t GYRO_NORMAL_MODE_CMD = 0x15;
    static constexpr uint8_t MAG_NORMAL_MODE_CMD = 0x19;
    static constexpr uint8_t ACCEL_SUSPEND_MODE_CMD = 0x12;
    static constexpr uint8_t GYRO_SUSPEND_MODE_CMD = 0x17;
    static constexpr uint8_t MAG_SUSPEND_MODE_CMD = 0x1B;

    // Constants
    static constexpr float MAG_SENSITIVITY = 0.3f;
    static constexpr float GRAVITY_MS2 = 9.80665f;
    static constexpr uint8_t CHIP_ID_EXPECTED = 0xD8;

    static const std::map<GyroRange, float> GYRO_SENSITIVITIES;
    static const std::map<AccelRange, float> ACCEL_SENSITIVITIES;
    static const std::map<AccelRange, uint8_t> ACCEL_RANGE_REG_VALUES;

    void configureMagnetometer();
    int16_t parseSignedInt16(uint8_t msb, uint8_t lsb);
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t* data, uint8_t len);
};

} // namespace bmx160_ros

#endif // BMX160_ROS__BMX160_HPP_

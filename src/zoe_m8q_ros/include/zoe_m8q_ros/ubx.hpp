#ifndef ZOE_M8Q_ROS__UBX_HPP_
#define ZOE_M8Q_ROS__UBX_HPP_

#include <cstdint>
#include <vector>
#include <cstddef>

namespace zoe_m8q_ros {

namespace ubx {

#pragma pack(push, 1)
struct NavPvt {
    uint32_t iTOW;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t flags2;
    uint8_t numSV;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gSpeed;
    int32_t headMot;
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;
    uint8_t reserved1[6];
    int32_t headVeh;
    int16_t magDec;
    uint16_t magAcc;
};
#pragma pack(pop)

static constexpr uint8_t SYNC1 = 0xB5;
static constexpr uint8_t SYNC2 = 0x62;

struct UbxHeader {
    uint8_t sync1;
    uint8_t sync2;
    uint8_t msgClass;
    uint8_t msgId;
    uint16_t length;
};

inline void calculateChecksum(const uint8_t* data, size_t len, uint8_t& ck_a, uint8_t& ck_b) {
    ck_a = 0;
    ck_b = 0;
    for (size_t i = 0; i < len; ++i) {
        ck_a += data[i];
        ck_b += ck_a;
    }
}

} // namespace ubx

} // namespace zoe_m8q_ros

#endif // ZOE_M8Q_ROS__UBX_HPP_

#ifndef EKF2_ROS__UTILS_HPP_
#define EKF2_ROS__UTILS_HPP_

#include <cmath>
#include <vector>
#include <Eigen/Dense>

namespace ekf2_ros {
namespace utils {

/**
 * @brief Convert Latitude, Longitude, Altitude to local NED (North, East, Down) coordinates.
 * 
 * @param lat Current Latitude (deg)
 * @param lon Current Longitude (deg)
 * @param alt Current Altitude (m)
 * @param lat0 Origin Latitude (deg)
 * @param lon0 Origin Longitude (deg)
 * @param alt0 Origin Altitude (m)
 * @return Eigen::Vector3d [North, East, Down] in meters
 */
inline Eigen::Vector3d lla_to_ned(double lat, double lon, double alt, double lat0, double lon0, double alt0) {
    // WGS84 ellipsoid constants
    static constexpr double R = 6378137.0;  // Equatorial radius (m)
    static constexpr double f = 1.0 / 298.257223563;  // Flattening
    static constexpr double e2 = 2.0 * f - f * f;
    static constexpr double deg2rad = M_PI / 180.0;

    double lat_rad = lat * deg2rad;
    double lon_rad = lon * deg2rad;
    double lat0_rad = lat0 * deg2rad;
    double lon0_rad = lon0 * deg2rad;

    // Radius of curvature in prime vertical
    double sin_lat0 = std::sin(lat0_rad);
    double N = R / std::sqrt(1.0 - e2 * sin_lat0 * sin_lat0);
    // Radius of curvature in meridian
    double M = R * (1.0 - e2) / std::pow(1.0 - e2 * sin_lat0 * sin_lat0, 1.5);

    double north = (lat_rad - lat0_rad) * (M + alt0);
    double east = (lon_rad - lon0_rad) * (N + alt0) * std::cos(lat0_rad);
    double down = -(alt - alt0);

    return Eigen::Vector3d(north, east, down);
}

/**
 * @brief Wrap angle to [-pi, pi]
 */
inline double wrap_pi(double angle) {
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0) angle += 2.0 * M_PI;
    return angle - M_PI;
}

} // namespace utils
} // namespace ekf2_ros

#endif // EKF2_ROS__UTILS_HPP_

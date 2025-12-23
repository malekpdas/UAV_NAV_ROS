#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

#include "zoe_m8q_ros/zoe_m8q.hpp"

using namespace std::chrono_literals;

namespace zoe_m8q_ros {

class ZoeM8QNode : public rclcpp::Node {
public:
    ZoeM8QNode() : Node("zoe_m8q_node") {
        // Parameters
        this->declare_parameter("bus", 1);
        this->declare_parameter("rate_hz", 10.0);
        this->declare_parameter("frame_id", "gps_link");
        this->declare_parameter("debug", false);
        this->declare_parameter("pos_std", std::vector<double>{4.0, 4.0, 10.0});
        this->declare_parameter("vel_std", std::vector<double>{0.5});

        loadParameters();

        // Initialize Sensor
        gps_ = std::make_unique<ZoeM8Q>(bus_num_);
        if (gps_->begin()) {
            if (gps_->configure(rate_hz_)) {
                RCLCPP_INFO(this->get_logger(), "✅ Configured GPS: %.1f Hz, UBX-NAV-PVT only", rate_hz_);
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ GPS configuration FAILED");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ GPS init FAILED (check I2C)");
            throw std::runtime_error("GPS init failed");
        }

        // Publishers
        pub_fix_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);
        pub_vel_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("gps/vel", 10);

        // Timer
        double period = 1.0 / rate_hz_;
        timer_ = this->create_wall_timer(std::chrono::duration<double>(period), std::bind(&ZoeM8QNode::tick, this));
    }

private:
    void loadParameters() {
        bus_num_ = this->get_parameter("bus").as_int();
        rate_hz_ = this->get_parameter("rate_hz").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();
        debug_ = this->get_parameter("debug").as_bool();
        pos_std_ = this->get_parameter("pos_std").as_double_array();
        vel_std_ = this->get_parameter("vel_std").as_double_array();
    }

    void tick() {
        auto packets = gps_->poll();
        for (const auto& pvt : packets) {
            publishData(pvt);
        }
    }

    void publishData(const ubx::NavPvt& data) {
        auto timestamp = this->now();

        // 1. NavSatFix
        auto fix_msg = sensor_msgs::msg::NavSatFix();
        fix_msg.header.stamp = timestamp;
        fix_msg.header.frame_id = frame_id_;

        fix_msg.latitude = data.lat * 1e-7;
        fix_msg.longitude = data.lon * 1e-7;
        fix_msg.altitude = data.height * 1e-3;

        // Status
        if (data.fixType >= 2) {
            fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        } else {
            fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        }
        fix_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

        // Covariance
        double h_acc = data.hAcc * 1e-3;
        double v_acc = data.vAcc * 1e-3;

        if (h_acc <= 0) h_acc = pos_std_[0];
        if (v_acc <= 0) v_acc = pos_std_[2];

        double lat_cov = h_acc * h_acc;
        double lon_cov = h_acc * h_acc;
        double alt_cov = v_acc * v_acc;

        fix_msg.position_covariance[0] = lat_cov;
        fix_msg.position_covariance[4] = lon_cov;
        fix_msg.position_covariance[8] = alt_cov;
        fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

        pub_fix_->publish(fix_msg);

        // 2. Velocity
        auto vel_msg = geometry_msgs::msg::TwistWithCovarianceStamped();
        vel_msg.header.stamp = timestamp;
        vel_msg.header.frame_id = frame_id_;

        vel_msg.twist.twist.linear.x = data.velN * 1e-3;
        vel_msg.twist.twist.linear.y = data.velE * 1e-3;
        vel_msg.twist.twist.linear.z = data.velD * 1e-3;

        double s_acc = data.sAcc * 1e-3;
        if (s_acc <= 0) s_acc = vel_std_[0];
        double cov = s_acc * s_acc;

        vel_msg.twist.covariance[0] = cov;
        vel_msg.twist.covariance[7] = cov;
        vel_msg.twist.covariance[14] = cov;

        pub_vel_->publish(vel_msg);

        if (debug_ && data.fixType < 3) {
            RCLCPP_WARN(this->get_logger(), "Bad Fix: %d, Sats: %d", data.fixType, data.numSV);
        }
    }

    std::unique_ptr<ZoeM8Q> gps_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_fix_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_vel_;
    rclcpp::TimerBase::SharedPtr timer_;

    int bus_num_;
    double rate_hz_;
    std::string frame_id_;
    bool debug_;
    std::vector<double> pos_std_;
    std::vector<double> vel_std_;
};

} // namespace zoe_m8q_ros

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<zoe_m8q_ros::ZoeM8QNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

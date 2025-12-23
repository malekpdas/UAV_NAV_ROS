#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "lidar_lite_ros/lidar_lite.hpp"

using namespace std::chrono_literals;

namespace lidar_lite_ros {

class LidarLiteNode : public rclcpp::Node {
public:
    LidarLiteNode() : Node("lidar_lite_node") {
        // Declare ROS2 parameters
        this->declare_parameter("i2c_bus", 1);
        this->declare_parameter("frequency", 100.0);
        this->declare_parameter("frame_id", "lidar_link");
        this->declare_parameter("min_range", 0.05);
        this->declare_parameter("max_range", 40.0);
        this->declare_parameter("preset", "balanced");
        
        // Individual overrides
        this->declare_parameter("sig_count_val", -1);
        this->declare_parameter("acq_config_reg", -1);
        this->declare_parameter("threshold_bypass", -1);
        this->declare_parameter("ref_count_val", -1);

        // Load parameters
        int bus_id = this->get_parameter("i2c_bus").as_int();
        double freq = this->get_parameter("frequency").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();
        min_range_ = this->get_parameter("min_range").as_double();
        max_range_ = this->get_parameter("max_range").as_double();
        std::string preset_name = this->get_parameter("preset").as_string();

        // Determine final settings
        LidarSettings settings;
        auto it = LidarLite::PRESETS.find(preset_name);
        if (it != LidarLite::PRESETS.end()) {
            settings = it->second;
            RCLCPP_INFO(this->get_logger(), "Using Lidar Lite preset: '%s'", preset_name.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown preset '%s', falling back to 'balanced'", preset_name.c_str());
            settings = LidarLite::PRESETS.at("balanced");
        }

        // Apply overrides
        int val;
        val = this->get_parameter("sig_count_val").as_int();
        if (val != -1) settings.sig_count_val = static_cast<uint8_t>(val);
        
        val = this->get_parameter("acq_config_reg").as_int();
        if (val != -1) settings.acq_config_reg = static_cast<uint8_t>(val);
        
        val = this->get_parameter("threshold_bypass").as_int();
        if (val != -1) settings.threshold_bypass = static_cast<uint8_t>(val);
        
        val = this->get_parameter("ref_count_val").as_int();
        if (val != -1) settings.ref_count_val = static_cast<uint8_t>(val);

        // Initialize Hardware
        lidar_ = std::make_unique<LidarLite>(bus_id);
        if (!lidar_->isConnected()) {
            RCLCPP_ERROR(this->get_logger(), "Lidar Lite not detected on bus %d!", bus_id);
        } else {
            if (lidar_->configure(settings)) {
                RCLCPP_INFO(this->get_logger(), "Lidar Lite configured successfully");
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to apply some Lidar Lite settings");
            }
        }

        // Publisher
        pub_range_ = this->create_publisher<sensor_msgs::msg::Range>("lidar/range", 10);

        // Timer
        auto period = std::chrono::duration<double>(1.0 / freq);
        timer_ = this->create_wall_timer(period, std::bind(&LidarLiteNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Lidar Lite Node started. Bus: %d, Freq: %.1fHz", bus_id, freq);
    }

private:
    void timer_callback() {
        auto dist = lidar_->readDistance();
        if (dist.has_value()) {
            auto msg = sensor_msgs::msg::Range();
            msg.header.stamp = this->now();
            msg.header.frame_id = frame_id_;
            msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
            msg.field_of_view = 0.01f; // Approx 8mrad ~ 0.5 deg
            msg.min_range = static_cast<float>(min_range_);
            msg.max_range = static_cast<float>(max_range_);
            msg.range = dist.value();

            pub_range_->publish(msg);
        }
    }

    std::unique_ptr<LidarLite> lidar_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_range_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string frame_id_;
    double min_range_;
    double max_range_;
};

} // namespace lidar_lite_ros

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lidar_lite_ros::LidarLiteNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

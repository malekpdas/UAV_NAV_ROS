#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"

#include "bmx160_ros/bmx160.hpp"

using namespace std::chrono_literals;

namespace bmx160_ros {

class BMX160Node : public rclcpp::Node {
public:
    BMX160Node() : Node("bmx160_node") {
        // Parameters
        this->declare_parameter("bus", 1);
        this->declare_parameter("rate_hz", 100.0);
        this->declare_parameter("frame_id", "imu_link");
        this->declare_parameter("bias_removal", true);
        this->declare_parameter("bias_duration_sec", 2.0);
        this->declare_parameter("accel_bias", std::vector<double>{0.0, 0.0, 0.0});
        this->declare_parameter("gyro_bias", std::vector<double>{0.0, 0.0, 0.0});
        
        this->declare_parameter("accel_variance", std::vector<double>{0.001, 0.001, 0.001});
        this->declare_parameter("gyro_variance", std::vector<double>{0.0001, 0.0001, 0.0001});
        this->declare_parameter("mag_variance", std::vector<double>{1e-3, 1e-3, 1e-3});
        this->declare_parameter("mag_declination_angle", 0.0);
        
        this->declare_parameter("mag_bias", std::vector<double>{0.0, 0.0, 0.0});
        this->declare_parameter("mag_transform", std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
        this->declare_parameter("imu_rotation_transformation", std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});

        loadParameters();

        // Initialize Sensor
        imu_ = std::make_unique<BMX160>(bus_);
        if (imu_->begin()) {
            RCLCPP_INFO(this->get_logger(), "✅ BMX160 init OK");
            imu_->setGyroRange(GyroRange::DPS_500);
            imu_->setAccelRange(AccelRange::G_4);
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ BMX160 init FAILED (check I2C)");
            throw std::runtime_error("BMX160 init failed");
        }

        // Publishers
        pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        pub_mag_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);

        // Calibration State
        calibrating_ = bias_removal_;
        if (calibrating_) {
            RCLCPP_INFO(this->get_logger(), "⚠ Starting Bias Calibration (%.1fs). Keep vehicle LEVEL and STATIONARY.", bias_duration_sec_);
            calibration_end_time_ = this->now().seconds() + bias_duration_sec_;
        }

        // Parameter Callback
        param_event_sub_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
            "/parameter_events", 10, std::bind(&BMX160Node::onParamEvent, this, std::placeholders::_1));

        // Timer
        double period = 1.0 / rate_hz_;
        timer_ = this->create_wall_timer(std::chrono::duration<double>(period), std::bind(&BMX160Node::tick, this));
    }

private:
    void loadParameters() {
        bus_ = this->get_parameter("bus").as_int();
        rate_hz_ = this->get_parameter("rate_hz").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();
        bias_removal_ = this->get_parameter("bias_removal").as_bool();
        bias_duration_sec_ = this->get_parameter("bias_duration_sec").as_double();
        mag_dec_angle_ = this->get_parameter("mag_declination_angle").as_double();

        auto ab = this->get_parameter("accel_bias").as_double_array();
        for (int i = 0; i < 3; ++i) accel_bias_[i] = ab[i];

        auto gb = this->get_parameter("gyro_bias").as_double_array();
        for (int i = 0; i < 3; ++i) gyro_bias_[i] = gb[i];

        auto av = this->get_parameter("accel_variance").as_double_array();
        for (int i = 0; i < 3; ++i) accel_variance_[i] = av[i];

        auto gv = this->get_parameter("gyro_variance").as_double_array();
        for (int i = 0; i < 3; ++i) gyro_variance_[i] = gv[i];

        auto mv = this->get_parameter("mag_variance").as_double_array();
        for (int i = 0; i < 3; ++i) mag_variance_[i] = mv[i];

        auto mb = this->get_parameter("mag_bias").as_double_array();
        for (int i = 0; i < 3; ++i) mag_bias_[i] = mb[i];

        auto mt = this->get_parameter("mag_transform").as_double_array();
        for (int i = 0; i < 9; ++i) mag_transform_[i/3][i%3] = mt[i];

        auto ir = this->get_parameter("imu_rotation_transformation").as_double_array();
        for (int i = 0; i < 9; ++i) imu_rot_[i/3][i%3] = ir[i];

        updateMagDeclinationMatrix();
    }

    void updateMagDeclinationMatrix() {
        double rad = mag_dec_angle_ * M_PI / 180.0;
        mag_dec_rot_[0][0] = std::cos(rad);
        mag_dec_rot_[0][1] = -std::sin(rad);
        mag_dec_rot_[0][2] = 0;
        mag_dec_rot_[1][0] = std::sin(rad);
        mag_dec_rot_[1][1] = std::cos(rad);
        mag_dec_rot_[1][2] = 0;
        mag_dec_rot_[2][0] = 0;
        mag_dec_rot_[2][1] = 0;
        mag_dec_rot_[2][2] = 1;
    }

    void tick() {
        auto data = imu_->getAllData();
        if (!data) return;

        if (calibrating_) {
            double now = this->now().seconds();
            if (now < calibration_end_time_) {
                calib_samples_accel_.push_back({data->accel[0], data->accel[1], data->accel[2]});
                calib_samples_gyro_.push_back({data->gyro[0] * (float)M_PI / 180.0f, 
                                               data->gyro[1] * (float)M_PI / 180.0f, 
                                               data->gyro[2] * (float)M_PI / 180.0f});
            } else {
                finishCalibration();
            }
            return;
        }

        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = frame_id_;

        // Apply bias (Sensor Frame)
        double ax = data->accel[0] - accel_bias_[0];
        double ay = data->accel[1] - accel_bias_[1];
        double az = data->accel[2] - accel_bias_[2];

        double gx = (data->gyro[0] * M_PI / 180.0) - gyro_bias_[0];
        double gy = (data->gyro[1] * M_PI / 180.0) - gyro_bias_[1];
        double gz = (data->gyro[2] * M_PI / 180.0) - gyro_bias_[2];

        // Apply Rotation (Mounting -> Body)
        double accel_body[3] = {0, 0, 0};
        double gyro_body[3] = {0, 0, 0};
        for (int i = 0; i < 3; ++i) {
            accel_body[i] = imu_rot_[i][0] * ax + imu_rot_[i][1] * ay + imu_rot_[i][2] * az;
            gyro_body[i] = imu_rot_[i][0] * gx + imu_rot_[i][1] * gy + imu_rot_[i][2] * gz;
        }

        imu_msg.linear_acceleration.x = accel_body[0];
        imu_msg.linear_acceleration.y = accel_body[1];
        imu_msg.linear_acceleration.z = accel_body[2];
        imu_msg.angular_velocity.x = gyro_body[0];
        imu_msg.angular_velocity.y = gyro_body[1];
        imu_msg.angular_velocity.z = gyro_body[2];

        imu_msg.orientation_covariance[0] = -1.0;
        for (int i = 0; i < 3; ++i) {
            imu_msg.linear_acceleration_covariance[i*4] = accel_variance_[i];
            imu_msg.angular_velocity_covariance[i*4] = gyro_variance_[i];
        }

        pub_imu_->publish(imu_msg);

        // Magnetometer
        auto mag_msg = sensor_msgs::msg::MagneticField();
        mag_msg.header.stamp = imu_msg.header.stamp;
        mag_msg.header.frame_id = frame_id_;

        double mx = data->mag[0];
        double my = data->mag[1];
        double mz = data->mag[2];

        // Apply Rotation (Mounting -> Body)
        double mx_body = imu_rot_[0][0] * mx + imu_rot_[0][1] * my + imu_rot_[0][2] * mz;
        double my_body = imu_rot_[1][0] * mx + imu_rot_[1][1] * my + imu_rot_[1][2] * mz;
        double mz_body = imu_rot_[2][0] * mx + imu_rot_[2][1] * my + imu_rot_[2][2] * mz;

        // Apply Mag Bias (In Body frame matching Python logic)
        double mb_body[3] = {0, 0, 0};
        for (int i = 0; i < 3; ++i) {
            mb_body[i] = imu_rot_[i][0] * mag_bias_[0] + imu_rot_[i][1] * mag_bias_[1] + imu_rot_[i][2] * mag_bias_[2];
        }

        double dx = mx_body - mb_body[0];
        double dy = my_body - mb_body[1];
        double dz = mz_body - mb_body[2];

        // Apply Soft Iron (mag_transform)
        double m_calib[3] = {0, 0, 0};
        for (int i = 0; i < 3; ++i) {
            m_calib[i] = mag_transform_[i][0] * dx + mag_transform_[i][1] * dy + mag_transform_[i][2] * dz;
        }

        // Apply Declination (Transpose of mag_dec_rot per Python code)
        double m_true[3] = {0, 0, 0};
        for (int i = 0; i < 3; ++i) {
            m_true[i] = mag_dec_rot_[0][i] * m_calib[0] + mag_dec_rot_[1][i] * m_calib[1] + mag_dec_rot_[2][i] * m_calib[2];
        }

        mag_msg.magnetic_field.x = m_true[0];
        mag_msg.magnetic_field.y = m_true[1];
        mag_msg.magnetic_field.z = m_true[2];
        for (int i = 0; i < 3; ++i) mag_msg.magnetic_field_covariance[i*4] = mag_variance_[i];

        pub_mag_->publish(mag_msg);
    }

    void finishCalibration() {
        if (calib_samples_accel_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Calibration failed: no data collected.");
            calibrating_ = false;
            return;
        }

        // Gyro bias is simple mean
        std::vector<double> sum_gyro = {0, 0, 0};
        for (const auto& s : calib_samples_gyro_) {
            sum_gyro[0] += s[0]; sum_gyro[1] += s[1]; sum_gyro[2] += s[2];
        }
        for (int i = 0; i < 3; ++i) gyro_bias_[i] = sum_gyro[i] / calib_samples_gyro_.size();

        // Accel bias assumes Z is gravity
        // gravity_sensor = imu_rot^T * [0, 0, 9.80665]
        double gravity_sensor[3] = {0, 0, 0};
        for (int i = 0; i < 3; ++i) gravity_sensor[i] = imu_rot_[0][i] * 0 + imu_rot_[1][i] * 0 + imu_rot_[2][i] * 9.80665;

        // bias = measured - gravity
        std::vector<double> sum_accel = {0, 0, 0};
        for (const auto& s : calib_samples_accel_) {
            sum_accel[0] += s[0] + gravity_sensor[0];
            sum_accel[1] += s[1] + gravity_sensor[1];
            sum_accel[2] += s[2] + gravity_sensor[2];
        }
        for (int i = 0; i < 3; ++i) accel_bias_[i] = sum_accel[i] / calib_samples_accel_.size();

        RCLCPP_INFO(this->get_logger(), "Calibration Done. Gyro Bias: [%.4f, %.4f, %.4f], Accel Bias: [%.4f, %.4f, %.4f]",
                    gyro_bias_[0], gyro_bias_[1], gyro_bias_[2], accel_bias_[0], accel_bias_[1], accel_bias_[2]);

        calibrating_ = false;
        calib_samples_accel_.clear();
        calib_samples_gyro_.clear();
    }

    void onParamEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
        if (event->node != this->get_fully_qualified_name()) return;

        for (auto& param : event->changed_parameters) {
            if (param.name == "mag_bias") {
                auto val = param.value.double_array_value;
                if (val.size() == 3) for (int i = 0; i < 3; ++i) mag_bias_[i] = val[i];
            } else if (param.name == "mag_transform") {
                auto val = param.value.double_array_value;
                if (val.size() == 9) for (int i = 0; i < 9; ++i) mag_transform_[i/3][i%3] = val[i];
            } else if (param.name == "imu_rotation_transformation") {
                auto val = param.value.double_array_value;
                if (val.size() == 9) for (int i = 0; i < 9; ++i) imu_rot_[i/3][i%3] = val[i];
            } else if (param.name == "accel_bias") {
                auto val = param.value.double_array_value;
                if (val.size() == 3) for (int i = 0; i < 3; ++i) accel_bias_[i] = val[i];
            } else if (param.name == "gyro_bias") {
                auto val = param.value.double_array_value;
                if (val.size() == 3) for (int i = 0; i < 3; ++i) gyro_bias_[i] = val[i];
            } else if (param.name == "mag_declination_angle") {
                mag_dec_angle_ = param.value.double_value;
                updateMagDeclinationMatrix();
            }
        }
    }

    std::unique_ptr<BMX160> imu_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_event_sub_;

    int bus_;
    double rate_hz_;
    std::string frame_id_;
    bool bias_removal_;
    double bias_duration_sec_;
    double mag_dec_angle_;

    double accel_bias_[3] = {0, 0, 0};
    double gyro_bias_[3] = {0, 0, 0};
    double mag_bias_[3] = {0, 0, 0};
    double accel_variance_[3], gyro_variance_[3], mag_variance_[3];
    double mag_transform_[3][3], imu_rot_[3][3], mag_dec_rot_[3][3];

    bool calibrating_;
    double calibration_end_time_;
    std::vector<std::vector<double>> calib_samples_accel_, calib_samples_gyro_;
};

} // namespace bmx160_ros

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bmx160_ros::BMX160Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

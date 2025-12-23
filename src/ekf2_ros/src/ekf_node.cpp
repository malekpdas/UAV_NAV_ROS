#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "ekf2_ros/ekf_core.hpp"
#include "ekf2_ros/utils.hpp"
#include "Fusion/Fusion.h"

using namespace std::chrono_literals;

namespace ekf2_ros {

class EkfNode : public rclcpp::Node {
public:
    EkfNode() : Node("ekf_node"), kf_initialized_(false), origin_set_(false) {
        // Parameters
        declare_all_parameters();
        load_parameters();

        // AHRS Setup
        FusionAhrsInitialise(&ahrs_);
        FusionAhrsSettings settings;
        settings.convention = FusionConventionNed;
        settings.gain = static_cast<float>(params_.ahrs_gain);
        settings.gyroscopeRange = static_cast<float>(params_.ahrs_gyro_range);
        settings.accelerationRejection = static_cast<float>(params_.ahrs_accel_rejection);
        settings.magneticRejection = static_cast<float>(params_.ahrs_mag_rejection);
        settings.recoveryTriggerPeriod = static_cast<unsigned int>(params_.ahrs_rejection_timeout);
        FusionAhrsSetSettings(&ahrs_, &settings);
        
        FusionBiasInitialise(&bias_, static_cast<float>(1.0f / 100.0f)); // Assuming ~100Hz

        // Kalman Filter Setup
        KFKConfig kf_config;
        kf_config.initial_pos_uncertainty = params_.kf_initial_pos_uncertainty;
        kf_config.initial_vel_uncertainty = params_.kf_initial_vel_uncertainty;
        kf_config.initial_bias_uncertainty = params_.kf_initial_bias_uncertainty;
        kf_config.process_noise_pos = params_.kf_process_noise_pos;
        kf_config.process_noise_vel = params_.kf_process_noise_vel;
        kf_config.process_noise_bias = params_.kf_process_noise_bias;
        kf_config.measurement_noise_pos = params_.kf_measurement_noise_pos;
        kf_config.measurement_noise_vel = params_.kf_measurement_noise_vel;

        kf_ = std::make_unique<LinearKalmanFilter>(kf_config);

        // Publishers
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(params_.topic_odometry, 100);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        if (params_.topic_publish_acceleration) {
            accel_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(params_.topic_filtered_imu, 100);
        }

        // Subscriptions
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            params_.topic_imu, 100, std::bind(&EkfNode::cb_imu, this, std::placeholders::_1));
        mag_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
            params_.topic_mag, 100, std::bind(&EkfNode::cb_mag, this, std::placeholders::_1));
        gps_fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            params_.topic_gps_fix, 10, std::bind(&EkfNode::cb_gps_fix, this, std::placeholders::_1));
        gps_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            params_.topic_gps_vel, 10, std::bind(&EkfNode::cb_gps_vel, this, std::placeholders::_1));

        // Filters
        lp_accel_ = std::make_unique<RobustLPFilter>(params_.filter_lowpass_alpha_acc);

        RCLCPP_INFO(this->get_logger(), "IMU/GPS Fusion Node (C++) initialized");
        log_parameters();
    }

private:
    struct Params {
        std::string frame_map, frame_odom, frame_base;
        std::string topic_imu, topic_mag, topic_gps_fix, topic_gps_vel, topic_odometry, topic_filtered_imu;
        bool topic_publish_acceleration;
        double ahrs_gain, ahrs_gyro_range, ahrs_accel_rejection, ahrs_mag_rejection;
        int ahrs_rejection_timeout;
        double kf_initial_pos_uncertainty, kf_initial_vel_uncertainty, kf_initial_bias_uncertainty;
        double kf_process_noise_pos, kf_process_noise_vel, kf_process_noise_bias;
        double kf_measurement_noise_pos, kf_measurement_noise_vel;
        double filter_lowpass_alpha_acc;
        double earth_gravity;
    } params_;

    void declare_all_parameters() {
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("base_link_frame", "base_link");
        this->declare_parameter("topics.imu", "/imu/data");
        this->declare_parameter("topics.mag", "/imu/mag");
        this->declare_parameter("topics.gps_fix", "/gps/fix");
        this->declare_parameter("topics.gps_vel", "/gps/vel");
        this->declare_parameter("topics.odometry", "/ekf/odometry");
        this->declare_parameter("topics.filtered_imu", "/ekf/filtered_imu");
        this->declare_parameter("topics.publish_acceleration", false);
        this->declare_parameter("ahrs.gain", 0.5);
        this->declare_parameter("ahrs.gyro_range", 500.0);
        this->declare_parameter("ahrs.accel_rejection", 10.0);
        this->declare_parameter("ahrs.mag_rejection", 10.0);
        this->declare_parameter("ahrs.rejection_timeout", 500);
        this->declare_parameter("kalman_filter.initial_pos_uncertainty", 100.0);
        this->declare_parameter("kalman_filter.initial_vel_uncertainty", 100.0);
        this->declare_parameter("kalman_filter.initial_bias_uncertainty", 1.0);
        this->declare_parameter("kalman_filter.process_noise_pos", 0.5);
        this->declare_parameter("kalman_filter.process_noise_vel", 0.5);
        this->declare_parameter("kalman_filter.process_noise_bias", 0.001);
        this->declare_parameter("kalman_filter.measurement_noise_pos", 25.0);
        this->declare_parameter("kalman_filter.measurement_noise_vel", 0.25);
        this->declare_parameter("filters.lowpass_alpha_acc", 0.9);
        this->declare_parameter("earth.gravity", 9.8066);
    }

    void load_parameters() {
        params_.frame_map = this->get_parameter("map_frame").as_string();
        params_.frame_odom = this->get_parameter("odom_frame").as_string();
        params_.frame_base = this->get_parameter("base_link_frame").as_string();
        params_.topic_imu = this->get_parameter("topics.imu").as_string();
        params_.topic_mag = this->get_parameter("topics.mag").as_string();
        params_.topic_gps_fix = this->get_parameter("topics.gps_fix").as_string();
        params_.topic_gps_vel = this->get_parameter("topics.gps_vel").as_string();
        params_.topic_odometry = this->get_parameter("topics.odometry").as_string();
        params_.topic_filtered_imu = this->get_parameter("topics.filtered_imu").as_string();
        params_.topic_publish_acceleration = this->get_parameter("topics.publish_acceleration").as_bool();
        params_.ahrs_gain = this->get_parameter("ahrs.gain").as_double();
        params_.ahrs_gyro_range = this->get_parameter("ahrs.gyro_range").as_double();
        params_.ahrs_accel_rejection = this->get_parameter("ahrs.accel_rejection").as_double();
        params_.ahrs_mag_rejection = this->get_parameter("ahrs.mag_rejection").as_double();
        params_.ahrs_rejection_timeout = this->get_parameter("ahrs.rejection_timeout").as_int();
        params_.kf_initial_pos_uncertainty = this->get_parameter("kalman_filter.initial_pos_uncertainty").as_double();
        params_.kf_initial_vel_uncertainty = this->get_parameter("kalman_filter.initial_vel_uncertainty").as_double();
        params_.kf_initial_bias_uncertainty = this->get_parameter("kalman_filter.initial_bias_uncertainty").as_double();
        params_.kf_process_noise_pos = this->get_parameter("kalman_filter.process_noise_pos").as_double();
        params_.kf_process_noise_vel = this->get_parameter("kalman_filter.process_noise_vel").as_double();
        params_.kf_process_noise_bias = this->get_parameter("kalman_filter.process_noise_bias").as_double();
        params_.kf_measurement_noise_pos = this->get_parameter("kalman_filter.measurement_noise_pos").as_double();
        params_.kf_measurement_noise_vel = this->get_parameter("kalman_filter.measurement_noise_vel").as_double();
        params_.filter_lowpass_alpha_acc = this->get_parameter("filters.lowpass_alpha_acc").as_double();
        params_.earth_gravity = this->get_parameter("earth.gravity").as_double();
    }

    void log_parameters() {
        RCLCPP_INFO(this->get_logger(), "=== Configuration (C++) ===");
        RCLCPP_INFO(this->get_logger(), "Frames: %s -> %s", params_.frame_odom.c_str(), params_.frame_base.c_str());
        RCLCPP_INFO(this->get_logger(), "IMU Topic: %s", params_.topic_imu.c_str());
        RCLCPP_INFO(this->get_logger(), "GPS Fix Topic: %s", params_.topic_gps_fix.c_str());
        RCLCPP_INFO(this->get_logger(), "GPS Vel Topic: %s", params_.topic_gps_vel.c_str());
        RCLCPP_INFO(this->get_logger(), "Odometry Topic: %s", params_.topic_odometry.c_str());
        RCLCPP_INFO(this->get_logger(), "Publish Filtered Accel: %s", params_.topic_publish_acceleration ? "True" : "False");
        RCLCPP_INFO(this->get_logger(), "AHRS Gain: %.2f", params_.ahrs_gain);
        RCLCPP_INFO(this->get_logger(), "KF Process Noise (pos): %.2f", params_.kf_process_noise_pos);
        RCLCPP_INFO(this->get_logger(), "KF Measurement Noise (pos): %.2f", params_.kf_measurement_noise_pos);
    }

    void cb_mag(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
        latest_mag_ = {
            static_cast<float>(msg->magnetic_field.x),
            static_cast<float>(msg->magnetic_field.y),
            static_cast<float>(msg->magnetic_field.z)
        };
        mag_available_ = true;
    }

    void cb_gps_vel(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
        Eigen::Vector3d gps_vel(
            msg->twist.twist.linear.x,  // North (mapping linear.y to North?) -> Re-checking Python code
            msg->twist.twist.linear.y,  // East
            msg->twist.twist.linear.z  // Down
        );
        latest_gps_vel_ = gps_vel;
        gps_vel_available_ = true;

        if (kf_initialized_) {
            Eigen::Matrix3d R_vel = Eigen::Matrix3d::Identity() * 0.1;
            // Using diagonal from message if available
            R_vel(0,0) = std::max(0.1, msg->twist.covariance[0]); // linear.x
            R_vel(1,1) = std::max(0.1, msg->twist.covariance[7]); // linear.y
            R_vel(2,2) = std::max(0.1, msg->twist.covariance[14]); // linear.z
            kf_->update_velocity(gps_vel, R_vel);
        }
    }

    void cb_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (!mag_available_) return;

        double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        if (last_imu_time_ < 0) {
            last_imu_time_ = t;
            return;
        }

        double dt = t - last_imu_time_;
        if (dt <= 0) return;

        // Fusion vectors
        FusionVector gyro = {
            static_cast<float>(msg->angular_velocity.x * 180.0 / M_PI),
            static_cast<float>(msg->angular_velocity.y * 180.0 / M_PI),
            static_cast<float>(msg->angular_velocity.z * 180.0 / M_PI)
        };
        FusionVector accel = {
            static_cast<float>(msg->linear_acceleration.x),
            static_cast<float>(msg->linear_acceleration.y),
            static_cast<float>(msg->linear_acceleration.z)
        };

        // Offset/Bias correction (optional but good)
        // gyro = FusionBiasUpdate(&bias_, gyro);

        // Update AHRS
        FusionAhrsUpdate(&ahrs_, gyro, accel, latest_mag_, static_cast<float>(dt));

        // Earth Acceleration (NED)
        FusionVector earth_acc = FusionAhrsGetEarthAcceleration(&ahrs_);
        Eigen::Vector3d a_ned(earth_acc.axis.x, earth_acc.axis.y, earth_acc.axis.z);
        a_ned(2) += params_.earth_gravity; // Add gravity to get total acceleration in NED

        // Low-pass filter for accel
        a_ned = lp_accel_->update(a_ned);

        if (kf_initialized_) {
            kf_->predict(a_ned, dt);
        }

        current_gyro_ = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
        last_imu_time_ = t;

        publish_odometry(msg->header.stamp);
    }

    void cb_gps_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (std::isnan(msg->latitude) || !gps_vel_available_) return;

        if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX){
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                     "GPS has NO FIX! Ignoring data...");
            return;
        };

        if (!origin_set_) {
            origin_ = {msg->latitude, msg->longitude, msg->altitude};
            origin_set_ = true;
            kf_->initialize(Eigen::Vector3d::Zero(), latest_gps_vel_);
            kf_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Filter initialized with first GPS fix at [%.6f, %.6f]", origin_[0], origin_[1]);
            return;
        }

        Eigen::Vector3d pos_gps = utils::lla_to_ned(msg->latitude, msg->longitude, msg->altitude, origin_[0], origin_[1], origin_[2]);
        
        Eigen::Matrix3d R_pos;
        for (int i=0; i<3; ++i) {
            for (int j=0; j<3; ++j) {
                R_pos(i,j) = msg->position_covariance[i*3 + j];
            }
        }
        // Ensure R is not too small
        R_pos = R_pos.array().max(0.5).matrix();

        if (kf_initialized_) {
            kf_->update_position(pos_gps, R_pos);
        }
    }

    void publish_odometry(const rclcpp::Time& stamp) {
        if (!kf_initialized_) return;
        
        Eigen::Vector3d pos = kf_->get_position();
        Eigen::Vector3d vel = kf_->get_velocity();
        Eigen::Matrix<double, 9, 9> P = kf_->get_covariance();
        FusionQuaternion q = FusionAhrsGetQuaternion(&ahrs_);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = params_.frame_odom;
        odom.child_frame_id = params_.frame_base;

        odom.pose.pose.position.x = pos(0);
        odom.pose.pose.position.y = pos(1);
        odom.pose.pose.position.z = pos(2);

        odom.pose.pose.orientation.x = q.element.x;
        odom.pose.pose.orientation.y = q.element.y;
        odom.pose.pose.orientation.z = q.element.z;
        odom.pose.pose.orientation.w = q.element.w;

        odom.pose.covariance[0] = P(0,0);
        odom.pose.covariance[7] = P(1,1);
        odom.pose.covariance[14] = P(2,2);

        odom.twist.twist.linear.x = vel(0);
        odom.twist.twist.linear.y = vel(1);
        odom.twist.twist.linear.z = vel(2);

        odom.twist.covariance[0] = P(3,3);
        odom.twist.covariance[7] = P(4,4);
        odom.twist.covariance[14] = P(5,5);

        odom.twist.twist.angular.x = current_gyro_[0];
        odom.twist.twist.angular.y = current_gyro_[1];
        odom.twist.twist.angular.z = current_gyro_[2];

        odom_pub_->publish(odom);

        // TF
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = params_.frame_odom;
        t.child_frame_id = params_.frame_base;
        t.transform.translation.x = pos(0);
        t.transform.translation.y = pos(1);
        t.transform.translation.z = pos(2);
        t.transform.rotation.x = q.element.x;
        t.transform.rotation.y = q.element.y;
        t.transform.rotation.z = q.element.z;
        t.transform.rotation.w = q.element.w;
        tf_broadcaster_->sendTransform(t);
    }

    // State
    FusionAhrs ahrs_;
    FusionBias bias_;
    FusionVector latest_mag_;
    bool mag_available_ = false;

    Eigen::Vector3d latest_gps_vel_;
    bool gps_vel_available_ = false;

    std::unique_ptr<LinearKalmanFilter> kf_;
    bool kf_initialized_;
    
    std::vector<double> origin_;
    bool origin_set_;

    double last_imu_time_ = -1.0;
    std::vector<double> current_gyro_ = {0,0,0};

    std::unique_ptr<RobustLPFilter> lp_accel_;

    // ROS
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr accel_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_fix_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr gps_vel_sub_;
};

} // namespace ekf2_ros

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ekf2_ros::EkfNode>());
    rclcpp::shutdown();
    return 0;
}

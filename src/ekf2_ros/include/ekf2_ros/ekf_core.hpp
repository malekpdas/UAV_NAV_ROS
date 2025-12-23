#ifndef EKF2_ROS__EKF_CORE_HPP_
#define EKF2_ROS__EKF_CORE_HPP_

#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>

namespace ekf2_ros {

/**
 * @brief Robust Low-pass filter for sensor data smoothing.
 */
class RobustLPFilter {
public:
    RobustLPFilter(double alpha = 0.95, double max_delta = 2.0)
        : alpha_(alpha), max_delta_(max_delta), initialized_(false) {
        prev_ = Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d update(const Eigen::Vector3d& current) {
        // Check for NaN
        if (std::isnan(current(0)) || std::isnan(current(1)) || std::isnan(current(2))) {
            std::cerr << "RobustLPFilter: NaN detected in input!" << std::endl;
            return prev_;
        }

        if (!initialized_) {
            prev_ = current;
            initialized_ = true;
            return prev_;
        }

        Eigen::Vector3d delta = current - prev_;
        // Component-wise clip
        delta(0) = std::clamp(delta(0), -max_delta_, max_delta_);
        delta(1) = std::clamp(delta(1), -max_delta_, max_delta_);
        delta(2) = std::clamp(delta(2), -max_delta_, max_delta_);

        Eigen::Vector3d filtered = prev_ + delta;
        prev_ = alpha_ * prev_ + (1.0 - alpha_) * filtered;
        
        return prev_;
    }

private:
    double alpha_;
    double max_delta_;
    bool initialized_;
    Eigen::Vector3d prev_;
};

/**
 * @brief Configuration for Linear Kalman Filter.
 */
struct KFKConfig {
    double initial_pos_uncertainty = 100.0;
    double initial_vel_uncertainty = 100.0;
    double initial_bias_uncertainty = 1.0;
    double process_noise_pos = 0.1;
    double process_noise_vel = 0.01;
    double process_noise_bias = 0.001;
    double measurement_noise_pos = 25.0;
    double measurement_noise_vel = 0.25;
};

/**
 * @brief Linear Kalman Filter for IMU/GPS integration.
 * State (9D): [pn, pe, pd, vn, ve, vd, ban, bae, bad]
 */
class LinearKalmanFilter {
public:
    LinearKalmanFilter(const KFKConfig& config) : config_(config) {
        x_ = Eigen::Matrix<double, 9, 1>::Zero();
        P_ = Eigen::Matrix<double, 9, 9>::Identity();
        P_.block<3, 3>(0, 0) *= config_.initial_pos_uncertainty;
        P_.block<3, 3>(3, 3) *= config_.initial_vel_uncertainty;
        P_.block<3, 3>(6, 6) *= config_.initial_bias_uncertainty;

        Q_ = Eigen::Matrix<double, 9, 9>::Identity();
        Q_.block<3, 3>(0, 0) *= config_.process_noise_pos;
        Q_.block<3, 3>(3, 3) *= config_.process_noise_vel;
        Q_.block<3, 3>(6, 6) *= config_.process_noise_bias;
    }

    void initialize(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel) {
        x_.segment<3>(0) = pos;
        x_.segment<3>(3) = vel;
        x_.segment<3>(6).setZero();
    }

    bool predict(const Eigen::Vector3d& a_measured, double dt) {
        // Check inputs for NaN
        if (std::isnan(a_measured(0)) || std::isnan(a_measured(1)) || std::isnan(a_measured(2))) {
            std::cerr << "KF Predict: NaN in acceleration measurement!" << std::endl;
            return false;
        }
        if (std::isnan(dt) || dt <= 0 || dt > 1.0) {
            std::cerr << "KF Predict: Invalid dt: " << dt << std::endl;
            return false;
        }

        Eigen::Vector3d p = x_.segment<3>(0);
        Eigen::Vector3d v = x_.segment<3>(3);
        Eigen::Vector3d ba = x_.segment<3>(6);

        Eigen::Vector3d a_corrected = a_measured - ba;

        // Transition matrix F
        Eigen::Matrix<double, 9, 9> F = Eigen::Matrix<double, 9, 9>::Identity();
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
        F.block<3, 3>(0, 6) = -0.5 * dt * dt * Eigen::Matrix3d::Identity();
        F.block<3, 3>(3, 6) = -dt * Eigen::Matrix3d::Identity();

        // State update
        x_.segment<3>(0) = p + v * dt + 0.5 * a_corrected * dt * dt;
        x_.segment<3>(3) = v + a_corrected * dt;
        // x_.segment<3>(6) stays same (bias)

        // Covariance update
        P_ = F * P_ * F.transpose() + Q_;

        // Ensure P remains symmetric and positive definite
        P_ = 0.5 * (P_ + P_.transpose());
        
        // Add small regularization to maintain numerical stability
        P_ += Eigen::Matrix<double, 9, 9>::Identity() * 1e-9;

        // Check for NaN in state
        if (has_nan(x_) || has_nan_matrix(P_)) {
            std::cerr << "KF Predict: NaN detected in state or covariance!" << std::endl;
            return false;
        }

        return true;
    }

    bool update_position(const Eigen::Vector3d& z_pos, const Eigen::Matrix3d& R) {
        // Check inputs
        if (has_nan(z_pos) || has_nan_matrix(R)) {
            std::cerr << "KF Update Position: NaN in measurement or covariance!" << std::endl;
            return false;
        }

        Eigen::Matrix<double, 3, 9> H = Eigen::Matrix<double, 3, 9>::Zero();
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

        Eigen::Vector3d y = z_pos - H * x_;
        Eigen::Matrix3d S = H * P_ * H.transpose() + R;

        // Check S for positive definiteness and invertibility
        Eigen::Matrix3d S_inv;
        if (!safe_inverse(S, S_inv)) {
            std::cerr << "KF Update Position: Failed to invert innovation covariance!" << std::endl;
            return false;
        }

        Eigen::Matrix<double, 9, 3> K = P_ * H.transpose() * S_inv;

        // Check Kalman gain for NaN
        if (has_nan_matrix(K)) {
            std::cerr << "KF Update Position: NaN in Kalman gain!" << std::endl;
            return false;
        }

        // State update
        x_ = x_ + K * y;

        // Joseph form covariance update for numerical stability
        Eigen::Matrix<double, 9, 9> I_KH = Eigen::Matrix<double, 9, 9>::Identity() - K * H;
        P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();

        // Ensure symmetry
        P_ = 0.5 * (P_ + P_.transpose());

        // Check for NaN
        if (has_nan(x_) || has_nan_matrix(P_)) {
            std::cerr << "KF Update Position: NaN detected after update!" << std::endl;
            return false;
        }

        return true;
    }

    bool update_velocity(const Eigen::Vector3d& z_vel, const Eigen::Matrix3d& R) {
        // Check inputs
        if (has_nan(z_vel) || has_nan_matrix(R)) {
            std::cerr << "KF Update Velocity: NaN in measurement or covariance!" << std::endl;
            return false;
        }

        Eigen::Matrix<double, 3, 9> H = Eigen::Matrix<double, 3, 9>::Zero();
        H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

        Eigen::Vector3d y = z_vel - H * x_;
        Eigen::Matrix3d S = H * P_ * H.transpose() + R;

        // Check S for positive definiteness and invertibility
        Eigen::Matrix3d S_inv;
        if (!safe_inverse(S, S_inv)) {
            std::cerr << "KF Update Velocity: Failed to invert innovation covariance!" << std::endl;
            return false;
        }

        Eigen::Matrix<double, 9, 3> K = P_ * H.transpose() * S_inv;

        // Check Kalman gain for NaN
        if (has_nan_matrix(K)) {
            std::cerr << "KF Update Velocity: NaN in Kalman gain!" << std::endl;
            return false;
        }

        // State update
        x_ = x_ + K * y;

        // Joseph form covariance update
        Eigen::Matrix<double, 9, 9> I_KH = Eigen::Matrix<double, 9, 9>::Identity() - K * H;
        P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();

        // Ensure symmetry
        P_ = 0.5 * (P_ + P_.transpose());

        // Check for NaN
        if (has_nan(x_) || has_nan_matrix(P_)) {
            std::cerr << "KF Update Velocity: NaN detected after update!" << std::endl;
            return false;
        }

        return true;
    }

    Eigen::Vector3d get_position() const { return x_.segment<3>(0); }
    Eigen::Vector3d get_velocity() const { return x_.segment<3>(3); }
    Eigen::Vector3d get_accel_bias() const { return x_.segment<3>(6); }
    Eigen::Matrix<double, 9, 9> get_covariance() const { return P_; }

private:
    KFKConfig config_;
    Eigen::Matrix<double, 9, 1> x_;
    Eigen::Matrix<double, 9, 9> P_;
    Eigen::Matrix<double, 9, 9> Q_;

    /**
     * @brief Check if a vector contains NaN
     */
    bool has_nan(const Eigen::Vector3d& v) const {
        return std::isnan(v(0)) || std::isnan(v(1)) || std::isnan(v(2));
    }

    /**
     * @brief Check if a vector contains NaN (9D version)
     */
    bool has_nan(const Eigen::Matrix<double, 9, 1>& v) const {
        for (int i = 0; i < 9; ++i) {
            if (std::isnan(v(i))) return true;
        }
        return false;
    }

    /**
     * @brief Check if a matrix contains NaN or Inf
     */
    template<typename Derived>
    bool has_nan_matrix(const Eigen::MatrixBase<Derived>& m) const {
        for (int i = 0; i < m.rows(); ++i) {
            for (int j = 0; j < m.cols(); ++j) {
                if (std::isnan(m(i,j)) || std::isinf(m(i,j))) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * @brief Safe matrix inversion with condition number check
     * @return true if successful, false if matrix is singular or poorly conditioned
     */
    bool safe_inverse(const Eigen::Matrix3d& M, Eigen::Matrix3d& M_inv) const {
        // Check for NaN/Inf
        if (has_nan_matrix(M)) {
            return false;
        }

        // Check determinant
        double det = M.determinant();
        if (std::abs(det) < 1e-10) {
            std::cerr << "Matrix near-singular (det=" << det << "), using pseudo-inverse" << std::endl;
            
            // Use pseudo-inverse as fallback
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Vector3d sing_vals = svd.singularValues();
            
            // Regularize very small singular values
            Eigen::Vector3d sing_vals_inv;
            for (int i = 0; i < 3; ++i) {
                if (sing_vals(i) > 1e-6) {
                    sing_vals_inv(i) = 1.0 / sing_vals(i);
                } else {
                    sing_vals_inv(i) = 0.0;
                }
            }
            
            M_inv = svd.matrixV() * sing_vals_inv.asDiagonal() * svd.matrixU().transpose();
        } else {
            // Standard inverse
            M_inv = M.inverse();
        }

        // Verify result
        if (has_nan_matrix(M_inv)) {
            std::cerr << "Matrix inversion produced NaN!" << std::endl;
            return false;
        }

        return true;
    }
};

} // namespace ekf2_ros

#endif // EKF2_ROS__EKF_CORE_HPP_
#pragma once

#include "state/State.h"
#include <nav_msgs/msg/odometry.hpp>
#include <memory>
#include <Eigen/Eigen>

namespace ov_msckf {

class UpdaterWheelOdom {
public:
    /**
     * @brief Construct a new Updater Wheel Odom object
     * @param R_IB Rotation from base_link to IMU
     * @param t_IB Translation from base_link to IMU (lever arm)
     * @param noise_vx Noise std dev for linear velocity x
     * @param noise_vy Noise std dev for linear velocity y
     * @param noise_vz Noise std dev for linear velocity z
     * @param noise_wx Noise std dev for angular velocity x
     * @param noise_wy Noise std dev for angular velocity y
     * @param noise_wz Noise std dev for angular velocity z
     */
    UpdaterWheelOdom(const Eigen::Matrix3d& R_IB, const Eigen::Vector3d& t_IB,
                     double noise_vx, double noise_vy, double noise_vz,
                     double noise_wx, double noise_wy, double noise_wz);

    /**
     * @brief Perform EKF update of the state based on a wheel odometry measurement.
     * @param state The state to update.
     * @param odom The wheel odometry measurement, containing twist information.
     */
    void update(std::shared_ptr<State> state, const nav_msgs::msg::Odometry& odom);

private:
    // Transformation parameters from base_link to IMU
    Eigen::Matrix3d _R_IB; // Rotation
    Eigen::Vector3d _t_IB; // Translation (lever arm)

    // Noise parameters for the wheel odometry measurement
    double _noise_vx;
    double _noise_vy;
    double _noise_vz;
    double _noise_wx;
    double _noise_wy;
    double _noise_wz;
};

} // namespace ov_msckf

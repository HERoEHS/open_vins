#include "update/UpdaterWheelOdom.h"
#include "state/StateHelper.h"
#include "utils/quat_ops.h"
#include <cmath>
#include <algorithm> // For std::all_of

namespace ov_msckf {

UpdaterWheelOdom::UpdaterWheelOdom(const Eigen::Matrix3d& R_IB, const Eigen::Vector3d& t_IB,
                                   double noise_vx, double noise_vy, double noise_vz,
                                   double noise_wx, double noise_wy, double noise_wz)
    : _R_IB(R_IB), _t_IB(t_IB),
      _noise_vx(noise_vx), _noise_vy(noise_vy), _noise_vz(noise_vz),
      _noise_wx(noise_wx), _noise_wy(noise_wy), _noise_wz(noise_wz) {}

void UpdaterWheelOdom::update(std::shared_ptr<State> state, const nav_msgs::msg::Odometry& odom) {

    // 0. Transform measurement from base_link to IMU frame
    Eigen::Vector3d lin_vel_meas_I = _R_IB * Eigen::Vector3d(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z);
    Eigen::Vector3d ang_vel_meas_I = _R_IB * Eigen::Vector3d(odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z);

    // 1. Define which state variables we are updating
    std::vector<std::shared_ptr<ov_type::Type>> Hx_order;
    Hx_order.push_back(state->_imu->v());
    Hx_order.push_back(state->_imu->q());
    Hx_order.push_back(state->_imu->bg());

    // 2. Define our measurement residual (z-h(x))
    Eigen::Matrix<double, 6, 1> res;

    // Predicted linear velocity
    const Eigen::Vector3d& v_IinG = state->_imu->vel();
    ov_type::JPLQuat q_GtoI;
    q_GtoI.set_value(state->_imu->quat());
    Eigen::Matrix3d R_GI = q_GtoI.Rot();
    Eigen::Vector3d v_I_pred = R_GI * v_IinG;

    // Predicted angular velocity (w_imu - bias_g)
    const Eigen::Vector3d w_I_pred = state->_imu->bias_g();

    // Add lever arm correction to predicted linear velocity
    v_I_pred += w_I_pred.cross(_t_IB);

    // Calculate residuals
    res.segment<3>(0) = lin_vel_meas_I - v_I_pred;
    res.segment<3>(3) = ang_vel_meas_I - w_I_pred;

    // 3. Define our measurement Jacobian H
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, state->max_covariance_size());

    int vel_idx = state->_imu->v()->id();
    int ori_idx = state->_imu->q()->id();
    int bg_idx = state->_imu->bg()->id();

    // Jacobian of linear velocity residual
    H.block<3, 3>(0, vel_idx) = -R_GI; // wrt v_G
    H.block<3, 3>(0, ori_idx) = R_GI * ov_core::skew_x(v_IinG); // wrt delta_theta
    H.block<3, 3>(0, bg_idx) = ov_core::skew_x(_t_IB); // wrt bias_g (from lever arm)

    // Jacobian of angular velocity residual
    H.block<3, 3>(3, bg_idx) = -Eigen::Matrix3d::Identity(); // wrt bias_g

    // 4. Define measurement noise covariance R
    Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Zero();
    bool covariance_is_valid = !std::all_of(std::begin(odom.twist.covariance), std::end(odom.twist.covariance), [](double v){ return v == 0.0; });
    if (covariance_is_valid) {
        for (int i = 0; i < 6; i++) {
            R(i, i) = std::max(1e-9, odom.twist.covariance[6 * i + i]);
        }
    } else {
        R(0, 0) = std::pow(_noise_vx, 2); R(1, 1) = std::pow(_noise_vy, 2); R(2, 2) = std::pow(_noise_vz, 2);
        R(3, 3) = std::pow(_noise_wx, 2); R(4, 4) = std::pow(_noise_wy, 2); R(5, 5) = std::pow(_noise_wz, 2);
    }

    // 5. Perform EKF Update
    StateHelper::EKFUpdate(state, Hx_order, H, res, R);
}

} // namespace ov_msckf

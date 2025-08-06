#include "update/UpdaterCorrectedPose.h"
#include "state/StateHelper.h"
#include "utils/quat_ops.h"
#include <cmath>

namespace ov_msckf {

UpdaterCorrectedPose::UpdaterCorrectedPose() {
    // These noise values can be loaded from a config file if needed
    _noise_x = 0.1;   // meters
    _noise_y = 0.1;   // meters
    _noise_yaw = 0.05; // radians
}

void UpdaterCorrectedPose::update(std::shared_ptr<State> state, const geometry_msgs::msg::Pose2D& pose) {

    // Before this second update, we should reset the FEJ point
    // This is because the state was just modified by the ZUPT update
    state->_imu->p()->set_fej(state->_imu->pos());
    state->_imu->q()->set_fej(state->_imu->quat());

    // 1. Define which state variables we are updating (position and orientation)
    std::vector<std::shared_ptr<ov_type::Type>> Hx_order;
    Hx_order.push_back(state->_imu->p());
    Hx_order.push_back(state->_imu->q());

    // 2. Define our measurement residual (z-h(x))
    Eigen::Vector3d res;

    // Position residual (x, y)
    const Eigen::Vector3d& p_IinG = state->_imu->pos_fej();
    res(0) = pose.x - p_IinG.x();
    res(1) = pose.y - p_IinG.y();

    // Orientation residual (yaw)
    ov_type::JPLQuat q_GtoI_fej;
    q_GtoI_fej.set_value(state->_imu->quat_fej());

    // ZYX Euler angles convention returns a vector (yaw, pitch, roll)
    Eigen::Vector3d rpy = q_GtoI_fej.Rot().eulerAngles(2, 1, 0);
    double yaw_error = pose.theta - rpy(0); // Yaw is at index 0 for ZYX convention
    // Handle angle wrapping
    yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));
    res(2) = yaw_error;

    // 3. Define our measurement Jacobian H
    // This is the derivative of the measurement function w.r.t the error state.
    // Error state is [dp_x, dp_y, dp_z, dalpha_x, dalpha_y, dalpha_z]
    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    H(0, 0) = 1.0; // d(res_x) / d(dp_x)
    H(1, 1) = 1.0; // d(res_y) / d(dp_y)
    
    // Derivative of yaw w.r.t orientation error state (dalpha)
    // d(res_yaw)/d(dalpha) = -d(yaw)/d(dalpha)
    double pitch = rpy(1); // Pitch is at index 1
    double roll = rpy(2); // Roll is at index 2

    // Avoid singularity at pitch = +/- 90 degrees
    if (std::abs(std::cos(pitch)) > 1e-4) {
        H(2, 3) = 0.0;
        H(2, 4) = -std::sin(roll) / std::cos(pitch);
        H(2, 5) = -std::cos(roll) / std::cos(pitch);
    }

    // 4. Define measurement noise covariance R
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    R(0, 0) = std::pow(_noise_x, 2);
    R(1, 1) = std::pow(_noise_y, 2);
    R(2, 2) = std::pow(_noise_yaw, 2);

    // 5. Perform EKF Update
    StateHelper::EKFUpdate(state, Hx_order, H, res, R);
}

}
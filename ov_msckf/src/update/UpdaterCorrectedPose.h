#pragma once

#include "state/State.h"
#include <geometry_msgs/msg/pose2_d.hpp>
#include <memory>

namespace ov_msckf {

class UpdaterCorrectedPose {
public:
  UpdaterCorrectedPose();

  /**
   * @brief Perform EKF update of the state based on an external pose measurement.
   * @param state The state to update.
   * @param pose The external pose measurement (x, y, yaw).
   */
  void update(std::shared_ptr<State> state, const geometry_msgs::msg::Pose2D& pose);

public:
    double _noise_x;
    double _noise_y;
    double _noise_yaw;
};

} // namespace ov_msckf
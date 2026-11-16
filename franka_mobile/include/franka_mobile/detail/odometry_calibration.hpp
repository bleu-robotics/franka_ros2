// Copyright (c) 2026 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cmath>

#include <Eigen/Dense>
#include <franka_mobile/detail/body_twist.hpp>

namespace franka_mobile {
namespace detail {

/**
 * @brief Advances the calibrated pose (p, q) by one measured step, its translation and turn scaled.
 *
 * The measured step from pose 0 to pose 1 is taken into the base frame at its mid-heading, its
 * translation scaled by linear_scale and its turn by angular_scale, then added at the calibrated
 * pose's own mid-heading:
 * \f[
 *   \Delta\psi = \psi_1 - \psi_0, \quad
 *   \Delta p_b = R(\psi_0 + \Delta\psi/2)^\top (p_1 - p_0)
 * \f]
 * \f[
 *   p \mathrel{+}= R(\psi + s_\omega \Delta\psi/2)\, s_d\, \Delta p_b, \quad
 *   \psi \mathrel{+}= s_\omega \Delta\psi
 * \f]
 * With both scales 1 and (p, q) seeded at pose 0, the calibrated pose follows the measured one
 * exactly. z is left as seeded.
 *
 * @param p0, q0         Measured pose at the start, in the odometry frame.
 * @param p1, q1         Measured pose at the end, in the odometry frame.
 * @param linear_scale   s_d: true distance over measured distance.
 * @param angular_scale  s_omega: true turn over measured turn.
 * @param p, q           Calibrated pose, advanced in place.
 */
inline void calibratedStep(const Eigen::Vector3d& p0,
                           const Eigen::Quaterniond& q0,
                           const Eigen::Vector3d& p1,
                           const Eigen::Quaterniond& q1,
                           double linear_scale,
                           double angular_scale,
                           Eigen::Vector3d& p,
                           Eigen::Quaterniond& q) {
  const double yaw0 = yawOf(q0);
  const double dyaw = std::remainder(yawOf(q1) - yaw0, 2.0 * M_PI);
  const Eigen::Vector2d body_step = Eigen::Rotation2Dd(-(yaw0 + 0.5 * dyaw)) * (p1 - p0).head<2>();
  const double yaw = yawOf(q);
  const double turn = angular_scale * dyaw;
  p.head<2>() += Eigen::Rotation2Dd(yaw + 0.5 * turn) * (linear_scale * body_step);
  q = Eigen::Quaterniond(Eigen::AngleAxisd(yaw + turn, Eigen::Vector3d::UnitZ()));
}

}  // namespace detail
}  // namespace franka_mobile

/**
 * inverse_kinematics.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: June 2, 2019
 * Authors: Toki Migimatsu
 */

#include "spatial_dyn/algorithms/inverse_kinematics.h"

#include <ctrl_utils/control.h>
#include <ctrl_utils/euclidian.h>

#include "spatial_dyn/algorithms/forward_kinematics.h"
#include "spatial_dyn/algorithms/opspace_dynamics.h"
#include "spatial_dyn/algorithms/simulation.h"

namespace {

// Controller parameters
const Eigen::Vector2d kKpKvPos = Eigen::Vector2d(100., 20.);
const Eigen::Vector2d kKpKvOri = Eigen::Vector2d(100., 20.);
const Eigen::Vector2d kKpKvJoint = Eigen::Vector2d(100., 20.);
const double kTimerFreq = 1000.;  // Max ~2200

const size_t kMaxIterations = 10000;
const double kTolerancePos = 1e-3;
const double kToleranceOri = 1e-3;

}  // namespace

namespace spatial_dyn {

Eigen::VectorXd InverseKinematics(const ArticulatedBody& ab_in,
                                  Eigen::Ref<const Eigen::Vector3d> x_des,
                                  const Eigen::Quaterniond& quat_des, int link,
                                  const Eigen::Vector3d& offset) {
  // Initialize robot
  spatial_dyn::ArticulatedBody ab = ab_in;
  const Eigen::VectorXd q_des = ab.q();

  for (size_t i = 0; i < kMaxIterations; i++) {
    // Compute Jacobian
    const Eigen::Matrix6Xd& J = spatial_dyn::Jacobian(ab, link, offset);

    // Compute position PD control
    const Eigen::Vector3d x = spatial_dyn::Position(ab, link, offset);
    const Eigen::Vector3d dx = J.topRows<3>() * ab.dq();
    Eigen::Vector3d x_err;
    const Eigen::Vector3d ddx =
        ctrl_utils::PdControl(x, x_des, dx, kKpKvPos, 0., &x_err);

    // Compute orientation PD control
    const Eigen::Quaterniond quat =
        ctrl_utils::NearQuaternion(spatial_dyn::Orientation(ab), quat_des);
    const Eigen::Vector3d w = J.bottomRows<3>() * ab.dq();
    Eigen::Vector3d ori_err;
    const Eigen::Vector3d dw =
        ctrl_utils::PdControl(quat, quat_des, w, kKpKvOri, 0., &ori_err);

    if (x_err.squaredNorm() < kTolerancePos * kTolerancePos &&
        ori_err.squaredNorm() < kToleranceOri * kToleranceOri) {
      break;
    }

    // Compute opspace torques
    Eigen::MatrixXd N;
    Eigen::VectorXd tau_cmd;
    if (spatial_dyn::opspace::IsSingular(ab, J)) {
      // If robot is at a singularity, control position only
      tau_cmd =
          spatial_dyn::opspace::InverseDynamics(ab, J.topRows<3>(), ddx, &N);
    } else {
      // Control position and orientation
      Eigen::Vector6d ddx_dw;
      ddx_dw << ddx, dw;
      tau_cmd = spatial_dyn::opspace::InverseDynamics(ab, J, ddx_dw, &N);
    }

    // Add joint task in nullspace
    static const Eigen::MatrixXd I =
        Eigen::MatrixXd::Identity(ab.dof(), ab.dof());
    const Eigen::VectorXd ddq =
        ctrl_utils::PdControl(ab.q(), q_des, ab.dq(), kKpKvJoint);
    tau_cmd += spatial_dyn::opspace::InverseDynamics(ab, I, ddq, &N);

    // Integrate
    spatial_dyn::Integrate(ab, tau_cmd, 1. / kTimerFreq, {},
                           IntegrationOptions(false));
    // spatial_dyn::discrete::Integrate(ab, tau_cmd, timer.dt(), f_ext);
  }

  return ab.q();
}

}  // namespace spatial_dyn

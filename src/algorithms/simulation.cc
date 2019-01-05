/**
 * simulation.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 4, 2018
 * Authors: Toki Migimatsu
 */

#include "algorithms/simulation.h"

#include <qpOASES.hpp>

#include "algorithms/forward_dynamics.h"

namespace SpatialDyn {

static qpOASES::Options QpOptions() {
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_NONE;
  return options;
}
static const qpOASES::Options kQpOptions = QpOptions();

static Eigen::VectorXd JointLimitImpulse(const ArticulatedBody& ab,
                                         Eigen::Ref<const Eigen::VectorXd> ddq) {
  std::vector<size_t> idx_limits;  // Joint indices exceeding limits
  std::vector<bool> is_max;        // Whether joint exceeds max
  for (size_t i = 0; i < ab.dof(); i++) {
    bool exceeds_max = ab.q(i) >= ab.rigid_bodies(i).joint().q_max();
    if (exceeds_max || ab.q(i) <= ab.rigid_bodies(i).joint().q_min()) {
      idx_limits.push_back(i);
      is_max.push_back(exceeds_max);
    }
  }
  const size_t m = idx_limits.size();

  if (m == 0) return Eigen::VectorXd::Zero(ab.dof());

  // Unit torque impulses applied at joint limits
  Eigen::MatrixXd tau_impulse_hat(ab.dof(), m);
  for (size_t i = 0; i < m; i++) {
    tau_impulse_hat.col(i) = Eigen::VectorXd::Unit(ab.dof(), idx_limits[i]);
  }

  // Solve impulse accelerations
  Eigen::MatrixXd ddq_impulse_hat = InertiaInverse(ab).solve(tau_impulse_hat);

  // Find linear combination of ddq_impulse_hat columns that produces valid
  // joint accelerations at the limit joints.
  // for i in idx_limits:
  //     if max: ddq_impulse_hat.row(i) * x + ddq(i) < 0
  //     if min: ddq_impulse_hat.row(i) * x + ddq(i) > 0
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(m, m);
  Eigen::VectorXd b(m);
  for (size_t i = 0; i < m; i++) {
    double coeff = is_max[i] ? 1. : -1.;
    A.row(i) = coeff * ddq_impulse_hat.row(idx_limits[i]);
    b(i) = -coeff * ddq(idx_limits[i]);
  }
  Eigen::VectorXd c = Eigen::VectorXd::Zero(m);
  Eigen::VectorXd x(m);

  // Find min norm x that satisfies above constraints.
  int nwsr = 10;
  qpOASES::QProblem qp(m, m, qpOASES::HST_IDENTITY);
  qp.setOptions(kQpOptions);
  qp.init(nullptr, c.data(), A.data(), nullptr, nullptr, nullptr, b.data(), nwsr);
  qp.getPrimalSolution(x.data());

  return ddq_impulse_hat * x;
}

static Eigen::VectorXd ClipQ(const ArticulatedBody& ab, const Eigen::VectorXd& q,
                             Eigen::VectorXd* q_err = nullptr) {
  Eigen::VectorXd q_clip = q;
  for (size_t i = 0; i < ab.dof(); i++) {
    const Joint& joint = ab.rigid_bodies(i).joint();
    if (q(i) > joint.q_max()) {
      q_clip(i) = joint.q_max();
    } else if (q(i) < joint.q_min()) {
      q_clip(i) = joint.q_min();
    }
    if (q_err != nullptr) {
      q_err->coeffRef(i) = q(i) - q_clip(i);
    }
  }
  return q_clip;
}

static Eigen::VectorXd ClipDq(const ArticulatedBody& ab, const Eigen::VectorXd& dq,
                              Eigen::VectorXd* dq_err = nullptr) {
  Eigen::VectorXd dq_clip = dq;
  for (size_t i = 0; i < ab.dof(); i++) {
    const Joint& joint = ab.rigid_bodies(i).joint();
    if ((ab.q(i) >= joint.q_max() && dq(i) > 0.) || (ab.q(i) <= joint.q_min() && dq(i) < 0.)) {
      dq_clip(i) = 0.;
    }
    if (dq_err != nullptr) {
      dq_err->coeffRef(i) = dq(i) - dq_clip(i);
    }
  }
  return dq_err != nullptr ? dq : dq_clip;
}

static Eigen::VectorXd Identity(const ArticulatedBody& ab, const Eigen::VectorXd& q,
                                Eigen::VectorXd* q_err = nullptr) {
  return q;
}

void Integrate(ArticulatedBody &ab, const Eigen::VectorXd& tau, double dt,
               const std::map<int, SpatialForced>& f_external,
               bool gravity, bool centrifugal_coriolis, bool friction, bool joint_limits,
               double stiction_epsilon, bool aba, IntegrationMethod method) {

  const auto& f = aba ? ForwardDynamicsAba : ForwardDynamics;
  const auto& FilterQ = joint_limits ? ClipQ : Identity;
  const auto& FilterDq = joint_limits ? ClipDq : Identity;
  // Eigen::VectorXd q_err(ab.dof());
  // Eigen::VectorXd dq_err(ab.dof());
  Eigen::VectorXd* p_q_err = nullptr;//joint_limits ? &q_err : nullptr;
  Eigen::VectorXd* p_dq_err = nullptr;//joint_limits ? &dq_err : nullptr;

  std::array<Eigen::VectorXd, 4> dq, ddq;
  Eigen::VectorXd q_0 = ab.q();
   dq[0] = ab.dq();
  ddq[0] = f(ab, tau, f_external, gravity, centrifugal_coriolis, friction, stiction_epsilon);

  switch (method) {
    case IntegrationMethod::EULER:
      if (joint_limits) ddq[0] += JointLimitImpulse(ab, ddq[0] + (*p_dq_err) / dt);
      ab.set_q(FilterQ(ab, q_0 + dt * dq[0] + 0.5 * dt * dt * ddq[0], p_q_err));
      ab.set_dq(FilterDq(ab, dq[0] + dt * ddq[0], nullptr));
      break;
    case IntegrationMethod::HEUNS:
      if (joint_limits) ddq[0] += JointLimitImpulse(ab, ddq[0]);
      // if (joint_limits) ddq[0] += JointLimitImpulse(ab, ddq[0] + (*p_dq_err) / dt);
      ab.set_q(FilterQ(ab, q_0 + dt * dq[0], p_q_err));
      ab.set_dq(FilterDq(ab, dq[0] + dt * ddq[0], p_dq_err));
       dq[1] = ab.dq();
      ddq[1] = f(ab, tau, f_external, gravity, centrifugal_coriolis, friction, stiction_epsilon);

      if (joint_limits) ddq[1] += JointLimitImpulse(ab, ddq[1]);
      // if (joint_limits) ddq[1] += JointLimitImpulse(ab, ddq[1] + (*p_dq_err) / dt);
      ab.set_q(FilterQ(ab, q_0 + 0.5 * dt * (dq[0] + dq[1]), p_q_err));
      ab.set_dq(FilterDq(ab, dq[0] + 0.5 * dt * (ddq[0] + ddq[1]), nullptr));
      break;
    case IntegrationMethod::RK4:
      if (joint_limits) ddq[0] += JointLimitImpulse(ab, ddq[0]);
      // if (joint_limits) ddq[0] += JointLimitImpulse(ab, ddq[0] + (*p_dq_err) / dt);
      ab.set_q(FilterQ(ab, q_0 + 0.5 * dt * dq[0], p_q_err));
      ab.set_dq(FilterDq(ab, dq[0] + 0.5 * dt * ddq[0], p_dq_err));
       dq[1] = ab.dq();
      ddq[1] = f(ab, tau, f_external, gravity, centrifugal_coriolis, friction, stiction_epsilon);

      if (joint_limits) ddq[1] += JointLimitImpulse(ab, ddq[1]);
      // if (joint_limits) ddq[1] += JointLimitImpulse(ab, ddq[1] + (*p_dq_err) / dt);
      ab.set_q(FilterQ(ab, q_0 + 0.5 * dt * dq[1], p_q_err));
      ab.set_dq(FilterDq(ab, dq[0] + 0.5 * dt * ddq[1], p_dq_err));
       dq[2] = ab.dq();
      ddq[2] = f(ab, tau, f_external, gravity, centrifugal_coriolis, friction, stiction_epsilon);

      if (joint_limits) ddq[2] += JointLimitImpulse(ab, ddq[2]);
      // if (joint_limits) ddq[2] += JointLimitImpulse(ab, ddq[2] + (*p_dq_err) / dt);
      ab.set_q(FilterQ(ab, q_0 + dt * dq[2], p_q_err));
      ab.set_dq(FilterDq(ab, dq[0] + dt * ddq[2], p_dq_err));
       dq[3] = ab.dq();
      ddq[3] = f(ab, tau, f_external, gravity, centrifugal_coriolis, friction, stiction_epsilon);

      if (joint_limits) ddq[3] += JointLimitImpulse(ab, ddq[3]);
      // if (joint_limits) ddq[3] += JointLimitImpulse(ab, ddq[3] + (*p_dq_err) / dt);
      ab.set_q(FilterQ(ab, q_0 + dt / 6. * (dq[0] + 2. * dq[1] + 2. * dq[2] + dq[3]), p_q_err));
      ab.set_dq(FilterDq(ab, dq[0] + dt / 6. * (ddq[0] + 2. * ddq[1] + 2. * ddq[2] + ddq[3]), nullptr));
      break;
  }
}

}  // namespace SpatialDyn

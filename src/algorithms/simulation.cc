/**
 * simulation.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 4, 2018
 * Authors: Toki Migimatsu
 */

#include "algorithms/simulation.h"

#include "algorithms/forward_dynamics.h"

namespace SpatialDyn {

void Integrate(ArticulatedBody &ab, const Eigen::VectorXd& tau, double dt,
               const std::map<int, SpatialForced>& f_external,
               bool gravity, bool centrifugal_coriolis, bool friction,
               double stiction_epsilon, bool aba, IntegrationMethod method) {

  const auto& f = aba ? ForwardDynamicsAba : ForwardDynamics;

  std::array<Eigen::VectorXd, 4> dq, ddq;
  Eigen::VectorXd q_0 = ab.q();
   dq[0] = ab.dq();
  ddq[0] = f(ab, tau, f_external, gravity, centrifugal_coriolis, friction, stiction_epsilon);

  switch (method) {
    case IntegrationMethod::EULER:
      ab.set_q(  q_0  + dt *  dq[0] + 0.5 * dt * dt * ddq[0]);
      ab.set_dq(dq[0] + dt * ddq[0]);
      break;
    case IntegrationMethod::HEUNS:
      ab.set_q(  q_0  + dt *  dq[0]);
      ab.set_dq(dq[0] + dt * ddq[0]);
       dq[1] = ab.dq();
      ddq[1] = f(ab, tau, f_external, gravity, centrifugal_coriolis, friction, stiction_epsilon);

      ab.set_q(  q_0  + 0.5 * dt * ( dq[0] +  dq[1]));
      ab.set_dq(dq[0] + 0.5 * dt * (ddq[0] + ddq[1]));
      break;
    case IntegrationMethod::RK4:
      ab.set_q(  q_0  + 0.5 * dt *  dq[0]);
      ab.set_dq(dq[0] + 0.5 * dt * ddq[0]);
       dq[1] = ab.dq();
      ddq[1] = f(ab, tau, f_external, gravity, centrifugal_coriolis, friction, stiction_epsilon);

      ab.set_q(  q_0  + 0.5 * dt *  dq[1]);
      ab.set_dq(dq[0] + 0.5 * dt * ddq[1]);
       dq[2] = ab.dq();
      ddq[2] = f(ab, tau, f_external, gravity, centrifugal_coriolis, friction, stiction_epsilon);

      ab.set_q(  q_0  + dt *  dq[2]);
      ab.set_dq(dq[0] + dt * ddq[2]);
       dq[3] = ab.dq();
      ddq[3] = f(ab, tau, f_external, gravity, centrifugal_coriolis, friction, stiction_epsilon);

      ab.set_q(  q_0  + dt / 6. * ( dq[0] + 2. *  dq[1] + 2. *  dq[2] +  dq[3]));
      ab.set_dq(dq[0] + dt / 6. * (ddq[0] + 2. * ddq[1] + 2. * ddq[2] + ddq[3]));
      break;
  }
}

}  // namespace SpatialDyn

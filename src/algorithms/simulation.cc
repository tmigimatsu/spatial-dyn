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

void Integrate(ArticulatedBody &ab, const Eigen::VectorXd& tau, double dt) {
  Eigen::VectorXd q  = ab.q();
  Eigen::VectorXd dq = ab.dq();
  Eigen::VectorXd ddq = ab.ddq();

  // Forward Euler
  ab.set_q(q + dq * dt);
  ab.set_dq(dq + ddq * dt);
  ddq = ForwardDynamics(ab, tau);

  q  += 0.5 * (ab.dq()  + dq)  * dt;
  dq += 0.5 * (ab.ddq() + ddq) * dt;

  ab.set_q(q + dq * dt);
  ab.set_dq(dq + ddq * dt);
  ddq = ForwardDynamics(ab, tau);
}

}  // namespace SpatialDyn
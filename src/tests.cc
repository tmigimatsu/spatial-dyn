/**
 * tests.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 21, 2018
 * Authors: Toki Migimatsu
 */

#include "spatial_math.h"
#include <rbdl/rbdl.h>

#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include <iostream>

TEST_CASE("spatial vector transforms", "[SpatialVector]") {
  Eigen::Quaterniond quat = Eigen::AngleAxisd(M_PI/3, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(M_PI/3, Eigen::Vector3d::UnitY());
  Eigen::Vector3d trans(4,5,6);

  Eigen::Affine3d T = Eigen::Translation3d(trans) * quat;
  SpatialDyn::SpatialMotiond m(1,3,5,2,4,6);
  SpatialDyn::SpatialMotiond n = T * m;
  SpatialDyn::SpatialForced f(1,3,5,2,4,6);
  SpatialDyn::SpatialForced g = T * f;

  RigidBodyDynamics::Math::SpatialTransform T_rbdl(quat.toRotationMatrix(), trans);
  RigidBodyDynamics::Math::SpatialVector m_rbdl(2,4,6,1,3,5);
  RigidBodyDynamics::Math::SpatialVector n_rbdl = T_rbdl.apply(m_rbdl);
  RigidBodyDynamics::Math::SpatialVector f_rbdl(2,4,6,1,3,5);
  RigidBodyDynamics::Math::SpatialVector g_rbdl = T_rbdl.applyAdjoint(f_rbdl);

  REQUIRE((n.matrix().head<3>() - n_rbdl.tail<3>()).norm() < 1e-10);
  REQUIRE((n.matrix().tail<3>() - n_rbdl.head<3>()).norm() < 1e-10);
  REQUIRE((g.matrix().head<3>() - g_rbdl.tail<3>()).norm() < 1e-10);
  REQUIRE((g.matrix().tail<3>() - g_rbdl.head<3>()).norm() < 1e-10);
}

TEST_CASE("spatial inertia", "[SpatialInertia]") {
  Eigen::Quaterniond quat = Eigen::AngleAxisd(M_PI/3, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(M_PI/3, Eigen::Vector3d::UnitY());
  Eigen::Vector3d trans(4,5,6);
  double mass = 2;
  Eigen::Vector3d com(1,1,1);
  Eigen::Vector6d I_com; I_com << 1,3,5,2,4,6;
  Eigen::Matrix3d I_com_3d;
  I_com_3d << I_com(0), I_com(3), I_com(4),
              I_com(3), I_com(1), I_com(5),
              I_com(4), I_com(5), I_com(2);

  Eigen::Affine3d T = Eigen::Translation3d(trans) * quat;
  SpatialDyn::SpatialInertiad I(mass, com, I_com);
  SpatialDyn::SpatialInertiad J = T * I;
  SpatialDyn::SpatialMotiond v(1,2,3,4,5,6);

  RigidBodyDynamics::Math::SpatialTransform T_rbdl(quat.toRotationMatrix(), trans);
  auto I_rbdl = RigidBodyDynamics::Math::SpatialRigidBodyInertia::createFromMassComInertiaC(mass, com, I_com_3d);
  RigidBodyDynamics::Math::SpatialRigidBodyInertia J_rbdl = T_rbdl.apply(I_rbdl);
  RigidBodyDynamics::Math::SpatialVector v_rbdl(4,5,6,1,2,3);

  Eigen::Matrix6d I_mat = I.matrix();
  const auto I_A = I_mat.topLeftCorner<3,3>();
  const auto I_B = I_mat.topRightCorner<3,3>();
  const auto I_C = I_mat.bottomLeftCorner<3,3>();
  const auto I_D = I_mat.bottomRightCorner<3,3>();
  Eigen::Matrix6d I_rbdl_mat = I_rbdl.toMatrix();
  const auto I_rbdl_A = I_rbdl_mat.topLeftCorner<3,3>();
  const auto I_rbdl_B = I_rbdl_mat.topRightCorner<3,3>();
  const auto I_rbdl_C = I_rbdl_mat.bottomLeftCorner<3,3>();
  const auto I_rbdl_D = I_rbdl_mat.bottomRightCorner<3,3>();
  REQUIRE((I_A - I_rbdl_D).norm() < 1e-10);
  REQUIRE((I_B - I_rbdl_C).norm() < 1e-10);
  REQUIRE((I_C - I_rbdl_B).norm() < 1e-10);
  REQUIRE((I_D - I_rbdl_A).norm() < 1e-10);

  Eigen::Matrix6d J_mat = J.matrix();
  const auto J_A = J_mat.topLeftCorner<3,3>();
  const auto J_B = J_mat.topRightCorner<3,3>();
  const auto J_C = J_mat.bottomLeftCorner<3,3>();
  const auto J_D = J_mat.bottomRightCorner<3,3>();
  Eigen::Matrix6d J_rbdl_mat = J_rbdl.toMatrix();
  const auto J_rbdl_A = J_rbdl_mat.topLeftCorner<3,3>();
  const auto J_rbdl_B = J_rbdl_mat.topRightCorner<3,3>();
  const auto J_rbdl_C = J_rbdl_mat.bottomLeftCorner<3,3>();
  const auto J_rbdl_D = J_rbdl_mat.bottomRightCorner<3,3>();
  REQUIRE((J_A - J_rbdl_D).norm() < 1e-10);
  REQUIRE((J_B - J_rbdl_C).norm() < 1e-10);
  REQUIRE((J_C - J_rbdl_B).norm() < 1e-10);
  REQUIRE((J_D - J_rbdl_A).norm() < 1e-10);

  SpatialDyn::SpatialForced f = I * v;
  SpatialDyn::SpatialForced g = J * v;
  RigidBodyDynamics::Math::SpatialVector f_rbdl = I_rbdl * v_rbdl;
  RigidBodyDynamics::Math::SpatialVector g_rbdl = J_rbdl * v_rbdl;
  REQUIRE((f.matrix().head<3>() - f_rbdl.tail<3>()).norm() < 1e-10);
  REQUIRE((f.matrix().tail<3>() - f_rbdl.head<3>()).norm() < 1e-10);
  REQUIRE((g.matrix().head<3>() - g_rbdl.tail<3>()).norm() < 1e-10);
  REQUIRE((g.matrix().tail<3>() - g_rbdl.head<3>()).norm() < 1e-10);
}

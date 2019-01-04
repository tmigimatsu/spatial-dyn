/**
 * tests.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 21, 2018
 * Authors: Toki Migimatsu
 */

#include "SpatialDyn/algorithms/forward_kinematics.h"
#include "SpatialDyn/algorithms/inverse_dynamics.h"
#include "SpatialDyn/algorithms/forward_dynamics.h"
#include "SpatialDyn/algorithms/opspace_dynamics.h"

#include <rbdl/rbdl.h>

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include <iostream>

TEST_CASE("spatial vector transforms", "[SpatialVector]") {
  Eigen::Quaterniond quat = Eigen::AngleAxisd(M_PI/3, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(M_PI/3, Eigen::Vector3d::UnitY());
  Eigen::Vector3d trans(4,5,6);

  Eigen::Isometry3d T = Eigen::Translation3d(trans) * quat;
  SpatialDyn::SpatialMotiond m(1,3,5,2,4,6);
  SpatialDyn::SpatialMotiond n = T * m;
  SpatialDyn::SpatialForced f(1,3,5,2,4,6);
  SpatialDyn::SpatialForced g = T * f;

  RigidBodyDynamics::Math::SpatialTransform T_rbdl(quat.toRotationMatrix(), -quat.toRotationMatrix().transpose() * trans);
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
  Eigen::Quaterniond quat = Eigen::AngleAxisd(1, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(1, Eigen::Vector3d::UnitY());
  Eigen::Vector3d trans(4,5,6);
  double mass = 2;
  Eigen::Vector3d com(1,1,1);
  Eigen::Vector6d I_com; I_com << 1,3,5,2,4,6;
  Eigen::Matrix3d I_com_3d;
  I_com_3d << I_com(0), I_com(3), I_com(4),
              I_com(3), I_com(1), I_com(5),
              I_com(4), I_com(5), I_com(2);

  Eigen::Isometry3d T = Eigen::Translation3d(trans) * quat;
  SpatialDyn::SpatialInertiad I(mass, com, I_com);
  SpatialDyn::SpatialInertiad J = T * I;
  SpatialDyn::SpatialMotiond v(1,2,3,4,5,6);

  RigidBodyDynamics::Math::SpatialTransform T_rbdl(quat.toRotationMatrix(), -quat.toRotationMatrix().transpose() * trans);
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

void AddBody(SpatialDyn::ArticulatedBody *ab, RigidBodyDynamics::Model *ab_rbdl,
             int id_parent, const std::string& name,
             const Eigen::AngleAxisd& aa, const Eigen::Vector3d& trans,
             double mass, const Eigen::Vector3d& com, const Eigen::Vector6d& I_com,
             const SpatialDyn::Joint::Type& joint_type) {
    SpatialDyn::RigidBody rb(name);
    rb.set_T_to_parent(Eigen::Quaterniond(aa), trans);
    rb.set_inertia(mass, com, I_com);
    rb.set_joint(SpatialDyn::Joint(joint_type));
    ab->AddRigidBody(std::move(rb), id_parent);

    auto joint_frame = RigidBodyDynamics::Math::Xrot(aa.angle(), aa.axis()) * 
                       RigidBodyDynamics::Math::Xtrans(trans);
    RigidBodyDynamics::Body body(mass, com,
        RigidBodyDynamics::Math::Matrix3d(I_com(0), I_com(3), I_com(4),
                                          I_com(3), I_com(1), I_com(5),
                                          I_com(4), I_com(5), I_com(2)));
    RigidBodyDynamics::JointType joint_type_rbdl;
    RigidBodyDynamics::Math::Vector3d joint_axis_rbdl;
    switch (joint_type) {
      case SpatialDyn::Joint::Type::RX:
        joint_type_rbdl = RigidBodyDynamics::JointTypeRevolute;
        joint_axis_rbdl = Eigen::Vector3d::UnitX();
        break;
      case SpatialDyn::Joint::Type::RY:
        joint_type_rbdl = RigidBodyDynamics::JointTypeRevolute;
        joint_axis_rbdl = Eigen::Vector3d::UnitY();
        break;
      case SpatialDyn::Joint::Type::RZ:
        joint_type_rbdl = RigidBodyDynamics::JointTypeRevolute;
        joint_axis_rbdl = Eigen::Vector3d::UnitZ();
        break;
      case SpatialDyn::Joint::Type::PX:
        joint_type_rbdl = RigidBodyDynamics::JointTypePrismatic;
        joint_axis_rbdl = Eigen::Vector3d::UnitX();
        break;
      case SpatialDyn::Joint::Type::PY:
        joint_type_rbdl = RigidBodyDynamics::JointTypePrismatic;
        joint_axis_rbdl = Eigen::Vector3d::UnitY();
        break;
      case SpatialDyn::Joint::Type::PZ:
        joint_type_rbdl = RigidBodyDynamics::JointTypePrismatic;
        joint_axis_rbdl = Eigen::Vector3d::UnitZ();
        break;
      default:
        joint_type_rbdl = RigidBodyDynamics::JointTypeUndefined;
        break;
    }
    RigidBodyDynamics::Joint joint(joint_type_rbdl, joint_axis_rbdl);
    ab_rbdl->AddBody(id_parent + 1, joint_frame, joint, body, name);
}

TEST_CASE("articulated body", "[ArticulatedBody]") {
  SpatialDyn::ArticulatedBody ab("ur5");
  RigidBodyDynamics::Model ab_rbdl;
  AddBody(&ab, &ab_rbdl, -1, "shoulder_pan",
          Eigen::AngleAxisd::Identity(), Eigen::Vector3d(0, 0, 0.089159),
          3.7, Eigen::Vector3d(0, 0, 0.14), Eigen::Vector6d(0.010267495893, 0.010267495893, 0.00666, 0, 0, 0),
          SpatialDyn::Joint::Type::RZ);

  AddBody(&ab, &ab_rbdl, 0, "shoulder_lift",
          Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()), Eigen::Vector3d(0, 0.13585, 0),
          8.393, Eigen::Vector3d(0, 0, 0.28), Eigen::Vector6d(0.22689067591, 0.22689067591, 0.0151074, 0, 0, 0),
          SpatialDyn::Joint::Type::RY);

  AddBody(&ab, &ab_rbdl, 1, "forearm",
          Eigen::AngleAxisd::Identity(), Eigen::Vector3d(0, -0.1197, 0.425),
          2.275, Eigen::Vector3d(0, 0, 0.25), Eigen::Vector6d(0.049443313556, 0.049443313556, 0.004095, 0, 0, 0),
          SpatialDyn::Joint::Type::RY);

  AddBody(&ab, &ab_rbdl, 2, "wrist_1",
          Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()), Eigen::Vector3d(0, 0, 0.39225),
          1.219, Eigen::Vector3d(0, 0, 0), Eigen::Vector6d(0.111172755531, 0.111172755531, 0.21942, 0, 0, 0),
          SpatialDyn::Joint::Type::RY);

  AddBody(&ab, &ab_rbdl, 3, "wrist_2",
          Eigen::AngleAxisd::Identity(), Eigen::Vector3d(0, 0.093, 0),
          1.219, Eigen::Vector3d(0, 0, 0), Eigen::Vector6d(0.111172755531, 0.111172755531, 0.21942, 0, 0, 0),
          SpatialDyn::Joint::Type::RZ);

  AddBody(&ab, &ab_rbdl, 4, "wrist_3",
          Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()), Eigen::Vector3d(0, 0, 0.09465),
          0.1879, Eigen::Vector3d(0, 0, 0), Eigen::Vector6d(0.0171364731454, 0.0171364731454, 0.033822, 0, 0, 0),
          SpatialDyn::Joint::Type::RY);

  Eigen::VectorXd q = Eigen::VectorXd::LinSpaced(ab.dof(), 0., 2.);
  ab.set_q(q);
  ab.set_dq(Eigen::VectorXd::LinSpaced(ab.dof(), 0., 1.));
  ab_rbdl.gravity = -9.81 * Eigen::Vector3d::UnitZ();

  SECTION("forward_kinematics") {
    Eigen::Vector3d x = SpatialDyn::Position(ab);
    Eigen::Quaterniond quat = SpatialDyn::Orientation(ab);
    ab.set_q(Eigen::VectorXd::Zero(ab.dof()));
    Eigen::Vector3d x_q = SpatialDyn::Position(ab, q);
    Eigen::Quaterniond quat_q = SpatialDyn::Orientation(ab, q);

    REQUIRE((x - x_q).norm() < 1e-10);
    REQUIRE((quat.coeffs() - quat_q.coeffs()).norm() < 1e-10);
  }

  SECTION("jacobian") {
    Eigen::Matrix6Xd J = SpatialDyn::Jacobian(ab);
    Eigen::Matrix3Xd J_v = SpatialDyn::LinearJacobian(ab);
    Eigen::Matrix3Xd J_w = SpatialDyn::AngularJacobian(ab);
    ab.set_q(Eigen::VectorXd::Zero(ab.dof()));
    Eigen::Matrix6Xd J_q = SpatialDyn::Jacobian(ab, q);
    Eigen::MatrixXd J_rbdl = Eigen::MatrixXd::Zero(6, ab.dof());
    RigidBodyDynamics::CalcPointJacobian6D(ab_rbdl, q, 6, Eigen::Vector3d::Zero(), J_rbdl);

    REQUIRE((J.topRows<3>() - J_rbdl.bottomRows<3>()).norm() < 1e-10);
    REQUIRE((J.bottomRows<3>() - J_rbdl.topRows<3>()).norm() < 1e-10);
    REQUIRE((J.topRows<3>() - J_v).norm() < 1e-10);
    REQUIRE((J.bottomRows<3>() - J_w).norm() < 1e-10);
    REQUIRE((J - J_q).norm() < 1e-10);
  }

  SECTION("inverse dynamics") {
    Eigen::VectorXd ddq = Eigen::VectorXd::Ones(ab.dof());

    Eigen::VectorXd tau = SpatialDyn::InverseDynamics(ab, ddq, {}, true, true);
    Eigen::VectorXd tau_Aq_g = SpatialDyn::InverseDynamics(ab, ddq, {}, true, false);
    Eigen::VectorXd tau_Aq_v = SpatialDyn::InverseDynamics(ab, ddq, {}, false, true);
    Eigen::VectorXd tau_Aq = SpatialDyn::InverseDynamics(ab, ddq, {}, false, false);
    Eigen::VectorXd tau_Aq_v_g_fr = SpatialDyn::InverseDynamics(ab, ddq, {}, true, true, true);

    Eigen::VectorXd tau_crba_Aq_v_g = SpatialDyn::Inertia(ab) * ddq +
                                      SpatialDyn::CentrifugalCoriolis(ab) +
                                      SpatialDyn::Gravity(ab);
    Eigen::VectorXd tau_crba_Aq_g = SpatialDyn::Inertia(ab) * ddq + SpatialDyn::Gravity(ab);
    Eigen::VectorXd tau_crba_Aq_v = SpatialDyn::Inertia(ab) * ddq +
                                    SpatialDyn::CentrifugalCoriolis(ab);
    Eigen::VectorXd tau_crba_Aq = SpatialDyn::Inertia(ab) * ddq;
    Eigen::VectorXd tau_crba_Aq_v_g_fr = SpatialDyn::Inertia(ab) * ddq +
                                         SpatialDyn::CentrifugalCoriolis(ab) +
                                         SpatialDyn::Gravity(ab);
    tau_crba_Aq_v_g_fr += SpatialDyn::Friction(ab, tau_crba_Aq_v_g_fr);

    SpatialDyn::SpatialForced f_ext = SpatialDyn::SpatialForced::Ones();
    Eigen::VectorXd tau_f_ext = SpatialDyn::InverseDynamics(ab, ddq, {{ab.dof()-1, f_ext}}, true, true);

    Eigen::Matrix6Xd J = Eigen::Matrix6Xd::Zero(6, ab.dof());
    for (const int i : ab.ancestors(-1)) {
      J.col(i) = ab.T_to_world(i) * ab.rigid_bodies(i).joint().subspace();
    }
    Eigen::VectorXd tau_crba_f_ext = SpatialDyn::Inertia(ab) * ddq +
                                     SpatialDyn::CentrifugalCoriolis(ab) +
                                     SpatialDyn::Gravity(ab) -
                                     J.transpose() * f_ext.matrix();
    Eigen::VectorXd tau_crba_f_ext_2 = SpatialDyn::Inertia(ab) * ddq +
                                       SpatialDyn::CentrifugalCoriolis(ab) +
                                       SpatialDyn::Gravity(ab) +
                                       SpatialDyn::ExternalTorques(ab, {{ab.dof()-1, f_ext}});

    REQUIRE((tau - tau_crba_Aq_v_g).norm() < 1e-10);
    REQUIRE((tau_Aq_g - tau_crba_Aq_g).norm() < 1e-10);
    REQUIRE((tau_Aq_v - tau_crba_Aq_v).norm() < 1e-10);
    REQUIRE((tau_Aq - tau_crba_Aq).norm() < 1e-10);
    REQUIRE((tau_Aq_v_g_fr - tau_crba_Aq_v_g_fr).norm() < 1e-10);

    REQUIRE((tau_f_ext - tau_crba_f_ext).norm() < 1e-10);
    REQUIRE((tau_f_ext - tau_crba_f_ext_2).norm() < 1e-10);
  }

  SECTION("inertia") {
    Eigen::MatrixXd A = Inertia(ab);
    Eigen::MatrixXd A_rbdl(ab.dof(), ab.dof());
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(ab_rbdl, ab.q(), A_rbdl);

    REQUIRE((A - A_rbdl).norm() < 1e-10);
  }

  SECTION("inertia inverse") {
    Eigen::MatrixXd A_inv = InertiaInverse(ab).solve(Eigen::MatrixXd::Identity(ab.dof(), ab.dof()));
    Eigen::MatrixXd A_inv_aba = InertiaInverseAba(ab);

    REQUIRE((A_inv - A_inv_aba).norm() < 1e-10);
  }

  SECTION("forward dynamics") {
    Eigen::VectorXd tau = Eigen::VectorXd::LinSpaced(ab.dof(), 1., 2.);

    Eigen::VectorXd ddq = SpatialDyn::ForwardDynamics(ab, tau);
    Eigen::VectorXd ddq_aba = SpatialDyn::ForwardDynamicsAba(ab, tau);

    SpatialDyn::SpatialForced f_ext = SpatialDyn::SpatialForced::Ones();
    Eigen::VectorXd ddq_f_ext = SpatialDyn::ForwardDynamics(ab, tau, {{ab.dof()-1, f_ext}});
    Eigen::VectorXd ddq_aba_f_ext = SpatialDyn::ForwardDynamicsAba(ab, tau, {{ab.dof()-1, f_ext}});

    Eigen::VectorXd ddq_no_g = SpatialDyn::ForwardDynamics(ab, tau, {}, false, true);
    Eigen::VectorXd ddq_aba_no_g = SpatialDyn::ForwardDynamicsAba(ab, tau, {}, false, true);

    Eigen::VectorXd ddq_no_cc = SpatialDyn::ForwardDynamics(ab, tau, {}, true, false);
    Eigen::VectorXd ddq_aba_no_cc = SpatialDyn::ForwardDynamicsAba(ab, tau, {}, true, false);

    Eigen::VectorXd ddq_fr = SpatialDyn::ForwardDynamics(ab, tau, {}, false, true, true);
    Eigen::VectorXd ddq_aba_fr = SpatialDyn::ForwardDynamicsAba(ab, tau, {}, false, true, true);

    Eigen::VectorXd ddq_des = tau;
    Eigen::VectorXd tau_cmd = SpatialDyn::InverseDynamics(ab, ddq_des, {{ab.dof()-1, f_ext}}, true, true, true);
    Eigen::VectorXd ddq_res = SpatialDyn::ForwardDynamics(ab, tau_cmd, {{ab.dof()-1, f_ext}}, true, true, true);

    REQUIRE((ddq - ddq_aba).norm() < 1e-10);
    REQUIRE((ddq_f_ext - ddq_aba_f_ext).norm() < 1e-10);
    REQUIRE((ddq_no_g - ddq_aba_no_g).norm() < 1e-10);
    REQUIRE((ddq_no_cc - ddq_aba_no_cc).norm() < 1e-10);
    // REQUIRE((ddq_fr - ddq_aba_fr).norm() < 1e-10);
    REQUIRE((ddq_des - ddq_res).norm() < 1e-10);
  }

  SECTION("centrifugal coriolis") {
    ab_rbdl.gravity.setZero();

    Eigen::VectorXd tau = SpatialDyn::CentrifugalCoriolis(ab);
    Eigen::VectorXd tau_rbdl(ab.dof());
    RigidBodyDynamics::NonlinearEffects(ab_rbdl, ab.q(), ab.dq(), tau_rbdl);

    REQUIRE((tau - tau_rbdl).norm() < 1e-10);
  }

  SECTION("gravity") {
    Eigen::VectorXd tau = SpatialDyn::Gravity(ab);
    Eigen::VectorXd tau_rbdl(ab.dof());
    RigidBodyDynamics::NonlinearEffects(ab_rbdl, ab.q(), Eigen::VectorXd::Zero(ab.dof()), tau_rbdl);

    REQUIRE((tau - tau_rbdl).norm() < 1e-10);
  }

  SECTION("operational space") {
    Eigen::Matrix6Xd J = Jacobian(ab);

    SECTION("inertia") {
      Eigen::Matrix6d Lambda = SpatialDyn::Opspace::Inertia(ab, J);
      Eigen::Matrix6d Lambda_aba = SpatialDyn::Opspace::InertiaAba(ab);
      REQUIRE((Lambda - Lambda_aba).norm() < 1e-10);
    }

    SECTION("centrifugal coriolis") {
      Eigen::Vector6d mu = SpatialDyn::Opspace::CentrifugalCoriolis(ab, J);
      Eigen::Vector6d mu_aba = SpatialDyn::Opspace::CentrifugalCoriolisAba(ab);
      REQUIRE((mu - mu_aba).norm() < 1e-10);
    }

    SECTION("gravity") {
      Eigen::Vector6d p = SpatialDyn::Opspace::Gravity(ab, J);
      Eigen::Vector6d p_aba = SpatialDyn::Opspace::GravityAba(ab);
      REQUIRE((p - p_aba).norm() < 1e-10);
    }

  }
}

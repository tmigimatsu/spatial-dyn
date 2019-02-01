/**
 * main.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 6, 2018
 * Authors: Toki Migimatsu
 */

#include <csignal>   // std::signal, std::sig_atomic_t
#include <iostream>  // std::cout
#include <utility>   // std::move

#include <Eigen/Eigen>
#include <nlohmann/json.hpp>
#include <rbdl/rbdl.h>
#include <spatial_dyn/spatial_dyn.h>

static volatile std::sig_atomic_t g_runloop = true;
void stop(int) { g_runloop = false; }

using json = nlohmann::json;

int main(int argc, char *argv[]) {

  spatial_dyn::ArticulatedBody ab("ur5");
  RigidBodyDynamics::Model ab_rbdl;
  ab.graphics.geometry.type = spatial_dyn::Geometry::Type::MESH;
  ab.graphics.geometry.mesh = "graphics/ur5/base.obj";
  {
    spatial_dyn::RigidBody rb("shoulder_pan");
    rb.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0, 0, 0.089159));
    rb.set_inertia(3.7, Eigen::Vector3d(0, 0, 0.14),
                   Eigen::Vector6d(0.010267495893, 0.010267495893, 0.00666, 0, 0, 0));
    rb.set_joint(spatial_dyn::Joint(spatial_dyn::Joint::Type::RZ));
    rb.graphics.geometry.type = spatial_dyn::Geometry::Type::MESH;
    rb.graphics.geometry.mesh = "graphics/ur5/shoulder.obj";
    ab.AddRigidBody(std::move(rb));

    auto joint_frame = RigidBodyDynamics::Math::Xtrans(Eigen::Vector3d(0, 0, 0.089159));
    RigidBodyDynamics::Body body(3.7, Eigen::Vector3d(0, 0, 0.14),
        RigidBodyDynamics::Math::Matrix3d(Eigen::Matrix3d(Eigen::Vector3d(0.010267495893, 0.010267495893, 0.00666).asDiagonal())));
    RigidBodyDynamics::Joint joint(RigidBodyDynamics::JointType::JointTypeRevolute, Eigen::Vector3d::UnitZ());
    ab_rbdl.AddBody(0, joint_frame, joint, body, "shoulder_pan");
  };
  {
    spatial_dyn::RigidBody rb("shoulder_lift");
    rb.set_T_to_parent(Eigen::Quaterniond(sqrt(2)/2, 0, sqrt(2)/2, 0), Eigen::Vector3d(0, 0.13585, 0));
    rb.set_inertia(8.393, Eigen::Vector3d(0, 0, 0.28),
                   Eigen::Vector6d(0.22689067591, 0.22689067591, 0.0151074, 0, 0, 0));
    rb.set_joint(spatial_dyn::Joint(spatial_dyn::Joint::Type::RY));
    rb.graphics.geometry.type = spatial_dyn::Geometry::Type::MESH;
    rb.graphics.geometry.mesh = "graphics/ur5/upperarm.obj";
    ab.AddRigidBody(std::move(rb), 0);

    auto joint_frame = RigidBodyDynamics::Math::Xrot(M_PI/2, Eigen::Vector3d::UnitY()) * 
                       RigidBodyDynamics::Math::Xtrans(Eigen::Vector3d(0, 0.13585, 0));
    RigidBodyDynamics::Body body(8.393, Eigen::Vector3d(0, 0, 0.28),
        RigidBodyDynamics::Math::Matrix3d(Eigen::Matrix3d(Eigen::Vector3d(0.22689067591, 0.22689067591, 0.0151074).asDiagonal())));
    RigidBodyDynamics::Joint joint(RigidBodyDynamics::JointType::JointTypeRevolute, Eigen::Vector3d::UnitY());
    ab_rbdl.AddBody(1, joint_frame, joint, body, "shoulder_lift");
  }
  {
    spatial_dyn::RigidBody rb("forearm");
    rb.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0, -0.1197, 0.425));
    rb.set_inertia(2.275, Eigen::Vector3d(0, 0, 0.25),
                   Eigen::Vector6d(0.049443313556, 0.049443313556, 0.004095, 0, 0, 0));
    rb.set_joint(spatial_dyn::Joint(spatial_dyn::Joint::Type::RY));
    rb.graphics.geometry.type = spatial_dyn::Geometry::Type::MESH;
    rb.graphics.geometry.mesh = "graphics/ur5/forearm.obj";
    ab.AddRigidBody(std::move(rb), 1);

    auto joint_frame = RigidBodyDynamics::Math::Xtrans(Eigen::Vector3d(0, -0.1197, 0.425));
    RigidBodyDynamics::Body body(2.275, Eigen::Vector3d(0, 0, 0.25),
        RigidBodyDynamics::Math::Matrix3d(Eigen::Matrix3d(Eigen::Vector3d(0.049443313556, 0.049443313556, 0.004095).asDiagonal())));
    RigidBodyDynamics::Joint joint(RigidBodyDynamics::JointType::JointTypeRevolute, Eigen::Vector3d::UnitY());
    ab_rbdl.AddBody(2, joint_frame, joint, body, "shoulder_forearm");
  }
  {
    spatial_dyn::RigidBody rb("wrist_1");
    rb.set_T_to_parent(Eigen::Quaterniond(sqrt(2)/2, 0, sqrt(2)/2, 0), Eigen::Vector3d(0, 0, 0.39225));
    rb.set_inertia(1.219, Eigen::Vector3d(0, 0, 0),
                   Eigen::Vector6d(0.111172755531, 0.111172755531, 0.21942, 0, 0, 0));
    rb.set_joint(spatial_dyn::Joint(spatial_dyn::Joint::Type::RY));
    rb.graphics.geometry.type = spatial_dyn::Geometry::Type::MESH;
    rb.graphics.geometry.mesh = "graphics/ur5/wrist1.obj";
    ab.AddRigidBody(std::move(rb), 2);

    auto joint_frame = RigidBodyDynamics::Math::Xrot(M_PI/2, Eigen::Vector3d::UnitY()) *
                       RigidBodyDynamics::Math::Xtrans(Eigen::Vector3d(0, 0, 0.39225));

    RigidBodyDynamics::Body body(1.219, Eigen::Vector3d(0, 0, 0),
        RigidBodyDynamics::Math::Matrix3d(Eigen::Matrix3d(Eigen::Vector3d(0.111172755531, 0.111172755531, 0.21942).asDiagonal())));
    RigidBodyDynamics::Joint joint(RigidBodyDynamics::JointType::JointTypeRevolute, Eigen::Vector3d::UnitY());
    ab_rbdl.AddBody(3, joint_frame, joint, body, "wrist_1");
  }
  {
    spatial_dyn::RigidBody rb("wrist_2");
    rb.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0, 0.093, 0));
    rb.set_inertia(1.219, Eigen::Vector3d(0, 0, 0),
                   Eigen::Vector6d(0.111172755531, 0.111172755531, 0.21942, 0, 0, 0));
    rb.set_joint(spatial_dyn::Joint(spatial_dyn::Joint::Type::RZ));
    rb.graphics.geometry.type = spatial_dyn::Geometry::Type::MESH;
    rb.graphics.geometry.mesh = "graphics/ur5/wrist2.obj";
    ab.AddRigidBody(std::move(rb), 3);

    auto joint_frame = RigidBodyDynamics::Math::Xtrans(Eigen::Vector3d(0, 0.093, 0));
    RigidBodyDynamics::Body body(1.219, Eigen::Vector3d(0, 0, 0),
        RigidBodyDynamics::Math::Matrix3d(Eigen::Matrix3d(Eigen::Vector3d(0.111172755531, 0.111172755531, 0.21942).asDiagonal())));
    RigidBodyDynamics::Joint joint(RigidBodyDynamics::JointType::JointTypeRevolute, Eigen::Vector3d::UnitZ());
    ab_rbdl.AddBody(4, joint_frame, joint, body, "wrist_2");
  }
  {
    spatial_dyn::RigidBody rb("wrist_3");
    rb.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0, 0, 0.09465));
    rb.set_inertia(0.1879, Eigen::Vector3d(1, 0, 0),
                   Eigen::Vector6d(0.0171364731454, 0.0171364731454, 0.033822, 0, 0, 0));
    rb.set_joint(spatial_dyn::Joint(spatial_dyn::Joint::Type::RY));
    rb.graphics.geometry.type = spatial_dyn::Geometry::Type::MESH;
    rb.graphics.geometry.mesh = "graphics/ur5/wrist3.obj";
    ab.AddRigidBody(std::move(rb), 4);

    auto joint_frame = RigidBodyDynamics::Math::Xtrans(Eigen::Vector3d(0, 0, 0.09465));
    RigidBodyDynamics::Body body(0.1879, Eigen::Vector3d(1, 0, 0),
        RigidBodyDynamics::Math::Matrix3d(Eigen::Matrix3d(Eigen::Vector3d(0.0171364731454, 0.0171364731454, 0.033822).asDiagonal())));
    RigidBodyDynamics::Joint joint(RigidBodyDynamics::JointType::JointTypeRevolute, Eigen::Vector3d::UnitY());
    ab_rbdl.AddBody(5, joint_frame, joint, body, "wrist_3");
  }
  ab_rbdl.gravity = -9.81 * Eigen::Vector3d::UnitZ();
  ab.set_q(Eigen::VectorXd::Zero(ab.dof()));
  ab.set_dq(Eigen::VectorXd::Zero(ab.dof()));
  ab.set_ddq(Eigen::VectorXd::Zero(ab.dof()));
  ab.set_tau(Eigen::VectorXd::Zero(ab.dof()));

  // json links;
  // std::array<std::string, 7> graphics = {
  //   "base.obj",
  //   "shoulder.obj",
  //   "upperarm.obj",
  //   "forearm.obj",
  //   "wrist1.obj",
  //   "wrist2.obj",
  //   "wrist3.obj"
  // };
  // for (int i = 0; i < ab.dof(); i++) {
  //   Eigen::Quaterniond quat = Eigen::Quaterniond(ab.rigid_bodies(i).T_to_parent().linear());
  //   const auto& pos = ab.rigid_bodies(i).T_to_parent().translation();
  //   json link;
  //   link["quat"] = {quat.x(), quat.y(), quat.z(), quat.w()};
  //   link["pos"] = {pos(0), pos(1), pos(2)};
  //   link["type"] = std::string(ab.rigid_bodies(i).joint());
  //   link["graphics"] = "graphics/ur5/" + graphics[i+1];
  //   links.push_back(std::move(link));
  // }
  // json robot;
  // robot["links"] = links;
  // std::cout << robot << std::endl;
  spatial_dyn::Timer timer(6000);
  spatial_dyn::RedisClient redis_client;
  redis_client.connect();
  // redis_client.sync_set("spatialdyn::robot", robot);
  // redis_client.sync_set("spatialdyn::sensor::q", ab.q().toMatlab());

  std::signal(SIGINT, stop);

  // ab.set_q(Eigen::VectorXd::Zero(6));
  // ab.set_dq(Eigen::VectorXd::Ones(6));
  // std::cout << std::endl;
  // std::cout << "Jacobian:" << std::endl;
  // std::cout << Jacobian(ab, 5) << std::endl << std::endl;
  // Eigen::MatrixXd J_rbdl = Eigen::MatrixXd::Zero(6, ab.dof());
  // RigidBodyDynamics::CalcPointJacobian6D(ab_rbdl, Eigen::VectorXd::Zero(6), 6, Eigen::Vector3d::Zero(), J_rbdl);
  // std::cout << J_rbdl << std::endl << std::endl;


  // std::cout << "InverseDynamics:" << std::endl;
  // std::cout << InverseDynamics(ab, Eigen::VectorXd::Ones(6)).transpose() << std::endl;
  Eigen::VectorXd tau_rbdl(6);
  // RigidBodyDynamics::InverseDynamics(ab_rbdl, Eigen::VectorXd::Zero(6), Eigen::VectorXd::Ones(6), Eigen::VectorXd::Ones(6), tau_rbdl);

  // std::cout << "Inertia:" << std::endl;
  // std::cout << Inertia(ab) << std::endl << std::endl;
  Eigen::MatrixXd A_rbdl(6,6);
  // RigidBodyDynamics::CompositeRigidBodyAlgorithm(ab_rbdl, Eigen::VectorXd::Zero(6), A_rbdl);
  // std::cout << A_rbdl << std::endl << std::endl;

  // std::cout << "ForwardDynamics:" << std::endl;
  // std::cout << ForwardDynamics(ab, Eigen::VectorXd::Ones(6)).transpose() << std::endl;
  Eigen::VectorXd ddq_rbdl(6);
  // RigidBodyDynamics::ForwardDynamics(ab_rbdl, Eigen::VectorXd::Zero(6), Eigen::VectorXd::Ones(6), Eigen::VectorXd::Ones(6), ddq_rbdl);
  // std::cout << ddq_rbdl.transpose() << std::endl << std::endl;

  // ab.set_q(Eigen::VectorXd::Constant(ab.dof(), 0));
  // ab.set_dq(Eigen::VectorXd::Constant(ab.dof(), 0.2));
  // ab.set_g(Eigen::Vector3d(0, 0, -9.81));

  // std::cout << "opspaceDynamics:" << std::endl;
  // auto J = Jacobian(ab);
  // std::cout << spatial_dyn::opspace::JacobianDynamicInverse(ab, Jacobian(ab)) << std::endl << std::endl;
  // ab.set_g(Eigen::Vector3d::Zero());
  // std::cout << spatial_dyn::opspace::CentrifugalCoriolisGravity(ab).transpose() << std::endl << std::endl;
  // std::cout << spatial_dyn::opspace::CentrifugalCoriolis(ab).transpose() << std::endl << std::endl;
  // std::cout << spatial_dyn::opspace::Gravity(ab, Jacobian(ab)).transpose() << std::endl << std::endl;
  // std::cout << "Gravity:" << std::endl;
  // std::cout << spatial_dyn::opspace::GravityAba(ab).transpose() << std::endl;
  // std::cout << spatial_dyn::opspace::Gravity(ab, J).transpose() << std::endl;

  // std::cout << "CentrifugalCoriolis:" << std::endl;
  // std::cout << spatial_dyn::opspace::CentrifugalCoriolisAba(ab).transpose() << std::endl;
  // std::cout << spatial_dyn::opspace::CentrifugalCoriolis(ab, J).transpose() << std::endl;

  spatial_dyn::ArticulatedBody ab_urdf = spatial_dyn::urdf::LoadModel("../resources/kuka_iiwa/kuka_iiwa.urdf");
  std::cout << ab_urdf.name << std::endl;
  std::cout << "dof: " << ab_urdf.dof() << std::endl;
  std::cout << "T_base_to_world:" << std::endl << ab_urdf.T_base_to_world().matrix() << std::endl;
  std::cout << std::endl;
  const auto& rbs = ab_urdf.rigid_bodies();
  for (const auto& rb : rbs) {
    std::cout << "Rigid body " << rb.id() << ": " << rb.name << std::endl;
    std::cout << "  Parent: " << rb.id_parent() << std::endl;
    std::cout << "  T_to_parent:" << std::endl << rb.T_to_parent().matrix() << std::endl;
    std::cout << "  Mass: " << rb.inertia().mass << std::endl;
    std::cout << "  Com: " << rb.inertia().com.transpose() << std::endl;
    std::cout << "  Inertia:" << std::endl << rb.inertia().I_com << std::endl;
    std::cout << "  Joint: " << std::string(rb.joint()) << std::endl;
    std::cout << "    Limits: " << rb.joint().q_min() << " " << rb.joint().q_max() << " " << rb.joint().dq_max() << " " << rb.joint().fq_max() << std::endl;
    std::cout << "    Friction: " << rb.joint().f_coulomb() << " " << rb.joint().f_stiction() << std::endl;
    std::cout << "  Graphics: " << rb.graphics.name << std::endl;
    std::cout << "    T_to_parent:" << std::endl << rb.graphics.T_to_parent.matrix() << std::endl;
    std::cout << "    Geometry: " << std::string(rb.graphics.geometry) << std::endl;
    std::cout << "      Mesh: " << rb.graphics.geometry.mesh << std::endl;
    std::cout << "      Scale: " << rb.graphics.geometry.scale.transpose() << std::endl;
    std::cout << std::endl;
  }

  ab = ab_urdf;
  redis_client.sync_set("spatialdyn::robot2", spatial_dyn::json::Serialize(ab).dump());

  // Time trials
  // auto t_start = std::chrono::high_resolution_clock::now();
  // for (int i = 0; i < 1000000; i++) {
  //   ab.set_q(Eigen::VectorXd::Constant(ab.dof(), 0));
  //   CentrifugalCoriolis(ab);
  //   Gravity(ab);
  //   // Jacobian(ab, 5);
  //   // InverseDynamics(ab, Eigen::VectorXd::Ones(6));
  //   Inertia(ab);
  //   // ForwardDynamics(ab, Eigen::VectorXd::Ones(6));
  //   ForwardDynamicsAba(ab, Eigen::VectorXd::Ones(6));
  //   // spatial_dyn::opspace::InertiaAba(ab);
  // }
  // auto t_end = std::chrono::high_resolution_clock::now();
  // std::cout << "spatial_dyn: \t" << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count() << std::endl;

  // t_start = std::chrono::high_resolution_clock::now();
  // for (int i = 0; i < 1000000; i++) {
  //   // RigidBodyDynamics::CalcPointJacobian6D(ab_rbdl, Eigen::VectorXd::Zero(6), 6, Eigen::Vector3d::Zero(), J_rbdl);
  //   RigidBodyDynamics::InverseDynamics(ab_rbdl, Eigen::VectorXd::Zero(6), Eigen::VectorXd::Ones(6), Eigen::VectorXd::Ones(6), tau_rbdl);
  // RigidBodyDynamics::CompositeRigidBodyAlgorithm(ab_rbdl, Eigen::VectorXd::Zero(6), A_rbdl);
  //   RigidBodyDynamics::ForwardDynamics(ab_rbdl, Eigen::VectorXd::Zero(6), Eigen::VectorXd::Ones(6), Eigen::VectorXd::Ones(6), ddq_rbdl);
  //   // spatial_dyn::opspace::Inertia(ab, Jacobian(ab));
  // }
  // t_end = std::chrono::high_resolution_clock::now();
  // std::cout << "RBDL: \t\t" << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count() << std::endl;

  // Eigen::VectorXd q_des(6);
  // q_des.fill(0.8);
  ab.set_q(Eigen::VectorXd::Zero(ab.dof()));
  Eigen::VectorXd q_des(ab.dof());
  q_des << 90., -30., 0., 60., 0., -90., -60.;
  q_des *= M_PI / 180.;
  while (g_runloop) {
    timer.Sleep();
    Eigen::VectorXd q_err = ab.q() - q_des;
    Eigen::VectorXd dq_err = ab.dq();
    Eigen::VectorXd ddq = -10 * q_err - 6 * dq_err;
    Eigen::VectorXd tau = spatial_dyn::InverseDynamics(ab, ddq, {}, true, true);

    spatial_dyn::Integrate(ab, tau, timer.dt());
    redis_client.set("spatialdyn::sensor::q", ab.q().toMatlab());
    redis_client.commit();

    if (q_err.norm() < 0.001 && dq_err.norm() < 0.001) break;
  }

  Eigen::Vector3d x_des_0 = spatial_dyn::Position(ab, -1);
  Eigen::Quaterniond quat_des = spatial_dyn::Orientation(ab, -1);
  while (g_runloop) {
    timer.Sleep();
    Eigen::Matrix6Xd J = spatial_dyn::Jacobian(ab, -1);

    Eigen::Vector3d x_des = x_des_0;
    x_des(0) += 0.2 * sin(2*M_PI / 2 * timer.time_sim());
    Eigen::Vector3d x_err = spatial_dyn::Position(ab, -1) - x_des;
    Eigen::Vector3d dx_err = J.topRows<3>() * ab.dq();
    Eigen::Vector3d ddx = -40 * x_err - 13 * dx_err;

    Eigen::Vector3d ori_err = spatial_dyn::opspace::OrientationError(Orientation(ab, -1), quat_des);
    Eigen::Vector3d w_err = J.bottomRows<3>() * ab.dq();
    Eigen::Vector3d dw = -10 * ori_err - 10 * w_err;

    Eigen::Vector6d ddx_dw;
    ddx_dw << ddx, dw;
    Eigen::MatrixXd N;
    Eigen::VectorXd tau = spatial_dyn::opspace::InverseDynamics(ab, J, ddx_dw, &N, {}, true, true);

    static const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(ab.dof(), ab.dof());
    Eigen::VectorXd q_err = ab.q() - q_des;
    Eigen::VectorXd dq_err = ab.dq();
    Eigen::VectorXd ddq = -16 * q_err - 8 * dq_err;
    tau += spatial_dyn::opspace::InverseDynamics(ab, I, ddq, &N, {}, true);

    spatial_dyn::Integrate(ab, tau, timer.dt());
    redis_client.set("spatialdyn::sensor::q", ab.q().toMatlab());
    redis_client.commit();
  }
  std::cout << "Simulated " << timer.time_sim() << "s in " << timer.time_elapsed() << "s." << std::endl;

}

/**
 * main.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: January 29, 2019
 * Authors: Toki Migimatsu
 */

#include <cmath>      // std::sin, std::cos
#include <csignal>    // std::signal, std::sig_atomic_t
#include <exception>  // std::exception
#include <future>     // std::future
#include <iostream>   // std::cout
#include <string>     // std::string

#include <spatial_dyn/spatial_dyn.h>
#include <ctrl_utils/control.h>
#include <ctrl_utils/filesystem.h>
#include <ctrl_utils/json.h>
#include <ctrl_utils/redis_client.h>
#include <ctrl_utils/timer.h>

namespace {

volatile std::sig_atomic_t g_runloop = true;

// Signal handler
void stop(int) {
  g_runloop = false;
}

// Urdf
const std::string kPathUrdf = "../../../resources/kuka_iiwa/kuka_iiwa.urdf";

// Redis keys
const std::string KEY_PREFIX         = "spatial_dyn::";
const std::string KEY_MODELS_PREFIX  = KEY_PREFIX + "model::";
const std::string KEY_OBJECTS_PREFIX = KEY_PREFIX + "object::";

// GET keys
const std::string KEY_SENSOR_Q  = KEY_PREFIX + "sensor::q";
const std::string KEY_SENSOR_DQ = KEY_PREFIX + "sensor::dq";

// SET keys
const std::string KEY_CONTROL_TAU = KEY_PREFIX + "control::tau";
const std::string KEY_CONTROL_POS = KEY_PREFIX + "control::pos";
const std::string KEY_TRAJ_POS    = KEY_PREFIX + "trajectory::pos";
const std::string KEY_TRAJ_ORI    = KEY_PREFIX + "trajectory::ori";
const std::string KEY_MODEL       = KEY_PREFIX + "model";

// Webapp keys
const std::string kNameApp            = "simulator";
const std::string KEY_WEB_RESOURCES   = "webapp::resources";
const std::string KEY_WEB_ARGS        = "webapp::" + kNameApp + "::args";
const std::string KEY_WEB_INTERACTION = "webapp::" + kNameApp + "::interaction";

// Controller gains
const std::string KEY_KP_KV_POS   = KEY_PREFIX + "control::kp_kv_pos";
const std::string KEY_KP_KV_ORI   = KEY_PREFIX + "control::kp_kv_ori";
const std::string KEY_KP_KV_JOINT = KEY_PREFIX + "control::kp_kv_joint";

// Controller parameters
const Eigen::Vector3d kEeOffset  = Eigen::Vector3d(0., 0., 0.03);
const Eigen::VectorXd kQHome     = (M_PI / 180.) * (Eigen::Matrix<double,7,1>() <<
                                   90., -30., 0., 60., 0., -90., 0.).finished();
const Eigen::Vector2d kKpKvPos   = Eigen::Vector2d(40., 10.);
const Eigen::Vector2d kKpKvOri   = Eigen::Vector2d(40., 10.);
const Eigen::Vector2d kKpKvJoint = Eigen::Vector2d(1., 2.);
const double kTimerFreq          = 1000.;  // Max ~2200
const double kGainKeyPress       = 0.1 / kTimerFreq;
const double kGainClickDrag      = 100.;

// General PD control law
template<typename Derived1, typename Derived2, typename Derived3>
typename Derived1::PlainObject PdControl(const Eigen::MatrixBase<Derived1>& x,
                                         const Eigen::MatrixBase<Derived2>& x_des,
                                         const Eigen::MatrixBase<Derived3>& dx,
                                         const Eigen::Vector2d& kp_kv) {
  return -kp_kv(0) * (x - x_des) - kp_kv(1) * dx;
}

// Special PD control law for orientation
Eigen::Vector3d PdControl(const Eigen::Quaterniond& quat,
                          const Eigen::Quaterniond& quat_des,
                          Eigen::Ref<const Eigen::Vector3d> w,
                          const Eigen::Vector2d& kp_kv) {
  return -kp_kv(0) * ctrl_utils::OrientationError(quat, quat_des) - kp_kv(1) * w;
}

void InitializeWebApp(ctrl_utils::RedisClient& redis_client, const spatial_dyn::ArticulatedBody& ab);

std::map<size_t, spatial_dyn::SpatialForced> ComputeExternalForces(const spatial_dyn::ArticulatedBody& ab,
                                                                   const nlohmann::json& interaction);

void AdjustPosition(const std::string& key, Eigen::Vector3d* pos);

}  // namespace

int main(int argc, char* argv[]) {

  // Create timer and Redis client
  ctrl_utils::Timer timer(kTimerFreq);
  ctrl_utils::RedisClient redis_client;
  redis_client.connect();

  // Initialize robot
  spatial_dyn::ArticulatedBody ab = spatial_dyn::urdf::LoadModel(kPathUrdf);
  ab.set_q(kQHome);

  // Initialize controller parameters
  Eigen::VectorXd q_des       = kQHome;
  Eigen::Vector3d x_des       = spatial_dyn::Position(ab, -1, kEeOffset);
  Eigen::Quaterniond quat_des = spatial_dyn::Orientation(ab);

  // Initialize Redis keys
  InitializeWebApp(redis_client, ab);
  redis_client.set(KEY_SENSOR_Q, ab.q());
  redis_client.set(KEY_KP_KV_POS, kKpKvPos);
  redis_client.set(KEY_KP_KV_ORI, kKpKvOri);
  redis_client.set(KEY_KP_KV_JOINT, kKpKvJoint);
  redis_client.sync_commit();

  // Create signal handler
  std::signal(SIGINT, &stop);

  try {
    while (g_runloop) {
      // Wait for next loop
      timer.Sleep();

      // Get Redis values
      std::future<Eigen::Vector2d> fut_kp_kv_pos   = redis_client.get<Eigen::Vector2d>(KEY_KP_KV_POS);
      std::future<Eigen::Vector2d> fut_kp_kv_ori   = redis_client.get<Eigen::Vector2d>(KEY_KP_KV_ORI);
      std::future<Eigen::Vector2d> fut_kp_kv_joint = redis_client.get<Eigen::Vector2d>(KEY_KP_KV_JOINT);
      std::future<nlohmann::json> fut_interaction  = redis_client.get<nlohmann::json>(KEY_WEB_INTERACTION);
      redis_client.commit();

      // Compute Jacobian
      const Eigen::Matrix6Xd& J = spatial_dyn::Jacobian(ab, -1, kEeOffset);

      // Compute position PD control
      Eigen::Vector3d x   = spatial_dyn::Position(ab, -1, kEeOffset);
      Eigen::Vector3d dx  = J.topRows<3>() * ab.dq();
      Eigen::Vector3d ddx = PdControl(x, x_des, dx, fut_kp_kv_pos.get());

      // Compute orientation PD control
      Eigen::Quaterniond quat = ctrl_utils::NearQuaternion(spatial_dyn::Orientation(ab), quat_des);
      Eigen::Vector3d w       = J.bottomRows<3>() * ab.dq();
      Eigen::Vector3d dw      = PdControl(quat, quat_des, w, fut_kp_kv_ori.get());

      // Compute opspace torques
      Eigen::MatrixXd N;
      Eigen::VectorXd tau_cmd;
      if (spatial_dyn::opspace::IsSingular(ab, J)) {
        // If robot is at a singularity, control position only
        tau_cmd = spatial_dyn::opspace::InverseDynamics(ab, J.topRows<3>(), ddx, &N);
      } else {
        // Control position and orientation
        Eigen::Vector6d ddx_dw;
        ddx_dw << ddx, dw;
        tau_cmd = spatial_dyn::opspace::InverseDynamics(ab, J, ddx_dw, &N);
      }

      // Add joint task in nullspace
      static const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(ab.dof(), ab.dof());
      Eigen::VectorXd ddq = PdControl(ab.q(), q_des, ab.dq(), fut_kp_kv_joint.get());
      tau_cmd += spatial_dyn::opspace::InverseDynamics(ab, I, ddq, &N);

      // Add gravity compensation
      tau_cmd += spatial_dyn::Gravity(ab);

      // Parse interaction from web app
      nlohmann::json interaction = fut_interaction.get();
      std::map<size_t, spatial_dyn::SpatialForced> f_ext = ComputeExternalForces(ab, interaction);
      AdjustPosition(interaction["key_down"].get<std::string>(), &x_des);

      // Integrate
      // spatial_dyn::Integrate(ab, tau_cmd, timer.dt(), f_ext);
      spatial_dyn::discrete::Integrate(ab, tau_cmd, timer.dt(), f_ext);

      // Set Redis values
      redis_client.set(KEY_CONTROL_TAU, tau_cmd);
      redis_client.set(KEY_CONTROL_POS, x_des);
      redis_client.set(KEY_SENSOR_Q, ab.q());
      redis_client.set(KEY_SENSOR_DQ, ab.dq());
      redis_client.set(KEY_TRAJ_POS, x);
      redis_client.set(KEY_TRAJ_ORI, quat.coeffs());
      redis_client.sync_commit();
    }
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  // Clear torques
  redis_client.set(KEY_CONTROL_TAU, Eigen::VectorXd::Zero(ab.dof()));

  std::cout << "Simulated " << timer.time_sim() << "s in " << timer.time_elapsed() << "s." << std::endl;

  return 0;
}

namespace {

void InitializeWebApp(ctrl_utils::RedisClient& redis_client, const spatial_dyn::ArticulatedBody& ab) {
  // Register the urdf path so the server knows it's safe to fulfill requests for files in that directory.
  std::string path_urdf = ctrl_utils::AbsolutePath(ctrl_utils::CurrentPath() + "/" + kPathUrdf);
  redis_client.hset(KEY_WEB_RESOURCES, kNameApp, ctrl_utils::ParentPath(path_urdf));

  // Register key prefixes so the web app knows which models and objects to render.
  nlohmann::json web_keys;
  web_keys["key_models_prefix"]  = KEY_MODELS_PREFIX;
  web_keys["key_objects_prefix"] = KEY_OBJECTS_PREFIX;
  redis_client.set(KEY_WEB_ARGS, web_keys);

  // Register the robot
  nlohmann::json web_model;
  web_model["model"]    = ab;
  web_model["key_q"]    = KEY_SENSOR_Q;
  web_model["key_traj"] = KEY_TRAJ_POS;
  redis_client.set(KEY_MODELS_PREFIX + ab.name, web_model);

  // Create a sphere marker for x_des
  nlohmann::json web_object;
  spatial_dyn::Graphics x_des_marker("x_des_marker");
  x_des_marker.geometry.type = spatial_dyn::Graphics::Geometry::Type::kSphere;
  x_des_marker.geometry.radius = 0.01;
  web_object["graphics"] = std::vector<spatial_dyn::Graphics>{ x_des_marker };
  web_object["key_pos"]  = KEY_CONTROL_POS;
  redis_client.set(KEY_OBJECTS_PREFIX + x_des_marker.name, web_object);
}

std::map<size_t, spatial_dyn::SpatialForced> ComputeExternalForces(const spatial_dyn::ArticulatedBody& ab,
                                                                   const nlohmann::json& interaction) {
  std::map<size_t, spatial_dyn::SpatialForced> f_ext;

  // Check if the clicked object is the robot
  std::string key_object = interaction["key_object"].get<std::string>();
  if (key_object != KEY_MODELS_PREFIX + ab.name) return f_ext;

  // Extract the json fields
  size_t idx_link = interaction["idx_link"].get<size_t>();
  Eigen::Vector3d pos_mouse = interaction["pos_mouse_in_world"].get<Eigen::Vector3d>();
  Eigen::Vector3d pos_click = interaction["pos_click_in_link"].get<Eigen::Vector3d>();

  // Get the click position in world coordinates
  Eigen::Vector3d pos_click_in_world = spatial_dyn::Position(ab, idx_link, pos_click);

  // Set the click force
  Eigen::Vector3d f = kGainClickDrag * (pos_mouse - pos_click_in_world);
  spatial_dyn::SpatialForced f_click(f, Eigen::Vector3d::Zero());

  // Translate the spatial force to the world frame
  f_ext[idx_link] = Eigen::Translation3d(pos_click_in_world) * f_click;

  return f_ext;
}

void AdjustPosition(const std::string& key, Eigen::Vector3d* pos) {
  if (key.empty() || pos == nullptr) return;

  size_t idx = 0;
  int sign = 1;
  switch (key[0]) {
    case 'a': idx = 0; sign = -1; break;
    case 'd': idx = 0; sign = 1; break;
    case 'w': idx = 1; sign = 1; break;
    case 's': idx = 1; sign = -1; break;
    case 'e': idx = 2; sign = 1; break;
    case 'q': idx = 2; sign = -1; break;
    default: return;
  }
  (*pos)(idx) += sign * kGainKeyPress;
}

}  // namespace

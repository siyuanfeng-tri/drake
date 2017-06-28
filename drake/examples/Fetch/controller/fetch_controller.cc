#include "drake/examples/Fetch/controller/fetch_controller.h"

namespace drake {
namespace examples {
namespace Fetch {

using systems::Context;
using systems::BasicVector;

//constexpr int kWheelQStart = 7;
constexpr int kWheelVStart = 6;

constexpr int kHeadQStart = 10;
constexpr int kHeadVStart = 9;

constexpr int kTorsoQStart = 9;
constexpr int kTorsoVStart = 8;

constexpr int kArmQStart = 12;
constexpr int kArmVStart = 11;

constexpr int kHandQStart = 19;
constexpr int kHandVStart = 18;

template <typename T>
FetchController<T>::FetchController(
    std::unique_ptr<RigidBodyTree<T>> full_robot)
    : full_robot_(std::move(full_robot)),
      base_link_(full_robot_->FindBody("base_link")) {
  kp_arm_.setConstant(10);
  kd_arm_.setConstant(5);

  kp_head_.setConstant(10);
  kd_head_.setConstant(5);

  kp_hand_.setConstant(10);
  kd_hand_.setConstant(5);

  kp_torso_ = 10;
  kd_torso_ = 5;

  lin_v_gain_ = 5;
  // The magic number is wheel to center.
  omega_v_gain_ = 5 * 0.18738;
}

template <typename T>
Vector2<T> FetchController<T>::CalcWheelTorque(const KinematicsCache<T>& cache,
                                               const T v_d, const T w_d) const {
  Vector6<T> V_WB =
      full_robot_->CalcBodySpatialVelocityInWorldFrame(cache, *base_link_);
  Isometry3<T> X_WB =
      full_robot_->CalcBodyPoseInWorldFrame(cache, *base_link_);

  Vector6<T> V_WB_B;
  V_WB_B.template head<3>() = X_WB.linear().transpose() * V_WB.template head<3>();
  V_WB_B.template tail<3>() = X_WB.linear().transpose() * V_WB.template tail<3>();

  // xdot in body frame.
  T v = V_WB_B[3];
  // yawdot in body frame.
  T w = V_WB_B[2];

  std::cout << "(v, w)" << v << " " << w << "\n";

  T R = lin_v_gain_ * (v_d - v) + omega_v_gain_ * (w_d - w);
  T L = lin_v_gain_ * (v_d - v) - omega_v_gain_ * (w_d - w);

  return Vector2<T>(R, L);
}

template <typename T>
Vector2<T> FetchController<T>::CalcHeadAcc(const KinematicsCache<T>& cache,
                                           const Vector2<T>& q_d,
                                           const Vector2<T>& v_d,
                                           const Vector2<T>& vd_d) const {
  auto q = cache.getQ().template segment<2>(kHeadQStart);
  auto v = cache.getV().template segment<2>(kHeadVStart);

  Vector2<T> acc = (kp_head_.array() * (q_d - q).array() +
                    kd_head_.array() * (v_d - v).array())
                       .matrix() +
                   vd_d;
  return acc;
}

template <typename T>
Vector2<T> FetchController<T>::CalcHandTorque(const KinematicsCache<T>& cache,
                                           const Vector2<T>& q_d) const {
  auto q = cache.getQ().template segment<2>(kHandQStart);
  auto v = cache.getV().template segment<2>(kHandVStart);

  Vector2<T> trq = (kp_hand_.array() * (q_d - q).array() -
                    kd_hand_.array() * v.array()).matrix();
  return trq;
}

template <typename T>
T FetchController<T>::CalcTorsoAcc(const KinematicsCache<T>& cache, const T q_d,
                                   const T v_d, const T vd_d) const {
  T q = cache.getQ()[kTorsoQStart];
  T v = cache.getV()[kTorsoVStart];

  T acc = kp_torso_ * (q_d - q) + kd_torso_ * (v_d - v) + vd_d;
  return acc;
}

template <typename T>
VectorX<T> FetchController<T>::CalcArmAcc(const KinematicsCache<T>& cache,
                                          const VectorX<T>& q_d,
                                          const VectorX<T>& v_d,
                                          const VectorX<T>& vd_d) const {
  auto q = cache.getQ().template segment<7>(kArmQStart);
  auto v = cache.getV().template segment<7>(kArmVStart);

  VectorX<T> acc = (kp_arm_.array() * (q_d - q).array() +
                    kd_arm_.array() * (v_d - v).array())
                       .matrix() +
                   vd_d;
  return acc;
}

template <typename T>
VectorX<T> FetchController<T>::CalcTorque(const VectorX<T>& acc,
                                          KinematicsCache<T>* cache) const {
  DRAKE_DEMAND(acc.size() == full_robot_->get_num_velocities());
  eigen_aligned_std_unordered_map<RigidBody<T> const*, Vector6<T>> f_ext;

  // Just the actuated parts.
  VectorX<T> torque = full_robot_->inverseDynamics(*cache, f_ext, acc)
                          .segment(kWheelVStart, full_robot_->get_num_actuators());
  return torque;
}

template <typename T>
FetchControllerSystem<T>::FetchControllerSystem(
    std::unique_ptr<RigidBodyTree<T>> full_robot)
    : controller_(std::move(full_robot)) {

  const auto& robot = controller_.get_full_robot();

  input_port_index_full_estimated_state_ =
      this->DeclareInputPort(systems::kVectorValued, robot.get_num_positions() + robot.get_num_velocities())
          .get_index();

  output_port_index_control_ =
      this->DeclareVectorOutputPort(BasicVector<T>(robot.get_num_actuators()),
                                    &FetchControllerSystem<T>::CalcOutputTorque)
          .get_index();
}

template <typename T>
void FetchControllerSystem<T>::CalcOutputTorque(const Context<T>& context,
                                                BasicVector<T>* output) const {
  /*
  output->get_mutable_value().setZero();
  */
  const auto& robot = controller_.get_full_robot();
  // Do kinematics.
  VectorX<T> x = this->EvalEigenVectorInput(
      context, input_port_index_full_estimated_state_);
  KinematicsCache<T> cache = robot.CreateKinematicsCache();
  cache.initialize(x.head(robot.get_num_positions()),
                   x.tail(robot.get_num_velocities()));
  robot.doKinematics(cache, true);

  VectorX<T> vd_d = VectorX<T>::Zero(robot.get_num_velocities());
  // Torso acc
  vd_d[kTorsoVStart] = controller_.CalcTorsoAcc(cache, 0.2, 0, 0);

  // Head acc
  vd_d.template segment<2>(kHeadVStart) =
      controller_.CalcHeadAcc(cache,
          Vector2<T>::Zero(), Vector2<T>::Zero(), Vector2<T>::Zero());

  // Arms acc
  VectorX<T> arm_q = VectorX<T>::Zero(7);
  arm_q[0] = 0.3;

  vd_d.template segment<7>(kArmVStart) = controller_.CalcArmAcc(
      cache, arm_q, VectorX<T>::Zero(7), VectorX<T>::Zero(7));

  // Call ID.
  output->get_mutable_value() = controller_.CalcTorque(vd_d, &cache);

  // Wheels
  output->get_mutable_value().template head<2>() =
      controller_.CalcWheelTorque(cache, 0.2, 0.2);

  // Hand
  output->get_mutable_value().template tail<2>() =
      controller_.CalcHandTorque(cache, Vector2<T>::Zero());
}

template class FetchController<double>;
template class FetchControllerSystem<double>;

}  // namespace Fetch
}  // namespace examples
}  // namespace drake

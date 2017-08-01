#include "drake/examples/kuka_iiwa_arm/jjz_primitives.h"

namespace drake {
namespace examples {
namespace jjz {

///////////////////////////////////////////////////////////
MoveJoint::MoveJoint(const std::string& name, const VectorX<double>& q_d, double duration)
    : FSMState(name), duration_(duration), q_end_(q_d) {}

void MoveJoint::DoInitialize(const IiwaState& state) {
  std::vector<double> times = {0, duration_};
  std::vector<MatrixX<double>> knots = {state.get_q(), q_end_};
  const int size = state.get_q().size();
  MatrixX<double> zero = MatrixX<double>::Zero(size, 1);
  traj_ = PiecewisePolynomial<double>::Cubic(times, knots, zero, zero);
}

void MoveJoint::Control(const IiwaState& state, Eigen::Ref<VectorX<double>> q_d) const {
  const double interp_time = get_in_state_time(state);
  q_d = traj_.value(interp_time);
}

bool MoveJoint::IsDone(const IiwaState& state) const {
  return (get_in_state_time(state) > duration_ + 0.5);
}

///////////////////////////////////////////////////////////
MoveTool::MoveTool(const std::string& name, const RigidBodyTree<double>* robot,
    const VectorX<double>& q0, const VectorX<double>& q_norm)
  : FSMState(name), robot_(*robot),
  frame_T_("tool", robot_.FindBody(jjz::kEEName), jjz::X_ET),
  cache_(robot_.CreateKinematicsCache()),
  jaco_planner_(robot), q_norm_(q_norm) {
    cache_.initialize(q0);
    robot_.doKinematics(cache_);

    gain_T_ = Vector6<double>::Constant(1);
    gain_T_(1) = 0; // no pitch tracking.
}

void MoveTool::DoInitialize(const IiwaState& state) {
  last_time_ = state.get_time();
}

void MoveTool::Update(const IiwaState& state) {
  Isometry3<double> X_WT_d = ComputeDesiredToolInWorld(state);
  Isometry3<double> X_WT = robot_.CalcFramePoseInWorldFrame(cache_, frame_T_);

  const double dt = state.get_time() - last_time_;
  last_time_ = state.get_time();

  if (dt <= 0)
    return;

  Vector6<double> V_WT_d =
    jaco_planner_.ComputePoseDiffInWorldFrame(X_WT, X_WT_d) / dt;

  VectorX<double> v = jaco_planner_.ComputeDofVelocity(
      cache_, frame_T_, V_WT_d, q_norm_, dt, gain_T_);

  cache_.initialize(cache_.getQ() + v * dt);
  robot_.doKinematics(cache_);
}

void MoveTool::Control(const IiwaState& state, Eigen::Ref<VectorX<double>> q_d) const {
  q_d = cache_.getQ();
}

///////////////////////////////////////////////////////////
MoveToolFollowTraj::MoveToolFollowTraj(const std::string& name, const RigidBodyTree<double>* robot,
    const VectorX<double>& q0, const VectorX<double>& q_norm,
    const manipulation::PiecewiseCartesianTrajectory<double>& traj)
  : MoveTool(name, robot, q0, q_norm), ee_traj_(traj) {
}

Isometry3<double> MoveToolFollowTraj::ComputeDesiredToolInWorld(const IiwaState& state) const {
  const double interp_t = get_in_state_time(state);
  const Isometry3<double> X_GT = ee_traj_.get_pose(interp_t);
  return jjz::X_WG * X_GT;
}

bool MoveToolFollowTraj::IsDone(const IiwaState& state) const {
  const double duration = ee_traj_.get_position_trajectory().get_end_time();
  return (get_in_state_time(state) > duration + 0.5);
}


}  // namespace jjz
}  // namespace examples
}  // namespace drake

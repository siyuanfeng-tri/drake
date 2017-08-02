#include "drake/examples/kuka_iiwa_arm/jjz_primitives.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace examples {
namespace jjz {

///////////////////////////////////////////////////////////
MoveJoint::MoveJoint(const std::string& name, const VectorX<double>& q0,
                     const VectorX<double>& q1, double duration)
    : FSMState(name) {
  DRAKE_DEMAND(q0.size() == q1.size());
  std::vector<double> times = {0, duration};
  std::vector<MatrixX<double>> knots = {q0, q1};
  MatrixX<double> zero = MatrixX<double>::Zero(q0.size(), 1);
  traj_ = PiecewisePolynomial<double>::Cubic(times, knots, zero, zero);
}

void MoveJoint::Control(const IiwaState& state,
                        Eigen::Ref<VectorX<double>> q_d) const {
  const double interp_time = get_in_state_time(state);
  q_d = traj_.value(interp_time);
}

bool MoveJoint::IsDone(const IiwaState& state) const {
  return get_in_state_time(state) > (traj_.getEndTime() + 0.5);
}

///////////////////////////////////////////////////////////
MoveTool::MoveTool(const std::string& name, const RigidBodyTree<double>* robot,
                   const VectorX<double>& q0)
    : FSMState(name),
      robot_(*robot),
      frame_T_("tool", robot_.FindBody(jjz::kEEName), jjz::X_ET),
      cache_(robot_.CreateKinematicsCache()),
      jaco_planner_(robot),
      q_norm_(robot_.getZeroConfiguration()) {
  cache_.initialize(q0);
  robot_.doKinematics(cache_);
}

void MoveTool::DoInitialize(const IiwaState& state) {
  last_time_ = state.get_time();
}

void MoveTool::Update(const IiwaState& state) {
  Isometry3<double> X_WT_d = ComputeDesiredToolInWorld(state);
  Isometry3<double> X_WT = robot_.CalcFramePoseInWorldFrame(cache_, frame_T_);

  const double dt = state.get_time() - last_time_;
  last_time_ = state.get_time();

  if (dt <= 0) return;

  Vector6<double> V_WT_d =
      jaco_planner_.ComputePoseDiffInWorldFrame(X_WT, X_WT_d) / dt;

  VectorX<double> v = jaco_planner_.ComputeDofVelocity(cache_, frame_T_, V_WT_d,
                                                       q_norm_, dt, gain_T_);

  cache_.initialize(cache_.getQ() + v * dt);
  robot_.doKinematics(cache_);
}

void MoveTool::Control(const IiwaState& state,
                       Eigen::Ref<VectorX<double>> q_d) const {
  q_d = cache_.getQ();
}

///////////////////////////////////////////////////////////
MoveToolFollowTraj::MoveToolFollowTraj(
    const std::string& name, const RigidBodyTree<double>* robot,
    const VectorX<double>& q0,
    const manipulation::PiecewiseCartesianTrajectory<double>& traj)
    : MoveTool(name, robot, q0), X_WT_traj_(traj) {}

void MoveToolFollowTraj::Update(const IiwaState& state) {
  // Just servo wrt to force.
  if (is_force_servo()) {
    Vector3<double> f_err_W = f_W_d_ - state.get_ext_wrench().tail<3>();
    for (int i = 0; i < 3; i++) {
      if (std::abs(f_err_W[i]) < f_W_dead_zone_[i]) {
        f_err_W[i] = 0;
      }
    }
    // -= because of reaction force.
    pose_err_I_.translation() -=
        (f_err_W.array() * ki_.tail<3>().array()).matrix();
  }

  for (int i = 0; i < 3; i++) {
    pose_err_I_.translation()[i] =
        clamp(pose_err_I_.translation()[i], -pos_I_range_[i], pos_I_range_[i]);
  }

  // Call the parent's
  MoveTool::Update(state);
}

Isometry3<double> MoveToolFollowTraj::ComputeDesiredToolInWorld(
    const IiwaState& state) const {
  const double interp_t = get_in_state_time(state);
  Isometry3<double> X_WT = X_WT_traj_.get_pose(interp_t);
  X_WT.translation() += pose_err_I_.translation();
  return X_WT;
}

bool MoveToolFollowTraj::IsDone(const IiwaState& state) const {
  const double duration = X_WT_traj_.get_position_trajectory().get_end_time();
  bool ret = get_in_state_time(state) > (duration + 0.5);
  if (is_force_servo()) {
    ret &= has_touched(state);
  }
  return ret;
}

bool MoveToolFollowTraj::has_touched(const IiwaState& state) const {
  DRAKE_DEMAND(is_force_servo());

  Vector3<double> f_err_W = f_W_d_ - state.get_ext_wrench().tail<3>();
  for (int i = 0; i < 3; i++) {
    if (std::abs(f_err_W[i]) < f_W_dead_zone_[i]) {
      f_err_W[i] = 0;
    }
  }
  return f_err_W.norm() < 1;
}

}  // namespace jjz
}  // namespace examples
}  // namespace drake

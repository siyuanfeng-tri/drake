#include "drake/examples/kuka_iiwa_arm/jjz_primitives.h"
#include "drake/common/drake_assert.h"
#include "drake/util/drakeUtil.h"

namespace drake {
namespace jjz {

MotionPrimitive::MotionPrimitive(const std::string& name,
                                 const RigidBodyTree<double>* robot,
                                 MotionPrimitive::Type type)
    : robot_(*robot), name_(name), type_(type) {
  // HACK, these are iiwa specific.
  v_upper_.resize(7);
  v_upper_ << 85, 85, 100, 75, 130, 135, 135;
  v_upper_ = v_upper_ * M_PI / 180.;
  v_lower_ = -v_upper_;
}

///////////////////////////////////////////////////////////
MoveJoint::MoveJoint(const std::string& name,
                     const RigidBodyTree<double>* robot,
                     const VectorX<double>& q0, const VectorX<double>& q1,
                     double duration)
    : MotionPrimitive(name, robot, MotionPrimitive::MOVE_J) {
  DRAKE_DEMAND(q0.size() == q1.size());
  DRAKE_DEMAND(q0.size() == get_robot().get_num_positions());
  std::vector<double> times = {0, duration};
  std::vector<MatrixX<double>> knots = {q0, q1};
  MatrixX<double> zero = MatrixX<double>::Zero(q0.size(), 1);
  //traj_ = PiecewisePolynomial<double>::Cubic(times, knots, zero, zero);
  traj_ = drake::jjz::RetimeTraj(knots,
      VectorX<double>::Zero(q0.size()), VectorX<double>::Zero(q0.size()),
      get_velocity_upper_limit(), get_velocity_lower_limit(),
      VectorX<double>::Constant(q0.size(), 10), VectorX<double>::Constant(q0.size(), -10));
  trajd_ = traj_.derivative();
}

void MoveJoint::DoControl(const IiwaState& state, PrimitiveOutput* output,
                          lcmt_jjz_controller* msg) const {
  const double interp_time = get_in_state_time(state);
  output->q_cmd = traj_.value(interp_time);
  eigenVectorToCArray(output->q_cmd, msg->q_ik);

  VectorX<double> v = trajd_.value(interp_time);
  eigenVectorToCArray(v, msg->v_ik);

  if (get_in_state_time(state) > (traj_.getEndTime() + 0.5)) {
    output->status = PrimitiveOutput::DONE;
  }
}

///////////////////////////////////////////////////////////
MoveTool::MoveTool(const std::string& name, const RigidBodyTree<double>* robot,
                   const RigidBodyFrame<double>* frame_T,
                   const VectorX<double>& q0, MotionPrimitive::Type type)
    : MotionPrimitive(name, robot, type),
      frame_T_(*frame_T),
      cache_(robot->CreateKinematicsCache()),
      jaco_planner_(robot),
      q_norm_(robot->getZeroConfiguration()) {
  cache_.initialize(q0);
  get_robot().doKinematics(cache_);

  jaco_planner_.SetJointSpeedLimit(get_velocity_upper_limit(),
                                   get_velocity_lower_limit());
}

void MoveTool::DoInitialize(const IiwaState& state) {
  last_time_ = state.get_time();
}

void MoveTool::Update(const IiwaState& state, lcmt_jjz_controller* msg) {
  Isometry3<double> X_WT_d = ComputeDesiredToolInWorld(state);
  Isometry3<double> X_WT =
      get_robot().CalcFramePoseInWorldFrame(cache_, frame_T_);

  const double dt = state.get_time() - last_time_;
  last_time_ = state.get_time();

  if (dt <= 0) return;

  Vector6<double> V_WT_d =
      jaco_planner_.ComputePoseDiffInWorldFrame(X_WT, X_WT_d) / dt;

  VectorX<double> v = jaco_planner_.ComputeDofVelocity(cache_, frame_T_, V_WT_d,
                                                       q_norm_, dt, gain_T_);

  // Make debug
  Eigen::Matrix<double, 7, 1> tmp_pose = jjz::pose_to_vec(X_WT_d);
  eigenVectorToCArray(tmp_pose, msg->X_WT_d);

  tmp_pose = jjz::pose_to_vec(X_WT);
  eigenVectorToCArray(tmp_pose, msg->X_WT_ik);

  eigenVectorToCArray(cache_.getQ(), msg->q_ik);
  eigenVectorToCArray(v, msg->v_ik);
  eigenVectorToCArray(V_WT_d, msg->V_WT_d);

  // Integrate ik's fake state.
  cache_.initialize(cache_.getQ() + v * dt);
  get_robot().doKinematics(cache_);
}

void MoveTool::DoControl(const IiwaState& state, PrimitiveOutput* output,
                         lcmt_jjz_controller* msg) const {
  output->q_cmd = cache_.getQ();
  output->X_WT_cmd = get_robot().CalcFramePoseInWorldFrame(cache_, frame_T_);
  eigenVectorToCArray(output->q_cmd, msg->q_ik);
}

///////////////////////////////////////////////////////////
MoveToolStraightUntilTouch::MoveToolStraightUntilTouch(
    const std::string& name, const RigidBodyTree<double>* robot,
    const RigidBodyFrame<double>* frame_T, const VectorX<double>& q0,
    const Vector3<double>& dir, double vel)
    : MoveTool(name, robot, frame_T, q0,
               MotionPrimitive::MOVE_TOOL_STRAIGHT_UNTIL_TOUCH),
      dir_{dir},
      vel_{vel} {
  dir_.normalize();
  X_WT0_ = get_X_WT_ik();
}

Isometry3<double> MoveToolStraightUntilTouch::ComputeDesiredToolInWorld(
    const IiwaState& state) const {
  Isometry3<double> ret = X_WT0_;
  ret.translation() += dir_ * vel_ * get_in_state_time(state);
  return ret;
}

void MoveToolStraightUntilTouch::DoControl(const IiwaState& state,
                                           PrimitiveOutput* output,
                                           lcmt_jjz_controller* msg) const {
  MoveTool::DoControl(state, output, msg);

  if (state.get_ext_wrench().tail<3>().norm() > f_ext_thresh_)
    output->status = PrimitiveOutput::DONE;
}

///////////////////////////////////////////////////////////
HoldPositionAndApplyForce::HoldPositionAndApplyForce(
    const std::string& name, const RigidBodyTree<double>* robot,
    const RigidBodyFrame<double>* frame_T)
    : MotionPrimitive(name, robot, HOLD_J_AND_APPLY_FORCE),
      frame_T_(*frame_T),
      cache_(robot->CreateKinematicsCache()) {}

void HoldPositionAndApplyForce::Update(const IiwaState& state,
                                       lcmt_jjz_controller* msg) {
  cache_.initialize(state.get_q(), state.get_v());
  get_robot().doKinematics(cache_);
}

void HoldPositionAndApplyForce::DoInitialize(const IiwaState& state) {
  q0_ = state.get_q();
  KinematicsCache<double> tmp = get_robot().CreateKinematicsCache();
  tmp.initialize(q0_);
  get_robot().doKinematics(tmp);
  X_WT0_ = get_robot().CalcFramePoseInWorldFrame(tmp, frame_T_);
}

void HoldPositionAndApplyForce::DoControl(const IiwaState& state,
                                          PrimitiveOutput* output,
                                          lcmt_jjz_controller* msg) const {
  const RigidBodyTree<double>& robot = get_robot();
  output->q_cmd = q0_;
  output->X_WT_cmd = X_WT0_;

  // ext_trq = trq_measured - trq_id
  //         = M * qdd + h - J^T * F - (M * qdd + h)
  //         = -J^T * F
  MatrixX<double> J =
      robot.CalcFrameSpatialVelocityJacobianInWorldFrame(cache_, frame_T_);
  output->trq_cmd = -J.transpose() * ext_wrench_d_;

  // Debug
  eigenVectorToCArray(output->q_cmd, msg->q_ik);
}

///////////////////////////////////////////////////////////
MoveToolFollowTraj::MoveToolFollowTraj(
    const std::string& name, const RigidBodyTree<double>* robot,
    const RigidBodyFrame<double>* frame_T, const VectorX<double>& q0,
    const manipulation::PiecewiseCartesianTrajectory<double>& traj)
    : MoveTool(name, robot, frame_T, q0, MOVE_TOOL), X_WT_traj_(traj) {}

Isometry3<double> MoveToolFollowTraj::ComputeDesiredToolInWorld(
    const IiwaState& state) const {
  const double interp_t = get_in_state_time(state);
  return X_WT_traj_.get_pose(interp_t);
}

void MoveToolFollowTraj::DoControl(const IiwaState& state,
                                   PrimitiveOutput* output,
                                   lcmt_jjz_controller* msg) const {
  // Gets the current actual jacobian.
  MatrixX<double> J = get_robot().CalcFrameSpatialVelocityJacobianInWorldFrame(
      state.get_cache(), get_tool_frame());

  // The desired q comes from MoveTool's
  MoveTool::DoControl(state, output, msg);

  // Adds the external force part.
  Vector6<double> wrench = Vector6<double>::Zero();
  wrench[5] = fz_;

  // Compensate for friction.
  const double interp_t = get_in_state_time(state);
  Vector6<double> V_WT = X_WT_traj_.get_velocity(interp_t);
  if (V_WT[2] > 0) {
    wrench[2] = -yaw_mu_ * fz_;
  } else if (V_WT[2] < 0) {
    wrench[2] = yaw_mu_ * fz_;
  }

  if (V_WT[3] > vel_thres_) {
    wrench[3] = -mu_ * fz_;
  } else if (V_WT[3] < -vel_thres_) {
    wrench[3] = mu_ * fz_;
  }

  if (V_WT[4] > vel_thres_) {
    wrench[4] = -mu_ * fz_;
  } else if (V_WT[4] < -vel_thres_) {
    wrench[4] = mu_ * fz_;
  }
  output->trq_cmd = -J.transpose() * wrench;

  // HACK
  for (int i = 0; i < 6; i++) {
    msg->X_WT_int[i] = wrench[i];
  }

  const double duration = X_WT_traj_.get_position_trajectory().get_end_time();
  if (get_in_state_time(state) > (duration + 0.5)) {
    output->status = PrimitiveOutput::DONE;
  }
}

}  // namespace jjz
}  // namespace drake

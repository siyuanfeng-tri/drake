#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/humanoid_plan.h"

#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

template <typename T>
GenericPlan<T>* HumanoidPlan<T>::CloneGenericPlanDerived() const {
  HumanoidPlan<T>* clone = CloneHumanoidPlanDerived();
  clone->zmp_planner_ = this->zmp_planner_;
  clone->dof_trajectory_ = this->dof_trajectory_;
  clone->dofd_trajectory_ = this->dofd_trajectory_;
  clone->dofdd_trajectory_ = this->dofdd_trajectory_;

  return clone;
}

template <typename T>
void HumanoidPlan<T>::InitializeGenericPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {
  std::cout << "HPlan init.\n";

  // Knots are constant, the second time doesn't matter.
  std::vector<T> times = {robot_status.time(), robot_status.time() + 1};

  // Makes a zmp planner that stays still.
  Vector4<double> xcom;
  xcom << robot_status.com().head<2>(), robot_status.comd().head<2>();
  MatrixX<double> com_d = robot_status.com().head<2>();
  PiecewisePolynomial<T> zmp_d =
      PiecewisePolynomial<T>::ZeroOrderHold(times, {com_d, com_d});
  this->UpdateZmpPlan(zmp_d, xcom, 1);

  // Sets a dof traj to track current.
  MatrixX<double> q_d = robot_status.position();
  this->UpdateDofTrajectory(
      PiecewisePolynomial<T>::ZeroOrderHold(times, {q_d, q_d}));

  // Sets contact state to double support always.
  ContactState double_support;
  double_support.insert(alias_groups.get_body("left_foot"));
  double_support.insert(alias_groups.get_body("right_foot"));
  this->UpdateContactState(double_support);

  // Sets body traj for pelvis and torso.
  const std::vector<std::string> tracked_body_names = {"pelvis", "torso"};
  for (const auto& name : tracked_body_names) {
    const RigidBody<T>* body = alias_groups.get_body(name);
    Isometry3<T> body_pose = robot_status.robot().CalcBodyPoseInWorldFrame(
        robot_status.cache(), *body);
    this->set_body_trajectory(
        body, PiecewiseCartesianTrajectory<T>(
            times, {body_pose, body_pose}, true));
  }

  // Calls derived classes' initialization.
  InitializeHumanoidPlanDerived(robot_status, paramset, alias_groups);
}

template <typename T>
void HumanoidPlan<T>::UpdateQpInputGenericPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
    QpInput* qp_input) const {
  T interp_time =
      robot_status.time();  // this->get_interp_time(robot_status.time());

  // Generates desired acceleration for tracked bodies
  for (const auto& body_motion_pair : this->get_body_trajectories()) {
    const RigidBody<T>* body = body_motion_pair.first;
    const PiecewiseCartesianTrajectory<T>& traj = body_motion_pair.second;

    Vector6<T> kp, kd;
    paramset.LookupDesiredBodyMotionGains(*body, &kp, &kd);

    CartesianSetpoint<T> tracker(traj.get_pose(interp_time),
                                 traj.get_velocity(interp_time),
                                 traj.get_acceleration(interp_time), kp, kd);

    Isometry3<T> pose = robot_status.robot().CalcBodyPoseInWorldFrame(
        robot_status.cache(), *body);
    Vector6<T> velocity =
        robot_status.robot().CalcBodySpatialVelocityInWorldFrame(
            robot_status.cache(), *body);
    qp_input->mutable_desired_body_motions()
        .at(body->get_name())
        .mutable_values() = tracker.ComputeTargetAcceleration(pose, velocity);
  }

  // Generates desired acceleration for all dof.
  VectorX<T> kp, kd;
  paramset.LookupDesiredDofMotionGains(&kp, &kd);
  VectorX<T> v_d = dofd_trajectory_.value(interp_time);
  VectorX<T> vd_d = dofdd_trajectory_.value(interp_time);
  if (interp_time > dofd_trajectory_.getEndTime()) {
    v_d.setZero();
    vd_d.setZero();
  }
  VectorSetpoint<T> tracker(dof_trajectory_.value(interp_time),
                            v_d, vd_d, kp, kd);
  qp_input->mutable_desired_dof_motions().mutable_values() =
      tracker.ComputeTargetAcceleration(robot_status.position(),
                                        robot_status.velocity());

  // Generates CoM acceleration.
  Vector4<T> xcom;
  xcom << robot_status.com().head<2>(), robot_status.comd().head<2>();
  Vector2<T> comdd_d = zmp_planner_.ComputeOptimalCoMdd(interp_time, xcom);
  qp_input->mutable_desired_centroidal_momentum_dot()
      .mutable_values()
      .segment<2>(3) = robot_status.robot().getMass() * comdd_d;
}

template class HumanoidPlan<double>;

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

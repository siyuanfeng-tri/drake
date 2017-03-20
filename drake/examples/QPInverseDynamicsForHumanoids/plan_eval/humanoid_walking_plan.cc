#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/humanoid_walking_plan.h"

#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

template <typename T>
HumanoidPlan<T>* HumanoidWalkingPlan<T>::CloneHumanoidPlanDerived() const {
  HumanoidWalkingPlan<T>* clone = new HumanoidWalkingPlan<T>();
  clone->footsteps_ = this->footsteps_;
  clone->next_event_ = this->next_event_;
  clone->state_ = this->state_;

  return clone;
}

template <typename T>
void HumanoidWalkingPlan<T>::InitializeHumanoidPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {
  footsteps_.reset();
  state_ = HOLD;
  next_event_.time = std::numeric_limits<T>::infinity();
  next_event_.action_func = &HumanoidWalkingPlan<T>::UpdatePlanOnTouchdown;
}

template <typename T>
void HumanoidWalkingPlan<T>::ExecutePlanGenericPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {
  if (next_event_.is_triggered(robot_status))
    next_event_.action_func(this, robot_status, paramset, alias_groups);
}

template <typename T>
void HumanoidWalkingPlan<T>::HandlePlanMessageGenericPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
    const void* message_bytes, int message_length) {
  // HACK
  double dx = 0.1;
  double dyaw = 0.2;
  Footstep<T> fs;
  fs.side = Side::LEFT;
  fs.pose = robot_status.foot(fs.side.underlying()).pose();
  fs.pose.translation()[0] += dx;
  fs.pose.linear() = AngleAxis<T>(dyaw, Vector3<T>::UnitZ()).toRotationMatrix();
  footsteps_.push_back(fs);

  fs.side = Side::RIGHT;
  fs.pose = robot_status.foot(fs.side.underlying()).pose();
  fs.pose.translation()[0] += 2 * dx;
  fs.pose.linear() =
      AngleAxis<T>(2 * dyaw, Vector3<T>::UnitZ()).toRotationMatrix();
  footsteps_.push_back(fs);

  fs.side = Side::LEFT;
  fs.pose = robot_status.foot(fs.side.underlying()).pose();
  fs.pose.translation()[0] += 3 * dx;
  fs.pose.linear() =
      AngleAxis<T>(3 * dyaw, Vector3<T>::UnitZ()).toRotationMatrix();
  footsteps_.push_back(fs);

  fs.side = Side::RIGHT;
  fs.pose = robot_status.foot(fs.side.underlying()).pose();
  fs.pose.translation()[0] += 3 * dx;
  fs.pose.linear() =
      AngleAxis<T>(4 * dyaw, Vector3<T>::UnitZ()).toRotationMatrix();
  footsteps_.push_back(fs);

  // plan com
  next_event_.time = robot_status.time();
  next_event_.action_func = &HumanoidWalkingPlan<T>::UpdatePlanOnTouchdown;
}

template <typename T>
void HumanoidWalkingPlan<T>::UpdatePlanOnTouchdown(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {
  std::cout << "TD time: " << robot_status.time() << std::endl;
  const ContactState& current_contact_state = this->get_contact_state();
  const RigidBody<T>* left_foot = alias_groups.get_body("left_foot");
  const RigidBody<T>* right_foot = alias_groups.get_body("right_foot");

  // Calls take a step only when we actually started walking (transitioning from
  // single support).
  if (current_contact_state.find(left_foot) == current_contact_state.end() ||
      current_contact_state.find(right_foot) == current_contact_state.end()) {
    footsteps_.take_a_step();
  }

  // Computes the desired zmp trajectory.
  T liftoff_time;
  PiecewisePolynomial<T> zmp_traj =
      GenerateDesiredZmpTrajectory(current_contact_state, robot_status,
                                   paramset, alias_groups, &liftoff_time);
  Vector4<T> x_com0;
  x_com0 << robot_status.com().template head<2>(),
      robot_status.comd().template head<2>();
  this->UpdateZmpPlan(zmp_traj, x_com0, pelvis_height_);

  // Changes contact state to double support.
  ContactState double_support;
  double_support.insert(left_foot);
  double_support.insert(right_foot);
  this->UpdateContactState(double_support);

  // Removes the left / right foot trakcing if possible.
  this->remove_body_trajectory(left_foot);
  this->remove_body_trajectory(right_foot);

  // Sets the next event.
  next_event_.time = liftoff_time;
  next_event_.action_func = &HumanoidWalkingPlan<T>::UpdatePlanOnLiftoff;

  // Updates the state depending if there are more steps to take.
  if (footsteps_.empty()) {
    state_ = HOLD;
  } else {
    state_ = WEIGHT_SHIFT;
  }
}

template <typename T>
void HumanoidWalkingPlan<T>::UpdatePlanOnLiftoff(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {
  std::cout << "LO time: " << robot_status.time() << std::endl;

  // Generates swing foot trajectory.
  Isometry3<T> touchdown_foot_pose = footsteps_.get_current_footstep().pose;
  auto swing = GenerateDesiredSwingFootTrajectory(
      robot_status, paramset, alias_groups, touchdown_foot_pose);
  this->set_body_trajectory(swing.first, swing.second);

  // Generates pelvis and torso trajectories.
  const RigidBody<T>* stance_foot = footsteps_.get_current_stance_foot(alias_groups);
  const RigidBodyTree<T>& robot = robot_status.robot();
  const KinematicsCache<T>& cache = robot_status.cache();
  Isometry3<T> stance_foot_pose =
      robot.CalcBodyPoseInWorldFrame(cache, *stance_foot);

  auto pelvis_traj_pair = GenerateDesiredSwingPelvisTrajectory(
      robot_status, paramset, alias_groups, stance_foot_pose,
      touchdown_foot_pose);
  this->set_body_trajectory(pelvis_traj_pair.first, pelvis_traj_pair.second);

  auto torso_traj_pair = GenerateDesiredSwingTorsoTrajectory(
      robot_status, paramset, alias_groups, stance_foot_pose,
      touchdown_foot_pose);
  this->set_body_trajectory(torso_traj_pair.first, torso_traj_pair.second);

  // Updates contact state.
  ContactState single_support;
  single_support.insert(footsteps_.get_current_stance_foot(alias_groups));
  this->UpdateContactState(single_support);

  // Sets the next event.
  next_event_.time = robot_status.time() + ss_duration_;
  next_event_.action_func = &HumanoidWalkingPlan<T>::UpdatePlanOnTouchdown;

  state_ = SWING;
}

template <typename T>
std::pair<const RigidBody<T>*, PiecewiseCartesianTrajectory<T>>
HumanoidWalkingPlan<T>::GenerateDesiredSwingPelvisTrajectory(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
    const Isometry3<T>& stance_foot_pose,
    const Isometry3<T>& touchdown_foot_pose) const {
  const T t0 = robot_status.time();
  const RigidBody<T>* pelvis = alias_groups.get_body("pelvis");
  Isometry3<T> pelvis_pose0 = this->get_body_trajectory(pelvis).get_pose(t0);
  Isometry3<T> pelvis_pose1 = Isometry3<T>::Identity();
  T end_pelvis_yaw = ComputePelvisYaw(stance_foot_pose, touchdown_foot_pose);
  T end_pelvis_z = ComputePelvisHeight(stance_foot_pose, touchdown_foot_pose);

  pelvis_pose1.translation()[2] = end_pelvis_z;
  pelvis_pose1.linear() =
      AngleAxis<T>(end_pelvis_yaw, Vector3<T>::UnitZ()).toRotationMatrix();

  const std::vector<T> times = {t0, t0 + ss_duration_};
  const std::vector<Isometry3<T>> poses = {pelvis_pose0, pelvis_pose1};
  PiecewiseCartesianTrajectory<T> traj =
      PiecewiseCartesianTrajectory<T>::MakeCubicLinearWithZeroEndVelocity(
          times, poses);

  return std::pair<const RigidBody<T>*, PiecewiseCartesianTrajectory<T>>(
      pelvis, traj);
}

template <typename T>
std::pair<const RigidBody<T>*, PiecewiseCartesianTrajectory<T>>
HumanoidWalkingPlan<T>::GenerateDesiredSwingTorsoTrajectory(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
    const Isometry3<T>& stance_foot_pose,
    const Isometry3<T>& touchdown_foot_pose) const {
  const T t0 = robot_status.time();
  const RigidBody<T>* torso = alias_groups.get_body("torso");
  Isometry3<T> torso_pose0 = this->get_body_trajectory(torso).get_pose(t0);
  Isometry3<T> torso_pose1 = Isometry3<T>::Identity();
  T end_torso_yaw = ComputePelvisYaw(stance_foot_pose, touchdown_foot_pose);

  torso_pose1.linear() =
      AngleAxis<T>(end_torso_yaw, Vector3<T>::UnitZ()).toRotationMatrix();

  const std::vector<T> times = {t0, t0 + ss_duration_};
  const std::vector<Isometry3<T>> poses = {torso_pose0, torso_pose1};
  PiecewiseCartesianTrajectory<T> traj =
      PiecewiseCartesianTrajectory<T>::MakeCubicLinearWithZeroEndVelocity(
          times, poses);
  return std::pair<const RigidBody<T>*, PiecewiseCartesianTrajectory<T>>(
      torso, traj);
}

template <typename T>
std::pair<const RigidBody<T>*, PiecewiseCartesianTrajectory<T>>
HumanoidWalkingPlan<T>::GenerateDesiredSwingFootTrajectory(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
    const Isometry3<T>& touchdown_foot_pose) const {
  const T t0 = robot_status.time();
  const RigidBodyTree<T>& robot = robot_status.robot();
  const KinematicsCache<T>& cache = robot_status.cache();

  const RigidBody<T>* swing_foot = footsteps_.get_current_swing_foot(alias_groups);
  DRAKE_DEMAND(swing_foot != nullptr);

  Isometry3<T> current_pose =
      robot.CalcBodyPoseInWorldFrame(cache, *swing_foot);

  std::vector<T> times = {t0, t0 + ss_duration_};
  std::vector<MatrixX<T>> xy_knots = {
      current_pose.translation().template head<2>(),
      touchdown_foot_pose.translation().template head<2>()};
  std::vector<MatrixX<T>> xy_knots_dot(2, MatrixX<T>::Zero(2, 1));
  PiecewisePolynomial<T> xy_line =
      PiecewisePolynomial<T>::Cubic(times, xy_knots, xy_knots_dot);

  eigen_aligned_std_vector<Matrix3<T>> rot_knots = {
      current_pose.linear(), touchdown_foot_pose.linear()};
  PiecewiseQuaternionSlerp<T> rot_line =
      PiecewiseQuaternionSlerp<T>(times, rot_knots);

  Vector2<T> mid_xy = xy_line.value(t0 + ss_duration_ / 2.);
  Quaternion<T> mid_rot = rot_line.orientation(t0 + ss_duration_ / 2.);

  Isometry3<T> mid_pose;
  mid_pose.translation().template head<2>() = mid_xy;
  mid_pose.translation()[2] = std::max(current_pose.translation()[2],
                                       touchdown_foot_pose.translation()[2]) +
                              0.05;
  mid_pose.linear() = mid_rot.toRotationMatrix();

  times = {t0, t0 + ss_duration_ / 2., t0 + ss_duration_};
  std::vector<Isometry3<T>> pose_knots = {current_pose, mid_pose,
                                          touchdown_foot_pose};
  PiecewiseCartesianTrajectory<T> traj =
      PiecewiseCartesianTrajectory<T>::MakeCubicLinearWithZeroEndVelocity(
          times, pose_knots);
  return std::pair<const RigidBody<T>*, PiecewiseCartesianTrajectory<T>>(
      swing_foot, traj);
}

template <typename T>
Vector2<T> HumanoidWalkingPlan<T>::ComputeMidFootXyFromFootPose(
    const Isometry3<T>& foot_pose,
    const param_parsers::ParamSet& paramset) const {
  Isometry3<T> offset = Isometry3<T>::Identity();
  offset.translation()[0] = 0.075;
  Isometry3<T> mid_foot = foot_pose * offset;
  return mid_foot.translation().template head<2>();
}

template <typename T>
PiecewisePolynomial<T> HumanoidWalkingPlan<T>::GenerateDesiredZmpTrajectory(
    const ContactState& current_contact_state,
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
    T* liftoff_time) const {
  const RigidBodyTree<T>& robot = robot_status.robot();
  const KinematicsCache<T>& cache = robot_status.cache();

  const RigidBody<T>* next_stance_foot =
      footsteps_.get_current_stance_foot(alias_groups);
  const RigidBody<T>* left_foot = alias_groups.get_body("left_foot");
  const RigidBody<T>* right_foot = alias_groups.get_body("right_foot");
  Vector2<T> left_foot_xy = ComputeMidFootXyFromFootPose(
      robot.CalcBodyPoseInWorldFrame(cache, *left_foot), paramset);
  Vector2<T> right_foot_xy = ComputeMidFootXyFromFootPose(
      robot.CalcBodyPoseInWorldFrame(cache, *right_foot), paramset);

  std::vector<MatrixX<T>> zmp_knots;
  std::vector<T> times;

  // The first desired zmp knot point is under the current supporting foot.
  if (current_contact_state.find(left_foot) != current_contact_state.end() &&
      current_contact_state.find(right_foot) != current_contact_state.end()) {
    zmp_knots.push_back((left_foot_xy + right_foot_xy) / 2.);
    times.push_back(robot_status.time());
    zmp_knots.push_back((left_foot_xy + right_foot_xy) / 2.);
    times.push_back(times.back() + extra_time_);
  } else if (current_contact_state.find(left_foot) !=
             current_contact_state.end()) {
    zmp_knots.push_back(left_foot_xy);
    times.push_back(robot_status.time());
  } else if (current_contact_state.find(right_foot) !=
             current_contact_state.end()) {
    zmp_knots.push_back(right_foot_xy);
    times.push_back(robot_status.time());
  } else {
    DRAKE_DEMAND(false);
  }

  // The second desired zmp knot point is under the supporting foot for the
  // next swing phase. If there are no more steps, shifts back to the middle.
  if (footsteps_.empty()) {
    zmp_knots.push_back((left_foot_xy + right_foot_xy) / 2.);
    times.push_back(times.back() + ds_duration_);
    zmp_knots.push_back((left_foot_xy + right_foot_xy) / 2.);
    times.push_back(times.back() + extra_time_);

    *liftoff_time = std::numeric_limits<T>::infinity();
  } else {
    DRAKE_DEMAND(next_stance_foot);
    zmp_knots.push_back(ComputeMidFootXyFromFootPose(
        robot.CalcBodyPoseInWorldFrame(cache, *next_stance_foot), paramset));
    times.push_back(times.back() + ds_duration_);

    *liftoff_time = times.back();
  }

  // Then just enqueue N extra furture supporting foot locations for the
  // desired zmp.
  int i = 0;
  const int num_look_ahead = 3;
  for (const auto& footstep : footsteps_.get_footsteps()) {
    if (i > num_look_ahead) break;
    zmp_knots.push_back(zmp_knots.back());
    times.push_back(times.back() + ss_duration_);

    zmp_knots.push_back(ComputeMidFootXyFromFootPose(footstep.pose, paramset));
    times.push_back(times.back() + ds_duration_);
    i++;
  }

  return PiecewisePolynomial<T>::Pchip(times, zmp_knots, true);
}

template <typename T>
T HumanoidWalkingPlan<T>::ComputePelvisYaw(
    const Isometry3<T>& stance_foot_pose,
    const Isometry3<T>& touchdown_foot_pose) const {
  T target_yaw = 0;
  if (footsteps_.is_last_step()) {
    T swing_yaw = math::rotmat2rpy(touchdown_foot_pose.linear())[2];
    T stance_yaw = math::rotmat2rpy(stance_foot_pose.linear())[2];
    target_yaw = average_angles(swing_yaw, stance_yaw);
  } else {
    target_yaw = math::rotmat2rpy(touchdown_foot_pose.linear())[2];
  }
  return target_yaw;
}

template <typename T>
T HumanoidWalkingPlan<T>::ComputePelvisHeight(
    const Isometry3<T>& stance_foot_pose,
    const Isometry3<T>& touchdown_foot_pose) const {
  T target_z = 0;
  if (footsteps_.is_last_step()) {
    T swing_z = touchdown_foot_pose.translation()[2];
    T stance_z = stance_foot_pose.translation()[2];
    target_z = (swing_z + stance_z) / 2.;
  } else {
    target_z = touchdown_foot_pose.translation()[2];
  }
  return target_z + pelvis_height_;
}

template class HumanoidWalkingPlan<double>;

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

#include <vector>

#include "drake/examples/QPInverseDynamicsForHumanoids/system/manip_plan_eval_system.h"
#include "drake/math/quaternion.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

ManipPlanEvalSystem::ManipPlanEvalSystem(const RigidBodyTree<double>& robot) :
    robot_(robot) {
  input_port_index_humanoid_status_ =
    DeclareAbstractInputPort(systems::kInheritedSampling).get_index();
  input_port_index_manip_plan_ =
    DeclareAbstractInputPort(systems::kInheritedSampling).get_index();
  output_port_index_qp_input_ =
    DeclareAbstractOutputPort(systems::kInheritedSampling).get_index();

  set_name("manip_plan_eval");

  // TODO(siyuan.feng): Move these to some param / config file eventually.
  // Set up gains.
  int dim = robot_.get_num_positions();
  Kp_com_ = Vector3<double>::Constant(40);
  Kd_com_ = Vector3<double>::Constant(12);
  Kp_pelvis_ = Vector6<double>::Constant(20);
  Kd_pelvis_ = Vector6<double>::Constant(8);
  Kp_torso_ = Vector6<double>::Constant(20);
  Kd_torso_ = Vector6<double>::Constant(8);
  Kp_joints_ = VectorX<double>::Constant(dim, 20);
  Kd_joints_ = VectorX<double>::Constant(dim, 8);
  // Don't do feedback on pelvis in the generalized coordinates.
  Kp_joints_.head<6>().setZero();
  Kd_joints_.head<6>().setZero();
}

void KeyframeToState(const bot_core::robot_state_t &keyframe,
                     Eigen::VectorXd &q, Eigen::VectorXd &v) {
  q.resize(6 + keyframe.joint_position.size());
  v.resize(6 + keyframe.joint_velocity.size());
  // assuming floating base
  q[0] = keyframe.pose.translation.x;
  q[1] = keyframe.pose.translation.y;
  q[2] = keyframe.pose.translation.z;

  Eigen::Matrix<double, 4, 1> quat;
  quat[0] = keyframe.pose.rotation.w;
  quat[1] = keyframe.pose.rotation.x;
  quat[2] = keyframe.pose.rotation.y;
  quat[3] = keyframe.pose.rotation.z;
  q.segment<3>(3) = math::quat2rpy(quat);

  v[0] = keyframe.twist.linear_velocity.x;
  v[1] = keyframe.twist.linear_velocity.y;
  v[2] = keyframe.twist.linear_velocity.z;
  v[3] = keyframe.twist.angular_velocity.x;
  v[4] = keyframe.twist.angular_velocity.y;
  v[5] = keyframe.twist.angular_velocity.z;

  for (size_t i = 0; i < keyframe.joint_position.size(); i++) {
    q[i + 6] = keyframe.joint_position[i];
  }
  for (size_t i = 0; i < keyframe.joint_velocity.size(); i++) {
    v[i + 6] = keyframe.joint_velocity[i];
  }
}


void ManipPlanEvalSystem::HandleNewManipPlan(const robotlocomotion::robot_plan_t& msg) const {
  size_t num_T = msg.plan.size();

  std::vector<double> Ts(num_T);
  std::vector<Vector2<double>> com_d(num_T);
  std::vector<VectorX<double>> q_d(num_T);  // t steps by n

  std::vector<const RigidBody<double>*> bodies;
  bodies.push_back(robot_.FindBody("pelvis"));
  size_t num_bodies = bodies.size();

  std::vector<std::vector<Eigen::Matrix<double, 7, 1>>> x_d(num_bodies);
  std::vector<std::vector<Eigen::Matrix<double, 7, 1>>> xd_d(num_bodies);
  for (size_t i = 0; i < num_bodies; i++) {
    x_d[i].resize(num_T);
    xd_d[i].resize(num_T);
  }

  // generate the current tracked body poses from the estimated robot state
  // maybe useful eventually
  /*
  KinematicsCache<double> cache_est = robot_.doKinematics(est_rs.q, est_rs.qd);
  std::vector<Eigen::Matrix<double, 7, 1>> x_est(num_bodies);
  for (size_t b = 0; b < num_bodies; b++) {
    int id = robot_.findLink(body_names[b])->body_index;
    Isometry3<double> pose = robot_.relativeTransform(cache_est, 0, id);
    x_est[b].segment<3>(0) = pose.translation();
    x_est[b].segment<4>(3) = rotmat2quat(pose.linear());
    std::cout << "cur pose " << body_names[b] << " " << x_est[b].segment<4>(3).transpose() << std::endl;
  }
  */

  VectorX<double> q, v;
  // generate q_traj first w. cubic spline, which gives velocities.
  for (size_t t = 0; t < num_T; t++) {
    const bot_core::robot_state_t &keyframe = msg.plan[t];
    KeyframeToState(keyframe, q, v);
    Ts[t] = static_cast<double>(keyframe.utime) / 1e6;
    q_d[t] = q;
  }

  // make q, v splines, make sure to set t0 and t1 vel to zero
  VectorX<double> zero = VectorX<double>::Zero(q_d[0].size());
  q_trajs_ = GenerateCubicSpline(Ts, q_d, zero, zero);
  auto v_trajs = q_trajs_.derivative();

  // go through Ts again to make trajs for all the cartesian stuff
  for (size_t t = 0; t < num_T; t++) {
    q_ = q_trajs_.value(Ts[t]);
    v_ = v_trajs.value(Ts[t]);

    KinematicsCache<double> cache_plan = robot_.doKinematics(q_, v_);

    for (size_t b = 0; b < num_bodies; b++) {
      const RigidBody *body = robot_.findLink(body_names[b]).get();
      int id = body->body_index;
      Isometry3<double> pose = robot_.relativeTransform(cache_plan, 0, id);
      x_d[b][t].segment<3>(0) = pose.translation();
      x_d[b][t].segment<4>(3) = rotmat2quat(pose.linear());

      Vector6<double> xd = robot_.CalcTwistInWorldAlignedBody(cache_plan, *body);
      xd_d[b][t].segment<3>(0) = xd.segment<3>(3);
      // http://www.euclideanspace.com/physics/kinematics/angularvelocity/QuaternionDifferentiation2.pdf
      Vector4<double> W(0, xd[0], xd[1], xd[2]);
      xd_d[b][t].segment<4>(3) = 0.5 * quatProduct(W, x_d[b][t].segment<4>(3));
    }

    // get com
    com_d[t] = robot_.centerOfMass(cache_plan).segment<2>(0);
  }


  // make zmp traj, since we are manip, com ~= zmp, zmp is created with pchip
  Vector2<double> zero2(Vector2<double>::Zero());
  zmp_traj_ = GeneratePCHIPSpline(Ts, com_d, zero2, zero2);
  // TODO: make traj for s1, and com
  // drake/examples/ZMP/LinearInvertedPendulum.m

  Vector4<double> x0(Vector4<double>::Zero());
  x0.head(2) = com_d[0];
  zmp_planner_.Plan(zmp_traj_, x0, p_zmp_height_);

  // make body motion splines
  body_motions_.resize(num_bodies);
  for (size_t b = 0; b < num_bodies; b++) {
    body_motions_[b] = MakeDefaultBodyMotionData(num_T);

    body_motions_[b].body_or_frame_id =
        robot_.findLink(body_names[b])->body_index;
    body_motions_[b].trajectory =
        GenerateCubicCartesianSpline(Ts, x_d[b], xd_d[b]);
    if (body_names[b].compare(robot_.getBodyOrFrameName(rpc_.pelvis_id)) == 0)
      body_motions_[b].control_pose_when_in_contact.resize(num_T, true);
  }

  // make support, dummy here since we are always in double support
  support_state_ = MakeDefaultSupportState(ContactState::DS());

  // constrained DOFs
  // TODO: this is not true for dragging hands around
  constrained_dofs_.clear();
  for (auto it = rpc_.position_indices.arms.begin(); it != rpc_.position_indices.arms.end(); it++) {
    const std::vector<int> &indices = it->second;
    for (size_t i = 0; i < indices.size(); i++)
      constrained_dofs_.push_back(indices[i]);
  }
  // neck
  for (size_t i = 0; i < rpc_.position_indices.neck.size(); i++)
    constrained_dofs_.push_back(rpc_.position_indices.neck[i]);
  // back
  constrained_dofs_.push_back(rpc_.position_indices.back_bkz);
  //constrained_dofs_.push_back(rpc_.position_indices.back_bky);

  std::cout << "committed robot plan proced\n";
}

void ManipPlanEvalSystem::EvalOutput(const Context<double>& context,
                                     SystemOutput<double>* output) const {
  // Inputs:
  const HumanoidStatus* robot_status = EvalInputValue<HumanoidStatus>(
      context, input_port_index_humanoid_status_);

  const robotlocomotion::robot_plan_t* plan = EvalInputValue<robotlocomotion::robot_plan_t>(
      context, input_port_index_manip_plan_);

  HandleNewManipPlan(*plan);

  // Output:
  lcmt_qp_input& msg = output->GetMutableData(output_port_index_qp_input_)
    ->GetMutableValue<lcmt_qp_input>();

  Vector3<double> com_err = desired_com_ - robot_status->com();
  Vector3<double> comd_err = -robot_status->comd();

  // Update desired accelerations.
  QPInput qp_input = MakeExampleQPInput(*robot_status);
  qp_input.mutable_desired_centroidal_momentum_dot()
    .mutable_values().tail<3>() = robot_.getMass() *
    (Kp_com_.array() * com_err.array() +
     Kd_com_.array() * comd_err.array()).matrix();

  qp_input.mutable_desired_dof_motions().mutable_values() =
    joint_PDff_.ComputeTargetAcceleration(robot_status->position(),
        robot_status->velocity());
  qp_input.mutable_desired_body_motions().at("pelvis").mutable_values() =
    pelvis_PDff_.ComputeTargetAcceleration(
        robot_status->pelvis().pose(), robot_status->pelvis().velocity());
  qp_input.mutable_desired_body_motions().at("torso").mutable_values() =
    torso_PDff_.ComputeTargetAcceleration(robot_status->torso().pose(),
        robot_status->torso().velocity());

  // Encode and send.
  EncodeQPInput(qp_input, &msg);
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

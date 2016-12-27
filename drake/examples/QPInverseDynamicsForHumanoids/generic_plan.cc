#include "drake/examples/QPInverseDynamicsForHumanoids/generic_plan.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

GenericHumanoidPlan::GenericHumanoidPlan(const RigidBodyTree<double>& robot)
    : robot_(&robot),
    left_foot_(robot_->FindBody("leftFoot")),
    right_foot_(robot_->FindBody("rightFoot")),
    pelvis_(robot_->FindBody("pelvis")),
    torso_(robot_->FindBody("torso")) {

  // Set up tracking for various body parts.
  DesiredBodyMotion body_motion(*pelvis_);
  body_motion.mutable_weights().head<3>() = Vector3<double>(1, 1, 1);
  body_motion.mutable_weights()[5] = 5;
  body_motion.SetConstraintType({0, 1, 2, 5}, ConstraintType::Soft);
  all_trackable_bodies_.emplace(pelvis_, body_motion);

  body_motion = DesiredBodyMotion(*torso_);
  body_motion.mutable_weights().head<3>() = Vector3<double>(1, 1, 1);
  body_motion.SetConstraintType({0, 1, 2}, ConstraintType::Soft);
  all_trackable_bodies_.emplace(torso_, body_motion);

  body_motion = DesiredBodyMotion(*left_foot_);
  body_motion.mutable_weights() << 1, 1, 1, 1, 1, 1;
  body_motion.SetConstraintType({0, 1, 2, 3, 4, 5}, ConstraintType::Soft);
  all_trackable_bodies_.emplace(left_foot_, body_motion);

  body_motion = DesiredBodyMotion(*right_foot_);
  body_motion.mutable_weights() << 1, 1, 1, 1, 1, 1;
  body_motion.SetConstraintType({0, 1, 2, 3, 4, 5}, ConstraintType::Soft);
  all_trackable_bodies_.emplace(right_foot_, body_motion);

  // Set up contacts
  all_contactable_bodies_.emplace(left_foot_, MakeDefaultFootContactInformation(left_foot_));
  all_contactable_bodies_.emplace(right_foot_, MakeDefaultFootContactInformation(right_foot_));

  Kp_pelvis_ = Vector6<double>::Constant(20);
  Kd_pelvis_ = Vector6<double>::Constant(8);
  Kp_foot_ = Vector6<double>::Constant(50);
  Kd_foot_ = Vector6<double>::Constant(14);

  Kp_joints_ = VectorX<double>::Constant(robot.get_num_velocities(), 20);
  Kd_joints_ = VectorX<double>::Constant(robot.get_num_velocities(), 8);

  Kp_joints_.head<6>().setZero();
  Kd_joints_.head<6>().setZero();
}

void GenericHumanoidPlan::InitializeQPInput(const HumanoidStatus& robot_status, QPInput* qp_input) const {
  // Make a clean copy.
  *qp_input = QPInput(*robot_);

  int dim = robot_->get_num_velocities();

  // Set up a PD tracking law for center of mass.
  qp_input->mutable_desired_centroidal_momentum_dot().mutable_weights().tail<3>() =
      Vector3<double>::Constant(1e1);
  // Wipe out the weights for the angular part.
  qp_input->mutable_desired_centroidal_momentum_dot()
      .mutable_weights()
      .head<3>()
      .setZero();
  qp_input->mutable_desired_centroidal_momentum_dot().SetConstraintType(
      {3, 4}, ConstraintType::Soft);

  // Minimize acceleration in the generalized coordinates.
  qp_input->mutable_desired_dof_motions().mutable_weights() =
      VectorX<double>::Constant(dim, 1e-2);
  qp_input->mutable_desired_dof_motions().SetAllConstraintType(
      ConstraintType::Soft);

  // Set tracked bodies
  for (const RigidBody<double>* body : tracked_bodies_) {
    std::map<const RigidBody<double>*, DesiredBodyMotion>::const_iterator it = all_trackable_bodies_.find(body);
    // Doesn't have params, need to make default
    if (it == all_trackable_bodies_.end()) {
      DesiredBodyMotion bodydd_d(*body);
      bodydd_d.mutable_weights() << 1, 1, 1, 1, 1, 1;
      bodydd_d.SetConstraintType({0, 1, 2, 3, 4, 5}, ConstraintType::Soft);
      qp_input->mutable_desired_body_motions().emplace(bodydd_d.body_name(), bodydd_d);
    } else {
      const DesiredBodyMotion& bodydd_d = it->second;
      qp_input->mutable_desired_body_motions().emplace(bodydd_d.body_name(), bodydd_d);
    }
  }

  // Weights are set arbitrarily by the control designer, these typically
  // require tuning.
  qp_input->mutable_w_basis_reg() = 1e-6;

  // Set arm and neck joint accelerations to hard constraints.
  for (const std::string& joint_name : robot_status.arm_joint_names()) {
    int idx = robot_status.name_to_position_index().at(joint_name);
    qp_input->mutable_desired_dof_motions().mutable_weight(idx) = -1;
    qp_input->mutable_desired_dof_motions().mutable_constraint_type(idx) =
        ConstraintType::Hard;
  }

  for (const std::string& joint_name : robot_status.neck_joint_names()) {
    int idx = robot_status.name_to_position_index().at(joint_name);
    qp_input->mutable_desired_dof_motions().mutable_weight(idx) = -1;
    qp_input->mutable_desired_dof_motions().mutable_constraint_type(idx) =
        ConstraintType::Hard;
  }
}

ContactInformation GenericHumanoidPlan::MakeDefaultFootContactInformation(const RigidBody<double>* body) const {
  ContactInformation contact(*body);
  contact.mutable_contact_points().resize(3, 4);
  contact.mutable_contact_points().col(0) =
      Vector3<double>(0.2, 0.05, -0.09);
  contact.mutable_contact_points().col(1) =
      Vector3<double>(0.2, -0.05, -0.09);
  contact.mutable_contact_points().col(2) =
      Vector3<double>(-0.05, -0.05, -0.09);
  contact.mutable_contact_points().col(3) =
      Vector3<double>(-0.05, 0.05, -0.09);
  contact.mutable_acceleration_constraint_type() =
      ConstraintType::Soft;
  contact.mutable_weight() = 1e5;
  contact.mutable_Kd() = 8;

  return contact;
}

QPInput GenericHumanoidPlan::CalcQPInput(const HumanoidStatus& rs) const {
  QPInput qp_input(*robot_);

  InitializeQPInput(rs, &qp_input);

  double plan_time = rs.time() - interp_t0_;

  // CoM feedback
  Vector4<double> xcom;
  xcom << rs.com().head<2>(), rs.comd().head<2>();

  Vector2<double> comdd_d = zmp_planner_.ComputeOptimalCoMdd(plan_time, xcom);
  qp_input.mutable_desired_centroidal_momentum_dot()
        .mutable_values().segment<2>(3) = robot_->getMass() * comdd_d;

  // Body motion feedback
  for (auto& body_motion_pair : qp_input.mutable_desired_body_motions()) {
    DesiredBodyMotion& body_motion_d = body_motion_pair.second;
    const RigidBody<double>* body = &(body_motion_d.body());
    auto tracker_it = body_trackers_.find(body);
    DRAKE_DEMAND(tracker_it != body_trackers_.end());
    // Compute measured body
    Isometry3<double> body_pose = rs.robot().relativeTransform(rs.cache(), 0, body->get_body_index());
    Vector6<double> body_vel = rs.robot().CalcTwistInWorldAlignedBody(rs.cache(), *body);

    // Compute desired accelerations
    body_motion_d.mutable_values() = tracker_it->second.ComputeTargetAcceleration(plan_time, body_pose, body_vel);
  }

  // Joint motion feedback
  qp_input.mutable_desired_dof_motions().mutable_values() =
      dof_tracker_.ComputeTargetAcceleration(plan_time, rs.position(), rs.velocity());

  // Contacts
  const ContactState& cur_contacts = contacts_traj_.get_contacts(plan_time);
  for (const auto& contact_pair : cur_contacts) {
    const ContactInformation& contact = contact_pair.second;
    auto it = qp_input.contact_information().find(contact.body_name());
    if (it == qp_input.contact_information().end()) {
      qp_input.mutable_contact_information().emplace(contact.body_name(), contact);
    }
  }

  return qp_input;
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"

#include <iostream>

#include "drake/common/constants.h"
#include "drake/math/quaternion.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/util/drakeGeometryUtil.h"
#include "drake/util/lcmUtil.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

void DecodeQPInput(const RigidBodyTree& robot, const lcmt_qp_input& msg, QPInput* qp_input) {
  if (!qp_input) return;

  ContactInformation info(*robot.FindBody("world"));
  qp_input->mutable_contact_information().clear();
  for (const auto& contact_msg : msg.contact_information) {
    DecodeContactInformation(robot, contact_msg, &info);
    qp_input->mutable_contact_information().emplace(info.body_name(), info);
  }

  DesiredBodyMotion mot(*robot.FindBody("world"));
  qp_input->mutable_desired_body_motions().clear();
  for (const auto& mot_msg : msg.desired_body_motions) {
    DecodeDesiredBodyMotion(robot, mot_msg, &mot);
    qp_input->mutable_desired_body_motions().emplace(mot.body_name(), mot);
  }

  DecodeDesiredDoFMotions(msg.desired_dof_motions, &(qp_input->mutable_desired_dof_motions()));
  DecodeDesiredCentroidalMomentumDot(msg.desired_centroidal_momentum_dot, &(qp_input->mutable_desired_centroidal_momentum_dot()));
  qp_input->mutable_w_basis_reg() = msg.w_basis_reg;

  if (!qp_input->is_valid(robot.get_num_velocities())) {
    throw std::runtime_error("invalid QPInput");
  }
}

void EncodeQPInput(const QPInput& qp_input, lcmt_qp_input* msg) {
  if (!msg) return;

  if (!qp_input.is_valid()) {
    throw std::runtime_error("invalid QPInput");
  }

  msg->num_contacts = static_cast<int>(qp_input.contact_information().size());
  msg->contact_information.resize(msg->num_contacts);
  int contact_ctr = 0;
  for (const auto& contact_pair : qp_input.contact_information()) {
    EncodeContactInformation(contact_pair.second, &(msg->contact_information[contact_ctr]));
    contact_ctr++;
  }

  msg->num_desired_body_motions = static_cast<int>(qp_input.desired_body_motions().size());
  msg->desired_body_motions.resize(msg->num_desired_body_motions);
  int desired_body_motion_ctr = 0;
  for (const auto& mot_pair : qp_input.desired_body_motions()) {
    EncodeDesiredBodyMotion(mot_pair.second, &(msg->desired_body_motions[desired_body_motion_ctr]));
    desired_body_motion_ctr++;
  }

  EncodeDesiredDoFMotions(qp_input.desired_dof_motions(), &(msg->desired_dof_motions));

  EncodeDesiredCentroidalMomentumDot(qp_input.desired_centroidal_momentum_dot(), &(msg->desired_centroidal_momentum_dot));

  msg->w_basis_reg = qp_input.w_basis_reg();
}

int8_t EncodeConstraintType(ConstraintType type) {
  switch (type) {
    case ConstraintType::Hard:
      return lcmt_constrained_values::HARD;
    case ConstraintType::Skip:
      return lcmt_constrained_values::SKIP;
    case ConstraintType::Soft:
      return lcmt_constrained_values::SOFT;
    default:
      throw std::runtime_error("unknown constraint type");
  }
}

ConstraintType DecodeConstraintType(int8_t type) {
  switch (type) {
    case lcmt_constrained_values::HARD:
      return ConstraintType::Hard;
    case lcmt_constrained_values::SKIP:
      return ConstraintType::Skip;
    case lcmt_constrained_values::SOFT:
      return ConstraintType::Soft;
    default:
      throw std::runtime_error("unknown constraint type");
  }
}

void DecodeDesiredBodyMotion(const RigidBodyTree& robot, const lcmt_desired_body_motion& msg, DesiredBodyMotion* body_motion) {
  if (!body_motion) return;

  *body_motion = DesiredBodyMotion(*robot.FindBody(msg.body_name));
  body_motion->mutable_control_during_contact() = msg.control_during_contact;
  DecodeConstrainedValues(msg.constrained_accelerations, body_motion);

  if (!body_motion->is_valid()) {
    throw std::runtime_error("invalid DesiredBodyMotion");
  }
}

void EncodeDesiredBodyMotion(const DesiredBodyMotion& body_motion, lcmt_desired_body_motion* msg) {
  if (!msg) return;
  if (!body_motion.is_valid()) {
    throw std::runtime_error("invalid DesiredBodyMotion");
  }

  msg->body_name = body_motion.body().get_name();
  msg->control_during_contact = body_motion.control_during_contact();
  EncodeConstrainedValues(body_motion, &(msg->constrained_accelerations));
}

void DecodeDesiredDoFMotions(const lcmt_desired_dof_motions& msg, DesiredDoFMotions* dof_motions) {
  if (!dof_motions) return;

  *dof_motions = DesiredDoFMotions(msg.dof_names);
  DecodeConstrainedValues(msg.constrained_accelerations, dof_motions);

  if (!dof_motions->is_valid()) {
    throw std::runtime_error("invalid DesiredDoFMotions");
  }
}

void EncodeDesiredDoFMotions(const DesiredDoFMotions& dof_motions, lcmt_desired_dof_motions* msg) {
  if (!msg) return;
  if (!dof_motions.is_valid()) {
    throw std::runtime_error("invalid DesiredDoFMotions");
  }

  msg->num_dof = dof_motions.size();
  msg->dof_names = dof_motions.dof_names();
  EncodeConstrainedValues(dof_motions, &(msg->constrained_accelerations));
}

void DecodeDesiredCentroidalMomentumDot(const lcmt_desired_centroidal_momentum_dot& msg, DesiredCentroidalMomentumDot* momdot) {
  if (!momdot) return;
  DecodeConstrainedValues(msg.centroidal_momentum_dot, momdot);
  if (!momdot->is_valid()) {
    throw std::runtime_error("invalid DesiredCentroidalMomentumDot");
  }
}

void EncodeDesiredCentroidalMomentumDot(const DesiredCentroidalMomentumDot& momdot, lcmt_desired_centroidal_momentum_dot* msg) {
  if (!msg) return;
  if (!momdot.is_valid()) {
    throw std::runtime_error("invalid DesiredCentroidalMomentumDot");
  }

  EncodeConstrainedValues(momdot, &(msg->centroidal_momentum_dot));
}

void DecodeConstrainedValues(const lcmt_constrained_values& msg, ConstrainedValues* val) {
  if (!val) return;
  if (msg.size != static_cast<int>(msg.types.size()) ||
      msg.types.size() != msg.weights.size() ||
      msg.types.size() != msg.values.size()) {
    throw std::runtime_error("lcmt_constrained_values has inconsistent dimensions.");
  }

  *val = ConstrainedValues(msg.size);
  for (int i = 0; i < msg.size; ++i) {
    val->mutable_constraint_type(i) = DecodeConstraintType(msg.types[i]);
    val->mutable_value(i) = msg.values[i];
    val->mutable_weight(i) = msg.weights[i];
  }

  if (!val->is_valid()) {
    throw std::runtime_error("invalid ConstrainedValues");
  }
}

void EncodeConstrainedValues(const ConstrainedValues& val, lcmt_constrained_values* msg) {
  if (!msg) return;
  if (!val.is_valid()) {
    throw std::runtime_error("invalid ConstrainedValues");
  }

  msg->size = val.size();
  msg->types.resize(msg->size);
  msg->weights.resize(msg->size);
  msg->values.resize(msg->size);

  for (int i = 0; i < static_cast<int>(msg->size); ++i) {
    msg->types[i] = EncodeConstraintType(val.constraint_type(i));
    msg->weights[i] = val.weight(i);
    msg->values[i] = val.value(i);
  }
}

void DecodeContactInformation(const RigidBodyTree& robot, const lcmt_contact_information& msg, ContactInformation* info) {
  if (!info) return;
  *info = ContactInformation(*robot.FindBody(msg.body_name), msg.num_basis_per_contact_point);
  info->mutable_contact_points().resize(msg.num_contact_points);
  // Check dimension of contact_points.
  if (msg.contact_points.size() != 3) {
    throw std::runtime_error("invalid contact_points row dimensions");
  }
  for (const auto& row : msg.contact_points) {
    if (static_cast<int>(row.size()) != msg.num_contact_points) {
      throw std::runtime_error("invalid contact_points col dimensions");
    }
  }
  // Parse contact points.
  for (int i = 0; i < msg.num_contact_points; ++i) {
    info->mutable_contact_points()[i] = Eigen::Vector3d(msg.contact_points[0][i], msg.contact_points[1][i], msg.contact_points[2][i]);
  }
  info->mutable_normal() = Eigen::Vector3d(msg.normal[0], msg.normal[1], msg.normal[2]);
  info->mutable_mu() = msg.mu;
  info->mutable_Kd() = msg.Kd;

  info->mutable_acceleration_constraint_type() = DecodeConstraintType(msg.acceleration_constraint_type);
  info->mutable_weight() = msg.weight;

  if (!info->is_valid()) {
    throw std::runtime_error("invalid ContactInformation.");
  }
}

void EncodeContactInformation(const ContactInformation& info, lcmt_contact_information* msg) {
  if (!msg) return;
  if (!info.is_valid()) {
    throw std::runtime_error("invalid ContactInformation.");
  }

  msg->body_name = info.body().get_name();
  msg->num_contact_points = static_cast<int>(info.contact_points().size());
  msg->num_basis_per_contact_point = info.num_basis_per_contact_point();
  msg->contact_points.resize(3);
  for (size_t i = 0; i < msg->contact_points.size(); ++i) {
    msg->contact_points[i].resize(msg->num_contact_points);
    for (int j = 0; j < msg->num_contact_points; ++j) {
      msg->contact_points[i][j] = info.contact_points()[j][i];
    }
  }

  for (int i = 0; i < 3; ++i) {
    msg->normal[i] = info.normal()[i];
  }

  msg->mu = info.mu();
  msg->Kd = info.Kd();

  msg->acceleration_constraint_type = EncodeConstraintType(info.acceleration_constraint_type());
  msg->weight = info.weight();
}

// TODO(siyuan.feng) Replace these with Twan's similar Encode / Decode methods.
void EncodeRobotStateLcmMsg(const std::vector<std::string>& act_joint_names,
                            double time, const Eigen::VectorXd& q,
                            const Eigen::VectorXd& qd,
                            const Eigen::VectorXd& joint_torque,
                            const Vector6<double>& l_foot_wrench,
                            const Vector6<double>& r_foot_wrench,
                            bot_core::robot_state_t* msg) {
  if (!msg) return;
  // Assuming q and qd belongs to a RPY floating based robot, and all the other
  // joints are fully actuated.
  const int floating_base_dim_q_dim = kSpaceDimension + kRpySize;
  if (q.size() != qd.size() ||
      q.size() != joint_torque.size() + floating_base_dim_q_dim ||
      act_joint_names.size() != static_cast<size_t>(joint_torque.size())) {
    throw std::runtime_error("invalid dimension");
  }

  msg->utime = static_cast<int64_t>(time * 1e6);
  msg->joint_name = act_joint_names;
  msg->num_joints = static_cast<char>(msg->joint_name.size());
  msg->joint_position.resize(msg->num_joints);
  msg->joint_velocity.resize(msg->num_joints);
  msg->joint_effort.resize(msg->num_joints);

  // Skip the first 6 floating base
  for (int i = floating_base_dim_q_dim; i < q.size(); ++i) {
    msg->joint_position[i - floating_base_dim_q_dim] = static_cast<float>(q[i]);
    msg->joint_velocity[i - floating_base_dim_q_dim] =
        static_cast<float>(qd[i]);
    msg->joint_effort[i - floating_base_dim_q_dim] =
        static_cast<float>(joint_torque[i - floating_base_dim_q_dim]);
  }

  // Set force torque readings.
  msg->force_torque.l_foot_force_z = static_cast<float>(l_foot_wrench[5]);
  msg->force_torque.l_foot_torque_x = static_cast<float>(l_foot_wrench[0]);
  msg->force_torque.l_foot_torque_y = static_cast<float>(l_foot_wrench[1]);
  msg->force_torque.r_foot_force_z = static_cast<float>(r_foot_wrench[5]);
  msg->force_torque.r_foot_torque_x = static_cast<float>(r_foot_wrench[0]);
  msg->force_torque.r_foot_torque_y = static_cast<float>(r_foot_wrench[1]);

  for (int i = 0; i < 3; ++i) {
    msg->force_torque.l_hand_force[i] = 0;
    msg->force_torque.l_hand_torque[i] = 0;
    msg->force_torque.r_hand_force[i] = 0;
    msg->force_torque.r_hand_torque[i] = 0;
  }

  // Set base
  Eigen::Isometry3d pose;
  pose.translation() = q.head<3>();
  pose.linear() = math::rpy2rotmat(q.segment<3>(3));
  EncodePose(pose, msg->pose);

  Eigen::Vector3d rpy = q.segment<3>(3);
  Eigen::Vector3d rpydot = qd.segment<3>(3);
  Eigen::Matrix3d phi = Eigen::Matrix3d::Zero();
  angularvel2rpydotMatrix(rpy, phi, (Eigen::MatrixXd*)nullptr,
                          (Eigen::MatrixXd*)nullptr);

  Vector6<double> vel;
  vel.head<3>() = phi.inverse() * rpydot;
  vel.tail<3>() = qd.head<3>();
  EncodeTwist(vel, msg->twist);
}

// TODO(siyuan.feng) Replace these with Twan's similar Encode / Decode methods.
void DecodeRobotStateLcmMsg(
    const bot_core::robot_state_t& msg,
    const std::unordered_map<std::string, int>& q_name_to_index, double* time,
    Eigen::VectorXd* q, Eigen::VectorXd* qd, Eigen::VectorXd* joint_torque,
    Vector6<double>* l_foot_wrench, Vector6<double>* r_foot_wrench) {
  if (!q || !qd || !joint_torque || !l_foot_wrench || !r_foot_wrench) return;
  const int floating_base_dim_q_dim = kSpaceDimension + kRpySize;
  if (q->size() != qd->size() ||
      q->size() != joint_torque->size() + floating_base_dim_q_dim) {
    throw std::runtime_error("invalid output state dimension");
  }
  if (msg.joint_position.size() != msg.joint_velocity.size() ||
      msg.joint_position.size() != msg.joint_effort.size()) {
    throw std::runtime_error("invalid input state dimension");
  }

  *time = static_cast<double>(msg.utime) / 1e6;

  std::unordered_map<std::string, int>::const_iterator it;

  // Set joint state.
  for (int i = 0; i < msg.num_joints; ++i) {
    it = q_name_to_index.find(msg.joint_name.at(i));
    // It's possible that the lcm message have more joints than what we care
    // about, so we will just ignore the extra joints.
    if (it != q_name_to_index.end()) {
      if (it->second > q->size()) {
        throw std::runtime_error("state index output bound");
      }
      (*q)[it->second] = static_cast<double>(msg.joint_position.at(i));
      (*qd)[it->second] = static_cast<double>(msg.joint_velocity.at(i));
      (*joint_torque)[it->second - floating_base_dim_q_dim] =
          static_cast<double>(msg.joint_effort.at(i));
    }
  }

  // Set floating base joint state.
  Eigen::Isometry3d pose = DecodePose(msg.pose);
  Vector6<double> vel = DecodeTwist(msg.twist);
  Eigen::Vector3d rpy = math::rotmat2rpy(pose.linear());
  Eigen::Matrix3d phi = Eigen::Matrix3d::Zero();
  angularvel2rpydotMatrix(rpy, phi, (Eigen::MatrixXd*)nullptr,
                          (Eigen::MatrixXd*)nullptr);
  Eigen::Vector3d rpydot = phi * vel.head<3>();

  (*q)[q_name_to_index.at("base_x")] = pose.translation()[0];
  (*q)[q_name_to_index.at("base_y")] = pose.translation()[1];
  (*q)[q_name_to_index.at("base_z")] = pose.translation()[2];
  (*qd)[q_name_to_index.at("base_x")] = vel[3];
  (*qd)[q_name_to_index.at("base_y")] = vel[4];
  (*qd)[q_name_to_index.at("base_z")] = vel[5];

  (*q)[q_name_to_index.at("base_roll")] = rpy[0];
  (*q)[q_name_to_index.at("base_pitch")] = rpy[1];
  (*q)[q_name_to_index.at("base_yaw")] = rpy[2];
  (*qd)[q_name_to_index.at("base_roll")] = rpydot[0];
  (*qd)[q_name_to_index.at("base_pitch")] = rpydot[1];
  (*qd)[q_name_to_index.at("base_yaw")] = rpydot[2];

  // Set foot force torque.
  l_foot_wrench->setZero();
  r_foot_wrench->setZero();
  (*l_foot_wrench)[0] = msg.force_torque.l_foot_torque_x;
  (*l_foot_wrench)[1] = msg.force_torque.l_foot_torque_y;
  (*l_foot_wrench)[5] = msg.force_torque.l_foot_force_z;
  (*r_foot_wrench)[0] = msg.force_torque.r_foot_torque_x;
  (*r_foot_wrench)[1] = msg.force_torque.r_foot_torque_y;
  (*r_foot_wrench)[5] = msg.force_torque.r_foot_force_z;
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

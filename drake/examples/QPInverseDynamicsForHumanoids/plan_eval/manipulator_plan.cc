#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/manipulator_plan.h"

#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"

#include <vector>

#include "drake/common/unused.h"
#include "drake/manipulation/util/motion_plan_translator.h"
#include "drake/util/drakeUtil.h"
#include "drake/util/lcmUtil.h"

#include "drake/manipulation/util/cartesian_trajectory_translator.h"
#include "drake/manipulation/util/dof_trajectory_translator.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

using manipulation::PiecewiseCartesianTrajectory;
using manipulation::PiecewiseCubicTrajectory;
using manipulation::CartesianTrajectoryTranslator;
using manipulation::DofTrajectoryTranslator;

template <typename T>
void ManipulatorPlan<T>::InitializeGenericPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {
  unused(paramset);  // TODO(jwnimmer-tri) This seems bad.
}

template <typename T>
void ManipulatorPlan<T>::HandlePlanMessageGenericPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
    const void* message_bytes, int message_length) {
  unused(paramset);  // TODO(jwnimmer-tri) This seems bad.

  /////////////////////////////////////////////////////////////
  // HACK
  // This first version assumes the given plan is actually a cartesian move
  // plan without contacts.

  // Tries to decode as a lcmt_motion_plan message.
  lcmt_motion_plan msg;
  int consumed = msg.decode(message_bytes, 0, message_length);
  DRAKE_DEMAND(consumed == message_length);

  // Q traj.
  PiecewiseCubicTrajectory<double> dof_traj;
  DofTrajectoryTranslator::DecodeMessage(msg.dof_motion, &dof_traj);
  dof_traj.shift_right(robot_status.time());
  this->set_dof_trajectory(dof_traj);

  // Body traj.
  std::string name;
  PiecewiseCartesianTrajectory<double> traj;
  for (const auto& body_motion : msg.body_motions) {
    CartesianTrajectoryTranslator::DecodeMessage(body_motion, &name, &traj);
    const RigidBody<T>* body = robot_status.robot().FindBody(name);
    traj.shift_right(robot_status.time());
    this->set_body_trajectory(body, traj);
  }

  // Contacts.
  override_contacts_.clear();
  ContactInformation tmp(
      *robot_status.robot().FindBody("world"));  // will get reset
  if (msg.num_contact_states != 0) {
    // TODO: currently, we are only looking at the first thing.
    const lcmt_contact_state& contact_state = msg.contact_states.front();
    for (int i = 0; i < contact_state.num_bodies_in_contact; ++i) {
      DecodeContactInformation(robot_status.robot(),
                               contact_state.bodies_in_contact[i], &tmp);
      override_contacts_.emplace(tmp.body_name(), tmp);
    }
  }
}

template <typename T>
void ManipulatorPlan<T>::UpdateQpInputGenericPlanDerived(
    const HumanoidStatus& status, const param_parsers::ParamSet&,
    const param_parsers::RigidBodyTreeAliasGroups<T>&, QpInput* qp_input,
    void* other_inputs) const {
  qp_input->mutable_contact_information() = override_contacts_;

  /*
  const RigidBody<double>* right_ee = status.robot().FindBody("right_iiwa_link_ee");
  auto right_pose = status.robot().CalcBodyPoseInWorldFrame(status.cache(), *right_ee);
  std::cout << "a " << right_pose.translation().transpose() << "\n";
  */

  static int init = 0;
  static double t0 = 0;

  // Picking stuff up
  if (!override_contacts_.empty()) {
    DRAKE_DEMAND(other_inputs != nullptr);
    qp_input->get_mutable_manipulation_objectives().emplace(
        "box", ManipulationObjective());

    // Don't hard code.
    ManipulationObjective& manip_obj =
        qp_input->get_mutable_manipulation_objectives()["box"];
    /////////////////////////////////////////////////////////
    // really bad
    const VectorX<T>* obj_state = static_cast<const VectorX<T>*>(other_inputs);
    /////////////////////////////////////////////////////////
    KinematicsCache<T> obj_cache =
        obj_->doKinematics(obj_state->head(obj_->get_num_positions()),
                           obj_state->tail(obj_->get_num_velocities()));

    // Set pose.
    manip_obj.get_mutable_X_WO() =
        obj_->CalcBodyPoseInWorldFrame(obj_cache, obj_->get_body(1));
    manip_obj.get_mutable_V_WO() =
        obj_->CalcBodySpatialVelocityInWorldFrame(obj_cache, obj_->get_body(1));

    Vector6<T> kp, kd;
    kp << 10, 10, 10, 10, 10, 10;
    kd << 3, 3, 3, 3, 3, 3;
    CartesianSetpoint<T> obj_controller(
        obj_traj_.get_pose(status.time()),
        obj_traj_.get_velocity(status.time()),
        obj_traj_.get_acceleration(status.time()), kp, kd);

    if (t0 == 0) {
      t0 = status.time();
    }

    // Set obj mass.
    manip_obj.set_M(obj_->massMatrix(obj_cache));
    drake::eigen_aligned_std_unordered_map<RigidBody<T> const*, Vector6<T>>
        f_ext;
    manip_obj.set_h(obj_->dynamicsBiasTerm(obj_cache, f_ext));

    // Set obj motion objective.
    manip_obj.get_mutable_desired_motion().SetAllConstraintType(
        ConstraintType::Soft);
    manip_obj.get_mutable_desired_motion().mutable_values() =
        obj_controller.ComputeTargetAcceleration(manip_obj.get_X_WO(),
                                                 manip_obj.get_V_WO());
    // HACK
    double hack_acc = -9.81 + (status.time() - t0) * 9.81;
    if (hack_acc >= 0)
      hack_acc = 0;
    //manip_obj.get_mutable_desired_motion().mutable_values()[5] += hack_acc;

    manip_obj.get_mutable_desired_motion().mutable_weights() << 10, 10, 10, 10,
        10, 10;

    // Set obj contacts.
    for (const auto& contact_pair : qp_input->contact_information()) {
      manip_obj.get_mutable_robot_contacts().insert(contact_pair.first);
    }

    // Turn off contact damping.
    for (auto& contact_pair : qp_input->mutable_contact_information()) {
      // TODO should really jsut disable this.
      contact_pair.second.mutable_weight() = 0.001;
      contact_pair.second.mutable_Kd() = 0.;
    }

    // Turn on both ee tracking to match the box.
    for (auto& body_motion_pair : qp_input->mutable_desired_body_motions()) {
      DesiredBodyMotion& body_motion = body_motion_pair.second;
      body_motion.mutable_control_during_contact() = true;

      body_motion.mutable_values() = manip_obj.get_desired_motion().values();
      // Now correct the linear acceleration.
      Isometry3<T> X_WB = status.robot().CalcBodyPoseInWorldFrame(
          status.cache(), *status.robot().FindBody(body_motion_pair.first));
      Vector3<T> r = X_WB.translation() - manip_obj.get_X_WO().translation();
      body_motion.mutable_values().template tail<3>() +=
          manip_obj.get_desired_motion().values().template head<3>().cross(r);
    }

    std::cout << "-------------------------\niteration: " << init << "\n";
    std::cout << *qp_input;
    init++;
  }

  // std::cout << "time: " << status.time() << std::endl;
  // std::cout << *qp_input << std::endl;
}

template <typename T>
void ManipulatorPlan<T>::ModifyPlanGenericPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet&,
    const param_parsers::RigidBodyTreeAliasGroups<T>&,
    void* other_inputs) {
  // picking up.
  if (!override_contacts_.empty()) {
    /////////////////////////////////////////////////////////
    // really bad
    const VectorX<T>* obj_state = static_cast<const VectorX<T>*>(other_inputs);
    /////////////////////////////////////////////////////////
    KinematicsCache<T> obj_cache =
        obj_->doKinematics(obj_state->head(obj_->get_num_positions()),
                           obj_state->tail(obj_->get_num_velocities()));
    const Isometry3<T> obj_X_WO =
        obj_->CalcBodyPoseInWorldFrame(obj_cache, obj_->get_body(1));

    switch (obj_state_id_) {
      case 0: {
        std::vector<double> times = {0, 1, 2};
        std::vector<Isometry3<double>> poses(times.size(), obj_X_WO);
        // poses.back().translation()[2] += 0.03;

        obj_traj_ = PiecewiseCartesianTrajectory<double>::MakeCubicLinearWithEndLinearVelocity(
            times, poses, Vector3<double>::Zero(), Vector3<double>::Zero());
        obj_traj_.shift_right(robot_status.time());

        obj_state_id_ = 1;

        break;
      }

      case 1: {
        break;
      }
    }
  }
}

template <typename T>
void ManipulatorPlan<T>::MakeDebugMessage(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
    const QpInput& qp_input, lcmt_plan_eval_debug_info* message) const {
  /*
  lcmt_plan_eval_debug_info& msg = *message;
  msg.timestamp = robot_status.time() * 1e6;

  // Dof
  manipulation::DofKeyframeTranslator::InitializeMessage(
      robot_status.robot().get_num_positions(),
      &msg.dof_motion);
  manipulation::DofKeyframeTranslator::EncodeMessage(robot_status.time(),
      this->get_dof_tracker().desired_position(),
      this->get_dof_tracker().desired_velocity(),
      this->get_dof_tracker().desired_acceleration(),
      &msg.dof_motion);

  // Body.
  const auto& body_trackers = this->get_body_trackers();
  if (!body_trackers.empty()) {
    msg.num_body_motions = body_trackers.size();
    msg.body_motion_names.resize(msg.num_body_motions);
    msg.body_motions.resize(msg.num_body_motions);
    int ctr = 0;
    for (const auto& tracker_pair : body_trackers) {
      manipulation::CartesianKeyframeTranslator::InitializeMessage(
          &msg.body_motions[ctr]);
      manipulation::CartesianKeyframeTranslator::EncodeMessage(
          tracker_pair.first->get_name(), robot_status.time(),
          tracker_pair.second.desired_pose(),
          tracker_pair.second.desired_velocity(),
          tracker_pair.second.desired_acceleration(),
          &msg.body_motions[ctr]);
      ctr++;

      std::cout << "desired body: " << tracker_pair.first->get_name() << ", " <<
  tracker_pair.second.desired_pose().translation().transpose() << std::endl;
    }
  } else {
    msg.num_body_motions = 1;
    msg.body_motion_names.resize(msg.num_body_motions);
    msg.body_motions.resize(msg.num_body_motions);
    manipulation::CartesianKeyframeTranslator::InitializeMessage(
        &msg.body_motions[0]);
    msg.body_motions[0].body_name = "iiwa_link_ee";
  }

  // Contact.
  msg.nominal_wrench[0] = 0;
  msg.nominal_wrench[1] = 0;
  msg.nominal_wrench[2] = 0;
  msg.nominal_wrench[3] = 0;
  msg.nominal_wrench[4] = 0;
  msg.nominal_wrench[5] = 0;
  if (!override_contacts_.empty()) {
    for (int i = 0; i < 3; ++i) {
      msg.nominal_wrench[3 + i] =
  override_contacts_.at("iiwa_link_ee").desired_force()[i];
    }
  }
  */
}

template <typename T>
GenericPlan<T>* ManipulatorPlan<T>::CloneGenericPlanDerived() const {
  return new ManipulatorPlan(*this);
}

template class ManipulatorPlan<double>;

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

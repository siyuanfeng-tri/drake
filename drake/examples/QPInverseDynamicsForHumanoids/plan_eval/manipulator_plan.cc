#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/manipulator_plan.h"

#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"

#include <vector>

#include "drake/common/unused.h"
#include "drake/manipulation/util/motion_plan_translator.h"
#include "drake/util/lcmUtil.h"
#include "drake/util/drakeUtil.h"

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
  std::cout << "asdf: " << msg.body_motions.size() << std::endl;
  PiecewiseCartesianTrajectory<double> traj;
  for (const auto& body_motion : msg.body_motions) {
    CartesianTrajectoryTranslator::DecodeMessage(body_motion, &name, &traj);
    std::cout << name << std::endl;
    const RigidBody<T>* body = robot_status.robot().FindBody(name);
    traj.shift_right(robot_status.time());
    this->set_body_trajectory(body, traj);
  }

  // Contacts.
  override_contacts_.clear();
  ContactInformation tmp(*robot_status.robot().FindBody("world")); // will get reset
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
      const HumanoidStatus& status,
      const param_parsers::ParamSet&,
      const param_parsers::RigidBodyTreeAliasGroups<T>&,
      QpInput* qp_input) const {
  qp_input->mutable_contact_information() = override_contacts_;
  std::cout << "time: " << status.time() << std::endl;
  std::cout << *qp_input << std::endl;
}

template <typename T>
void ManipulatorPlan<T>::ModifyPlanGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet&,
      const param_parsers::RigidBodyTreeAliasGroups<T>&) {
  if (!override_contacts_.empty()) {
    override_contacts_.at("iiwa_link_ee").mutable_desired_force()[1] = 2 * cos(robot_status.time());
    override_contacts_.at("iiwa_link_ee").mutable_desired_force()[2] = 2 * sin(robot_status.time());
  }
}

template <typename T>
void ManipulatorPlan<T>::MakeDebugMessage(
    const HumanoidStatus& robot_status,
    const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
    const QpInput& qp_input,
    lcmt_plan_eval_debug_info* message) const {

  lcmt_plan_eval_debug_info& msg = *message;
  msg.timestamp = robot_status.time() * 1e6;

  // Dof
  manipulation::DofKeyframeTranslator::InitializeMessage(7,
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

      std::cout << "desired body: " << tracker_pair.first->get_name() << ", " << tracker_pair.second.desired_pose().translation().transpose() << std::endl;
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
      msg.nominal_wrench[3 + i] = override_contacts_.at("iiwa_link_ee").desired_force()[i];
    }
  }
}

template <typename T>
GenericPlan<T>* ManipulatorPlan<T>::CloneGenericPlanDerived()
    const {
  return new ManipulatorPlan(*this);
}

template class ManipulatorPlan<double>;

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

#include "drake/examples/QPInverseDynamicsForHumanoids/system/plan_eval_system.h"

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <utility>

#include "drake/common/drake_path.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

// Example struct of a "Plan" object containing internal state such as desired
// trajectories or setpoints.
struct SimplePlan {
  VectorSetpoint<double> joint_PDff;
  CartesianSetpoint<double> pelvis_PDff;
  CartesianSetpoint<double> torso_PDff;

  Vector3<double> desired_com;
  Vector3<double> initial_com;
  Vector3<double> Kp_com;
  Vector3<double> Kd_com;
};

PlanEvalSystem::PlanEvalSystem(const RigidBodyTree<double>& robot)
    : robot_(robot), alias_groups_(robot) {
  input_port_index_humanoid_status_ = DeclareAbstractInputPort().get_index();
  output_port_index_qp_input_ = DeclareAbstractOutputPort().get_index();

  set_name("plan_eval");

  std::string alias_groups_config =
      drake::GetDrakePath() + std::string(
                                  "/examples/QPInverseDynamicsForHumanoids/"
                                  "config/alias_groups.yaml");
  std::string controller_config =
      drake::GetDrakePath() + std::string(
                                  "/examples/QPInverseDynamicsForHumanoids/"
                                  "config/controller.yaml");

  // KinematicsProperty
  alias_groups_.LoadFromYAMLFile(YAML::LoadFile(alias_groups_config));

  // Controller config
  paramset_.LoadFromYAMLConfigFile(YAML::LoadFile(controller_config),
                                   alias_groups_);
}

void PlanEvalSystem::DoCalcOutput(const systems::Context<double>& context,
                                  systems::SystemOutput<double>* output) const {
  // Output:
  lcmt_qp_input& msg = output->GetMutableData(output_port_index_qp_input_)
                           ->GetMutableValue<lcmt_qp_input>();

  // Gets QPInput from AbstractState, encodes and sends as a lcm msg.
  const QPInput& qp_input = context.get_abstract_state<QPInput>(1);
  EncodeQPInput(qp_input, &msg);
}

void PlanEvalSystem::DoCalcNextUpdateTime(
    const systems::Context<double>& context,
    systems::UpdateActions<double>* actions) const {
  actions->time = context.get_time() + control_dt_;
  actions->events.push_back(systems::DiscreteEvent<double>());
  actions->events.back().action =
      systems::DiscreteEvent<double>::kUnrestrictedUpdateAction;
}

void PlanEvalSystem::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  systems::AbstractState* abs_state = state->get_mutable_abstract_state();
  DRAKE_DEMAND(abs_state->size() == 2);

  // Gets the plan from abstract state.
  SimplePlan& plan =
      abs_state->get_mutable_abstract_state(0).GetMutableValue<SimplePlan>();

  // Gets the robot state from input.
  const HumanoidStatus* robot_status = EvalInputValue<HumanoidStatus>(
      context, input_port_index_humanoid_status_);

  std::cout << context.get_time() << robot_status->time() <<std::endl;

  /////////////////////////////////////////////////////////////////////////////
  // Mutates the plan.
  // This can be much more complicated like replanning trajectories, etc.
  paramset_.LookupDesiredBodyMotionGains(*robot_.FindBody("pelvis"),
                                         &(plan.pelvis_PDff.mutable_Kp()),
                                         &(plan.pelvis_PDff.mutable_Kd()));
  paramset_.LookupDesiredBodyMotionGains(*robot_.FindBody("torso"),
                                         &(plan.torso_PDff.mutable_Kp()),
                                         &(plan.torso_PDff.mutable_Kd()));
  paramset_.LookupDesiredDoFMotionGains(&(plan.joint_PDff.mutable_Kp()),
                                        &(plan.joint_PDff.mutable_Kd()));

  Vector6<double> Kp, Kd;
  paramset_.LookupDesiredCentroidalMomentumDotGains(&Kp, &Kd);
  plan.Kp_com = Kp.tail<3>();
  plan.Kd_com = Kd.tail<3>();

  // Moves desired com height in a sine wave.
  plan.desired_com[2] =
      plan.initial_com[2] + 0.1 * std::sin(robot_status->time() * 2 * M_PI);

  /////////////////////////////////////////////////////////////////////////////
  // Generates a QPInput and store it in AbstractState.
  QPInput& qp_input =
      abs_state->get_mutable_abstract_state(1).GetMutableValue<QPInput>();
  qp_input = QPInput(GetDoFNames(robot_));
  qp_input.mutable_contact_information() =
      paramset_.MakeContactInformation("feet", alias_groups_);

  std::unordered_map<std::string, DesiredBodyMotion> motion_d =
      paramset_.MakeDesiredBodyMotion("pelvis", alias_groups_);
  qp_input.mutable_desired_body_motions().insert(motion_d.begin(),
                                                 motion_d.end());
  motion_d = paramset_.MakeDesiredBodyMotion("torso", alias_groups_);
  qp_input.mutable_desired_body_motions().insert(motion_d.begin(),
                                                 motion_d.end());

  qp_input.mutable_desired_dof_motions() = paramset_.MakeDesiredDoFMotions();
  qp_input.mutable_w_basis_reg() = paramset_.get_basis_regularization_weight();

  qp_input.mutable_desired_centroidal_momentum_dot() =
      paramset_.MakeDesiredCentroidalMomentumDot();

  // Does acceleration feedback based on the plan.
  Vector3<double> com_err = plan.desired_com - robot_status->com();
  Vector3<double> comd_err = -robot_status->comd();

  qp_input.mutable_desired_centroidal_momentum_dot()
      .mutable_values()
      .tail<3>() = robot_.getMass() *
                   (plan.Kp_com.array() * com_err.array() +
                    plan.Kd_com.array() * comd_err.array())
                       .matrix();

  qp_input.mutable_desired_dof_motions().mutable_values() =
      plan.joint_PDff.ComputeTargetAcceleration(robot_status->position(),
                                                robot_status->velocity());

  qp_input.mutable_desired_body_motions().at("pelvis").mutable_values() =
      plan.pelvis_PDff.ComputeTargetAcceleration(
          robot_status->pelvis().pose(), robot_status->pelvis().velocity());
  qp_input.mutable_desired_body_motions().at("torso").mutable_values() =
      plan.torso_PDff.ComputeTargetAcceleration(
          robot_status->torso().pose(), robot_status->torso().velocity());
}

std::unique_ptr<systems::AbstractState> PlanEvalSystem::AllocateAbstractState()
    const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals;
  abstract_vals.reserve(2);
  abstract_vals.push_back(std::unique_ptr<systems::AbstractValue>(
      new systems::Value<SimplePlan>(SimplePlan())));
  abstract_vals.push_back(std::unique_ptr<systems::AbstractValue>(
      new systems::Value<QPInput>(QPInput(GetDoFNames(robot_)))));
  return std::make_unique<systems::AbstractState>(std::move(abstract_vals));
}

std::unique_ptr<systems::SystemOutput<double>> PlanEvalSystem::AllocateOutput(
    const systems::Context<double>& context) const {
  std::unique_ptr<systems::LeafSystemOutput<double>> output(
      new systems::LeafSystemOutput<double>);
  output->add_port(std::unique_ptr<systems::AbstractValue>(
      new systems::Value<lcmt_qp_input>(lcmt_qp_input())));
  return std::move(output);
}

void PlanEvalSystem::SetDesired(const VectorX<double>& q_d,
                                systems::State<double>* state) {
  systems::AbstractState* abs_state = state->get_mutable_abstract_state();
  DRAKE_DEMAND(abs_state->size() == 2);

  // Get the plan.
  SimplePlan& plan =
      abs_state->get_mutable_abstract_state(0).GetMutableValue<SimplePlan>();

  KinematicsCache<double> cache = robot_.doKinematics(q_d);

  plan.initial_com = plan.desired_com = robot_.centerOfMass(cache);
  plan.pelvis_PDff.mutable_desired_pose() = robot_.relativeTransform(
      cache, 0,
      alias_groups_.get_body_group("pelvis").front()->get_body_index());
  plan.torso_PDff.mutable_desired_pose() = robot_.relativeTransform(
      cache, 0,
      alias_groups_.get_body_group("torso").front()->get_body_index());

  int dim = robot_.get_num_velocities();
  plan.joint_PDff = VectorSetpoint<double>(dim);
  plan.joint_PDff.mutable_desired_position() = q_d;
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

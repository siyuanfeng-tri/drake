#include "drake/examples/QPInverseDynamicsForHumanoids/system/plan_eval_system.h"
#include "drake/common/drake_path.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

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

  paramset_.LookupDesiredBodyMotionGains(*robot_.FindBody("pelvis"),
                                         &Kp_pelvis_, &Kd_pelvis_);
  paramset_.LookupDesiredBodyMotionGains(*robot_.FindBody("torso"), &Kp_torso_,
                                         &Kd_torso_);
  paramset_.LookupDesiredDoFMotionGains(&Kp_dof_, &Kd_dof_);
  Vector6<double> Kp, Kd;
  paramset_.LookupDesiredCentroidalMomentumDotGains(&Kp, &Kd);
  Kp_com_ = Kp.tail<3>();
  Kd_com_ = Kd.tail<3>();
}

void PlanEvalSystem::DoCalcOutput(const systems::Context<double>& context,
                                  systems::SystemOutput<double>* output) const {
  // Input:
  const HumanoidStatus* robot_status = EvalInputValue<HumanoidStatus>(
      context, input_port_index_humanoid_status_);

  // Output:
  lcmt_qp_input& msg = output->GetMutableData(output_port_index_qp_input_)
                           ->GetMutableValue<lcmt_qp_input>();

  Vector3<double> com_err = desired_com_ - robot_status->com();
  Vector3<double> comd_err = -robot_status->comd();

  // Make a new QPInput.
  QPInput qp_input(robot_);
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

  // Do Feedback.
  qp_input.mutable_desired_centroidal_momentum_dot()
      .mutable_values()
      .tail<3>() =
      robot_.getMass() *
      (Kp_com_.array() * com_err.array() + Kd_com_.array() * comd_err.array())
          .matrix();

  qp_input.mutable_desired_dof_motions().mutable_values() =
      joint_PDff_.ComputeTargetAcceleration(robot_status->position(),
                                            robot_status->velocity());
  qp_input.mutable_desired_body_motions().at("pelvis").mutable_values() =
      pelvis_PDff_.ComputeTargetAcceleration(robot_status->pelvis().pose(),
                                             robot_status->pelvis().velocity());
  qp_input.mutable_desired_body_motions().at("torso").mutable_values() =
      torso_PDff_.ComputeTargetAcceleration(robot_status->torso().pose(),
                                            robot_status->torso().velocity());

  // Encode and send.
  EncodeQPInput(qp_input, &msg);
}

std::unique_ptr<systems::SystemOutput<double>> PlanEvalSystem::AllocateOutput(
    const systems::Context<double>& context) const {
  std::unique_ptr<systems::LeafSystemOutput<double>> output(
      new systems::LeafSystemOutput<double>);
  output->add_port(std::unique_ptr<systems::AbstractValue>(
      new systems::Value<lcmt_qp_input>(lcmt_qp_input())));
  return std::move(output);
}

void PlanEvalSystem::SetDesired(const HumanoidStatus& robot_status) {
  desired_com_ = robot_status.com();
  pelvis_PDff_ = CartesianSetpoint<double>(
      robot_status.pelvis().pose(), Vector6<double>::Zero(),
      Vector6<double>::Zero(), Kp_pelvis_, Kd_pelvis_);
  torso_PDff_ = CartesianSetpoint<double>(
      robot_status.torso().pose(), Vector6<double>::Zero(),
      Vector6<double>::Zero(), Kp_torso_, Kd_torso_);
  int dim = robot_status.position().size();
  joint_PDff_ = VectorSetpoint<double>(
      robot_status.position(), VectorX<double>::Zero(dim),
      VectorX<double>::Zero(dim), Kp_dof_, Kd_dof_);
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

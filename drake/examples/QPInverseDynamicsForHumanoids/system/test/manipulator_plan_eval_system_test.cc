#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/manipulator_plan_eval_system.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

GTEST_TEST(testManipPlanEval, ManipPlanEval) {
  const std::string kModelPath =
      drake::GetDrakePath() +
      "/examples/kuka_iiwa_arm/models/iiwa14/iiwa14.urdf";

  const std::string kAliasGroupsPath =
      drake::GetDrakePath() +
      "/examples/QPInverseDynamicsForHumanoids/config/"
      "iiwa.alias_groups";

  const std::string kControllConfigPath =
      drake::GetDrakePath() +
      "/examples/QPInverseDynamicsForHumanoids/config/"
      "iiwa.id_controller_config";

  auto robot = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      kModelPath, multibody::joints::kFixed, robot.get());

  param_parsers::RigidBodyTreeAliasGroups<double> alias_groups(*robot);
  alias_groups.LoadFromFile(kAliasGroupsPath);

  VectorX<double> q(robot->get_num_positions());
  VectorX<double> v(robot->get_num_velocities());
  VectorX<double> x_d(q.size() + v.size());
  VectorX<double> vd_d(v.size());

  // Sets estimated position and velocity.
  for (int i = 0; i < q.size(); ++i) q[i] = i;
  for (int i = 0; i < v.size(); ++i) v[i] = 0.1 * i;

  // Sets desired position velocity and acceleration.
  for (int i = 0; i < x_d.size(); ++i) {
    x_d[i] = (-M_PI + i) / 2.;
  }
  for (int i = 0; i < vd_d.size(); ++i) {
    vd_d[i] = -1 + i;
  }
  HumanoidStatus robot_status(*robot, alias_groups);
  robot_status.Update(0, q, v,
                      VectorX<double>::Zero(robot->get_num_actuators()),
                      Vector6<double>::Zero(), Vector6<double>::Zero());

  // Makes a plan eval block and system related structs.
  std::unique_ptr<ManipulatorPlanEvalSystem> plan_eval =
      std::make_unique<ManipulatorPlanEvalSystem>(*robot, kAliasGroupsPath,
                                                  kControllConfigPath, 0.02);
  std::unique_ptr<systems::Context<double>> context =
      plan_eval->CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      plan_eval->AllocateOutput(*context);

  // Initializes.
  plan_eval->InitializePlan(robot_status, context->get_mutable_state());

  // Connects inputs.
  context->FixInputPort(
      plan_eval->get_input_port_humanoid_status().get_index(),
      systems::AbstractValue::Make<HumanoidStatus>(robot_status));

  context->FixInputPort(plan_eval->get_input_port_desired_state().get_index(),
                        std::make_unique<systems::BasicVector<double>>(x_d));
  context->FixInputPort(
      plan_eval->get_input_port_desired_acceleration().get_index(),
      std::make_unique<systems::BasicVector<double>>(vd_d));

  // Computes results.
  systems::DiscreteEvent<double> event;
  event.action = systems::DiscreteEvent<double>::kUnrestrictedUpdateAction;
  std::unique_ptr<systems::State<double>> state = context->CloneState();

  plan_eval->CalcUnrestrictedUpdate(*context, event, state.get());
  context->get_mutable_state()->CopyFrom(*state);
  plan_eval->CalcOutput(*context, output.get());

  const QpInput& qp_input = output->get_data(0)->GetValue<QpInput>();

  // Computes the expected output.
  const param_parsers::ParamSet& params = plan_eval->get_paramset();
  VectorX<double> kp(q.size());
  VectorX<double> kd(v.size());
  params.LookupDesiredDofMotionGains(&kp, &kd);

  VectorX<double> expected =
      vd_d + (kp.array() * (x_d.head(q.size()) - q).array()).matrix() +
      (kd.array() * (x_d.tail(v.size()) - v).array()).matrix();

  // Desired generalized acceleration should match expected.
  EXPECT_TRUE(
      drake::CompareMatrices(expected, qp_input.desired_dof_motions().values(),
                             1e-12, drake::MatrixCompareType::absolute));
  VectorX<double> expected_weights = params.MakeDesiredDofMotions().weights();
  EXPECT_TRUE(drake::CompareMatrices(
      expected_weights, qp_input.desired_dof_motions().weights(), 1e-12,
      drake::MatrixCompareType::absolute));
  std::vector<ConstraintType> expected_constraint_type =
      params.MakeDesiredDofMotions().constraint_types();
  for (size_t i = 0; i < expected_constraint_type.size(); ++i) {
    EXPECT_EQ(qp_input.desired_dof_motions().constraint_type(i),
              expected_constraint_type[i]);
  }

  // Contact force basis regularization weight is irrelevant here since there
  // is not contacts, but its value should match params'.
  EXPECT_EQ(qp_input.w_basis_reg(), params.get_basis_regularization_weight());

  // Not tracking Cartesian motions.
  EXPECT_TRUE(qp_input.desired_body_motions().empty());

  // No contacts.
  EXPECT_TRUE(qp_input.contact_information().empty());

  // Doesn't care about overall center of mass or angular momentum.
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(qp_input.desired_centroidal_momentum_dot().value(i), 0);
    EXPECT_EQ(qp_input.desired_centroidal_momentum_dot().weight(i), 0);
    EXPECT_EQ(qp_input.desired_centroidal_momentum_dot().constraint_type(i),
              ConstraintType::Skip);
  }
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

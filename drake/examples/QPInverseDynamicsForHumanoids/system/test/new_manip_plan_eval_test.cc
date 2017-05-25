#include "drake/examples/QPInverseDynamicsForHumanoids/system/new_manipulator_plan_eval_system.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/manipulation/util/motion_plan_translator.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/lcmt_motion_plan.hpp"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

using manipulation::MotionPlanTranslator;
using manipulation::CartesianTrajectoryTranslator;

void aa() {
  auto robot = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFile(
      GetDrakePath() +
      "/manipulation/models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf",
      drake::multibody::joints::kFixed, nullptr,
      robot.get());

  const std::string kAliasGroupsPath =
      drake::GetDrakePath() +
      "/examples/QPInverseDynamicsForHumanoids/"
      "config/iiwa.alias_groups";

  const std::string kControlConfigPath =
      drake::GetDrakePath() +
      "/examples/QPInverseDynamicsForHumanoids/"
      "config/iiwa.id_controller_config";

  auto dut = std::make_unique<ManipulatorPlanEvalSystem>(
      *robot, kAliasGroupsPath, kControlConfigPath, 0.002);

  auto context = dut->CreateDefaultContext();

  HumanoidStatus robot_status(*robot, dut->get_alias_groups());

  robot_status.UpdateKinematics(0., VectorX<double>::Zero(7), VectorX<double>::Zero(7));

  {
    auto input = systems::AbstractValue::Make<HumanoidStatus>(robot_status);
    context->FixInputPort(dut->get_input_port_humanoid_status().get_index(),
        std::move(input));
  }

  // Make plan message.
  lcmt_motion_plan msg;
  MotionPlanTranslator::InitializeMessage(&msg);

  std::vector<double> times = {0, 2};
  std::vector<Isometry3<double>> poses(2, Isometry3<double>::Identity());
  std::vector<Vector6<double>> velocities(2, Vector6<double>::Zero());
  std::vector<Vector6<double>> accelerations(2, Vector6<double>::Zero());

  msg.num_body_motions = 1;
  msg.body_motions.resize(1);
  CartesianTrajectoryTranslator::EncodeMessage("flange",
      times, poses, velocities, accelerations, &(msg.body_motions[0]));

  {
    auto input = systems::AbstractValue::Make<lcmt_motion_plan>(msg);
    context->FixInputPort(dut->get_input_port_plan().get_index(),
        std::move(input));
  }

  // Init controller.
  dut->Initialize(robot_status, context->get_mutable_state());

  systems::DiscreteEvent<double> event;
  event.action = systems::DiscreteEvent<double>::ActionType::kUnrestrictedUpdateAction;
  dut->CalcUnrestrictedUpdate(*context, event, context->get_mutable_state());

  // Compute output.
  auto output = dut->AllocateOutput(*context);
  dut->CalcOutput(*context, output.get());
  const QpInput& qp_input =
      output->get_data(0)->GetValue<QpInput>();

  std::cout << qp_input;
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

int main() {
  drake::examples::qp_inverse_dynamics::aa();
  return 0;
}

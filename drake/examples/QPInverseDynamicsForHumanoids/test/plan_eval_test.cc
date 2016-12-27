#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/manip_plan.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"


namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

GTEST_TEST(TestManipPlanEval, TestManipPlanEval) {
  // Loads model.
  std::string urdf =
      drake::GetDrakePath() +
      std::string(
          "/examples/Valkyrie/urdf/urdf/"
          "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom_no_collision.urdf");
  auto robot = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      urdf, multibody::joints::kRollPitchYaw, robot.get());

  HumanoidStatus robot_status(*robot);

  // Set up initial condition.
  VectorX<double> q(robot->get_num_positions());
  VectorX<double> v(robot->get_num_velocities());

  q = robot_status.GetNominalPosition();
  v.setZero();
  VectorX<double> q_ini = q;

  robot_status.Update(0, q, v, VectorX<double>::Zero(robot->actuators.size()),
                      Vector6<double>::Zero(), Vector6<double>::Zero());

  HumanoidManipPlan manip(*robot);

  // make plan
  QPController con;
  QPOutput output(*robot);

  manip.HandleManipPlan(robot_status);

  // Perturb initial condition.
  v[robot_status.name_to_position_index().at("torsoRoll")] += 1;
  robot_status.Update(0, q, v, VectorX<double>::Zero(robot->actuators.size()),
                      Vector6<double>::Zero(), Vector6<double>::Zero());

  // dt = 3e-3 is picked arbitrarily, with Gurobi, this one control call takes
  // roughly 3ms.
  double dt = 3e-3;
  double time = 0;

  // Feet should be stationary.
  EXPECT_TRUE(robot_status.foot(Side::LEFT).velocity().norm() < 1e-10);
  EXPECT_TRUE(robot_status.foot(Side::RIGHT).velocity().norm() < 1e-10);

  int tick_ctr = 0;
  while (time < 4) {
    QPInput input = manip.CalcQPInput(robot_status);
    int status = con.Control(robot_status, input, &output);

    if (status) {
      std::cout << input << output;
      break;
    }

    // Dummy integration.
    // TODO(siyuan.feng): replace this with sys2 simulator when it's ready.
    q += v * dt;
    v += output.vd() * dt;
    time += dt;

    robot_status.Update(time, q, v,
                        VectorX<double>::Zero(robot->actuators.size()),
                        Vector6<double>::Zero(), Vector6<double>::Zero());
    tick_ctr++;
  }

  // Check the final state.
  // Since the feet have equality constraints set to 0 in the qp controller,
  // they should have no velocity after simulation.
  // Thus, the tolerances on feet velocities are smaller than those for the
  // generalized position and velocity.
  std::cout << robot_status.foot(Side::LEFT).velocity().transpose() << std::endl;
  std::cout << robot_status.foot(Side::RIGHT).velocity().transpose() << std::endl;

  EXPECT_TRUE(robot_status.foot(Side::LEFT).velocity().norm() < 1e-6);
  EXPECT_TRUE(robot_status.foot(Side::RIGHT).velocity().norm() < 1e-6);
  EXPECT_TRUE(drake::CompareMatrices(q, q_ini, 1e-3,
                                     drake::MatrixCompareType::absolute));
  EXPECT_TRUE(drake::CompareMatrices(
      v, VectorX<double>::Zero(robot->get_num_velocities()), 1e-5,
      drake::MatrixCompareType::absolute));

  std::cout << output;
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

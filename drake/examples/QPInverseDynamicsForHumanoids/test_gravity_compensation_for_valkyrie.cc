#include "drake/common/drake_path.h"
#include "qp_controller.h"
#include "drake/math/rotation_matrix.h"
#include <eigen3/Eigen/Geometry>
#include <chrono>

#include "dummy_integrate.h"
#include "util.h"

QPOutput TestGravityCompensation(HumanoidStatus& robot_status) {
  // Make controller.
  QPController con(robot_status);
  QPInput input;
  QPOutput output;
  InitQPOutput(robot_status.robot(), &output);
  InitQPInput(robot_status.robot(), &input);

  // Make input.
  // These represent the desired motions for the robot, and are typically
  // outputs of motion planner or hand-crafted behavior state machines.
  input.comdd_d.setZero();
  input.pelvdd_d.setZero();
  input.torsodd_d.setZero();
  input.footdd_d[Side::LEFT].setZero();
  input.footdd_d[Side::RIGHT].setZero();
  input.wrench_d[Side::LEFT].setZero();
  input.wrench_d[Side::RIGHT].setZero();
  input.vd_d.setZero();
  // [5] is Fz, 660N * 2 is about robot weight.
  input.wrench_d[Side::LEFT][5] = 660;
  input.wrench_d[Side::RIGHT][5] = 660;

  // Weights are set arbitrarily by the control designer, these typically
  // require tuning.
  input.w_com = 1e2;
  input.w_pelv = 1e1;
  input.w_torso = 1e1;
  input.w_foot = 1e1;
  input.w_vd = 1e3;
  input.w_wrench_reg = 1e-5;

  double t0 = get_time();
  for (int i = 0; i < 100; i++) {
    con.Control(robot_status, input, &output);
  }
  printf("%g\n", (get_time() - t0) / 100.);

  // Print quadratic costs for all the terms.
  ComputeQPCost(robot_status, input, output);

  return output;
}

int main() {
  // Loads model.
  std::string urdf =
      drake::GetDrakePath() +
      std::string(
          "/examples/QPInverseDynamicsForHumanoids/valkyrie_sim_drake.urdf");
  HumanoidStatus robot_status(
      std::make_shared<RigidBodyTree>(urdf, DrakeJoint::ROLLPITCHYAW));

  // Sets state and does kinematics.
  VectorXd q(robot_status.robot().number_of_positions());
  VectorXd v(robot_status.robot().number_of_velocities());

  q = robot_status.GetNominalPosition();
  v.setZero();

  robot_status.Update(0, q, v,
                      VectorXd::Zero(robot_status.robot().actuators.size()),
                      Vector6d::Zero(), Vector6d::Zero());

  QPOutput output = TestGravityCompensation(robot_status);

  // Print results.
  PrintQPOutput(output);

  return 0;
}

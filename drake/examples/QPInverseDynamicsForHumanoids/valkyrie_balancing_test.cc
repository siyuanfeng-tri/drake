#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/example_balancing_controller.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"
#include "drake/systems/plants/joints/floating_base_types.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {
// In this test, the Valkyrie robot is initialized to a nominal configuration
// with zero velocities, and the qp controller is setup to track this
// state. The robot is then perturbed in velocity for the Torso Pitch joint.
// The test forward simulates the closed loop system for 2 seconds.
// The simulation does not perform forward dynamics computation, instead, it
// integrates the computed acceleration from the controller. This dummy
// simulation should be replaced later with real simulation.
// The controller should drive the position and velocity close to zero in 2
// seconds.
GTEST_TEST(testQPInverseDynamicsController, testStanding) {
  // Loads model.
  std::string urdf =
      drake::GetDrakePath() +
      std::string(
          "/examples/Valkyrie/urdf/urdf/valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf");
  RigidBodyTree robot(urdf, drake::systems::plants::joints::kRollPitchYaw);
  HumanoidStatus robot_status(robot);

  QPController con;
  QPInput input(robot_status.robot());
  QPOutput output(robot_status.robot());

  // Setup initial condition.
  Eigen::VectorXd q(robot_status.robot().get_num_positions());
  Eigen::VectorXd v(robot_status.robot().get_num_velocities());

  q = robot_status.GetNominalPosition();
  v.setZero();
  Eigen::VectorXd q_ini = q;

  robot_status.Update(
      0, q, v, Eigen::VectorXd::Zero(robot_status.robot().actuators.size()),
      Eigen::Vector6d::Zero(), Eigen::Vector6d::Zero());

  // Setup a tracking problem.
  Eigen::Vector3d Kp_com = Eigen::Vector3d::Constant(40);
  Eigen::Vector3d Kd_com = Eigen::Vector3d::Constant(12);

  Eigen::Vector3d desired_com = robot_status.com();

  // Get an example controller that tracks a fixed point.
  input = MakeExampleQPInput(robot);
  VectorSetpoint joint_PDff(q, Eigen::VectorXd::Zero(q.size()), Eigen::VectorXd::Zero(q.size()), Eigen::VectorXd::Constant(q.size(), 20), Eigen::VectorXd::Constant(q.size(), 8));
  CartesianSetpoint pelvis_PDff(robot_status.pelvis().pose(), Eigen::Vector6d::Zero(), Eigen::Vector6d::Zero(), Eigen::Vector6d::Constant(20), Eigen::Vector6d::Constant(8));
  CartesianSetpoint torso_PDff(robot_status.torso().pose(), Eigen::Vector6d::Zero(), Eigen::Vector6d::Zero(), Eigen::Vector6d::Constant(20), Eigen::Vector6d::Constant(8));

  // Perturb initial condition.
  v[robot_status.name_to_position_index().at("torsoPitch")] += 1;
  robot_status.Update(
      0, q, v, Eigen::VectorXd::Zero(robot_status.robot().actuators.size()),
      Eigen::Vector6d::Zero(), Eigen::Vector6d::Zero());

  // dt = 4e-3 is picked arbitrarily to ensure the test finishes within a
  // reasonable amount of time.
  double dt = 3e-3;
  double time = 0;

  // Feet should be stationary.
  EXPECT_TRUE(robot_status.foot(Side::LEFT).velocity().norm() < 1e-10);
  EXPECT_TRUE(robot_status.foot(Side::RIGHT).velocity().norm() < 1e-10);

  int tick_ctr = 0;
  while (time < 4) {
    for (const std::string& joint_name : robot_status.arm_joint_names()) {
      int idx = robot_status.name_to_position_index().at(joint_name);
      input.mutable_desired_joint_motions().mutable_weights()[idx] = -1;
    }
    for (const std::string& joint_name : robot_status.neck_joint_names()) {
      int idx = robot_status.name_to_position_index().at(joint_name);
      input.mutable_desired_joint_motions().mutable_weights()[idx] = -1;
    }

    // Update desired accelerations.
    input.mutable_desired_body_motions().at("pelvis").mutable_accelerations() =
      pelvis_PDff.ComputeTargetAcceleration(robot_status.pelvis().pose(), robot_status.pelvis().velocity());
    input.mutable_desired_body_motions().at("torso").mutable_accelerations() =
      torso_PDff.ComputeTargetAcceleration(robot_status.torso().pose(), robot_status.torso().velocity());
    input.mutable_desired_joint_motions().mutable_accelerations() =
      joint_PDff.ComputeTargetAcceleration(robot_status.position(), robot_status.velocity());
    input.mutable_desired_centroidal_momentum_change().mutable_momentum_dot().segment<3>(3) =
      (Kp_com.array() * (desired_com - robot_status.com()).array() -
       Kd_com.array() * robot_status.comd().array()).matrix() * robot.getMass();

    std::cout << input;

    int status = con.Control(robot_status, input, &output);

    if (status) {
      throw std::runtime_error("can't solve.");
    }

    // Dummy integration.
    // TODO(siyuan.feng@tri.gloabl): replace this with sys2 simulator when it's
    // ready.
    q += v * dt;
    v += output.vd() * dt;
    time += dt;

    //std::cout << "rs.q: " << robot_status.position().transpose() << std::endl;
    //std::cout << "rs.v: " << robot_status.velocity().transpose() << std::endl;
    robot_status.Update(time, q, v, output.joint_torque(),
                        Eigen::Vector6d::Zero(), Eigen::Vector6d::Zero());
    //std::cout << output;
    tick_ctr++;
  }
  std::cout << "total ticks: " << tick_ctr << std::endl;

  // Check final state.
  // Since the feet have equality constraints set to 0 in the qp controller,
  // they should have no velocity after simulation.
  // Thus, the tolerances on feet velocities are smaller than those for the
  // generalized position and velocity.
  EXPECT_TRUE(robot_status.foot(Side::LEFT).velocity().norm() < 1e-6);
  EXPECT_TRUE(robot_status.foot(Side::RIGHT).velocity().norm() < 1e-6);

  for (int i = 0; i < q.size(); i++) {
    std::cout << "rs.q: " << robot.get_position_name(i) << " " << (q[i]-q_ini[i]) << std::endl;
  }
  EXPECT_TRUE(drake::CompareMatrices(q, q_ini, 1e-4,
                                     drake::MatrixCompareType::absolute));
  EXPECT_TRUE(drake::CompareMatrices(
      v, Eigen::VectorXd::Zero(robot_status.robot().get_num_velocities()), 1e-4,
      drake::MatrixCompareType::absolute));
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

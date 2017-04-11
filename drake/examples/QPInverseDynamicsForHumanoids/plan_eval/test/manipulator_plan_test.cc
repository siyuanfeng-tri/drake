#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/manipulator_move_end_effector_plan.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/manipulator_move_joints_plan.h"

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/test/test_common.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"

#include "robotlocomotion/robot_plan_t.hpp"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

namespace {

// Makes a robotlocomotion::robot_plan_t message, whose keyframes are defined
// by @p times and @p knots.
robotlocomotion::robot_plan_t make_robot_plan_t_message(
    const RigidBodyTree<double>& robot, const std::vector<double>& times,
    const std::vector<MatrixX<double>>& knots) {
  const int dim = robot.get_num_positions();
  robotlocomotion::robot_plan_t msg;
  msg.num_states = static_cast<int>(times.size());
  msg.robot_name = "iiwa";
  msg.plan.resize(msg.num_states);
  msg.plan_info.resize(msg.num_states, 1);
  msg.num_bytes = 0;
  msg.matlab_data.resize(msg.num_bytes);
  msg.num_grasp_transitions = 0;
  msg.grasps.resize(msg.num_grasp_transitions);

  for (int i = 0; i < msg.num_states; ++i) {
    msg.plan[i].num_joints = dim;
    msg.plan[i].joint_name.resize(dim);
    msg.plan[i].joint_position.resize(dim, 0);
    msg.plan[i].joint_velocity.resize(dim, 0);
    msg.plan[i].joint_effort.resize(dim, 0);
    for (int j = 0; j < dim; ++j) {
      msg.plan[i].joint_name[j] = robot.get_position_name(j);
      msg.plan[i].joint_position[j] = knots[i](j, 0);
    }
    msg.plan[i].utime = static_cast<int64_t>(times[i] * 1e6);
  }
  return msg;
}

}  // namespace

class ManipPlanTest : public GenericPlanTest {
 protected:
  void SetUp() override {
    const std::string kModelPath = drake::GetDrakePath() +
        "/manipulation/models/iiwa_description/urdf/"
        "iiwa14_polytope_collision.urdf";

    const std::string kAliasGroupsPath = drake::GetDrakePath() +
        "/examples/QPInverseDynamicsForHumanoids/"
        "config/iiwa.alias_groups";

    const std::string kControlConfigPath = drake::GetDrakePath() +
        "/examples/QPInverseDynamicsForHumanoids/"
        "config/iiwa.id_controller_config";

    std::default_random_engine generator(123);
    AllocateRescourse(kModelPath, kAliasGroupsPath, kControlConfigPath);
    SetRandomConfiguration(generator);

    ee_body_ = alias_groups_->get_body(
        ManipulatorMoveEndEffectorPlan<double>::kEndEffectorAliasGroupName);
  }

  // Checks that @p qp_input matches (the modes and weights are specified in
  // the controller config file):
  // 0. desired dof acceleration matches @p expected_vd
  // 1. desired dof tracking mode should all be ConstraintType::Soft
  // 2. desired dof weights should be 1e-1.
  // 3. contact force basis regularization weight should be 1e-6.
  // 4. there are not desired body motion objectives.
  // 5. there are not contacts.
  // 6. centroidal momentum objectives should be 0 weight,
  // ConstraintType::Skip.
  /*
  void CheckQpInput(const QpInput& qp_input,
                            const VectorX<double>& expected_vd) {
    // Desired generalized acceleration should match expected.
    EXPECT_EQ(qp_input.desired_dof_motions().size(),
              robot_->get_num_positions());

    EXPECT_TRUE(drake::CompareMatrices(
        expected_vd, qp_input.desired_dof_motions().values(), 1e-12,
        drake::MatrixCompareType::absolute));
    for (int i = 0; i < robot_->get_num_positions(); ++i) {
      EXPECT_EQ(qp_input.desired_dof_motions().constraint_type(i),
                ConstraintType::Soft);
      EXPECT_EQ(qp_input.desired_dof_motions().weight(i), 1e-1);
    }

    // Contact force basis regularization weight is irrelevant here since there
    // is not contacts, but its value should match params'.
    EXPECT_EQ(qp_input.w_basis_reg(), 1e-6);

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
  */

  Isometry3<double> get_end_effector_pose() const {
    return robot_status_->robot().CalcBodyPoseInWorldFrame(
        robot_status_->cache(), *ee_body_);
  }

  Vector6<double> get_end_effector_velocity() const {
    return robot_status_->robot().CalcBodySpatialVelocityInWorldFrame(
        robot_status_->cache(), *ee_body_);
  }

  // End effector body pointer.
  const RigidBody<double>* ee_body_;
};

// Test HandlePlanMessageGenericPlanDerived().
// The rest of the virtual functions are empty.
TEST_F(ManipPlanTest, MoveJointsHandleMessageTest) {
  dut_ = std::make_unique<ManipulatorMoveJointsPlan<double>>();
  dut_->Initialize(*robot_status_, *params_, *alias_groups_);

  const int dim = robot_->get_num_positions();
  double time_now = robot_status_->time();

  std::vector<uint8_t> bytes;
  robotlocomotion::robot_plan_t msg;
  // Makes up a plan that starts with the first time stamp being zero, meaning
  // the trajectory should start immediately from the first given knot point,
  // without any smoothing from the current setpoint.
  const int num_steps = 3;
  std::vector<MatrixX<double>> plan_knots(num_steps,
                                          MatrixX<double>::Zero(dim, 1));
  std::vector<double> plan_times(num_steps, 0);
  plan_times[1] += 1;
  plan_times[2] += 3;

  plan_knots[0] << 0.3, -0.1, 1, -0.06, -0.3, 0.6, 0.23;
  plan_knots[1] << -0.3, 0.1, -1, 0.06, 0.3, -0.6, -0.23;
  plan_knots[2] << 0.3, -0.1, 1, -0.06, -0.3, 0.6, 0.23;

  // Gets msg's raw bytes.
  msg = make_robot_plan_t_message(*robot_, plan_times, plan_knots);
  bytes.resize(msg.getEncodedSize());
  msg.encode(bytes.data(), 0, msg.getEncodedSize());
  dut_->HandlePlanMessage(*robot_status_, *params_, *alias_groups_,
                                bytes.data(), bytes.size());

  {
    // Constructs the expected splines.
    std::vector<double> traj_times = plan_times;
    for (size_t i = 0; i < traj_times.size(); ++i) traj_times[i] += time_now;

    PiecewiseCubicTrajectory<double> expected_trajs(
        PiecewisePolynomial<double>::Cubic(traj_times, plan_knots,
                                           MatrixX<double>::Zero(dim, 1),
                                           MatrixX<double>::Zero(dim, 1)));
    // Tolerance is low because the knot points are stored as floats in the
    // lcm message..
    EXPECT_TRUE(
        expected_trajs.is_approx(dut_->get_dof_trajectory(), 1e-7));
  }

  /////////////////////////////////////////////////////////////////////////////
  // Makes another plan that starts with the first time stamp larger than zero,
  // and the current planned position with t = 0 is inserted at the beginning
  // of the desired knot and time pairs.
  plan_times[0] = 0.3;
  msg = make_robot_plan_t_message(*robot_, plan_times, plan_knots);
  bytes.resize(msg.getEncodedSize());
  msg.encode(bytes.data(), 0, msg.getEncodedSize());

  // Moves time by 0.5s to make it more interesting.
  time_now += 0.5;
  robot_status_->UpdateKinematics(
      time_now, robot_status_->position(), robot_status_->velocity());
  dut_->HandlePlanMessage(*robot_status_, *params_, *alias_groups_,
                                bytes.data(), bytes.size());

  {
    VectorX<double> q_d_now =
        dut_->get_dof_trajectory().get_position(time_now);
    std::vector<double> traj_times;
    std::vector<MatrixX<double>> traj_knots;
    traj_times.push_back(time_now);
    traj_knots.push_back(q_d_now);
    for (size_t i = 0; i < plan_times.size(); ++i) {
      traj_times.push_back(time_now + plan_times[i]);
      traj_knots.push_back(plan_knots[i]);
    }

    PiecewiseCubicTrajectory<double> expected_trajs(
        PiecewisePolynomial<double>::Cubic(traj_times, traj_knots,
                                           MatrixX<double>::Zero(dim, 1),
                                           MatrixX<double>::Zero(dim, 1)));
    // Tolerance is low because the knot points are stored as floats in the
    // lcm message..
    EXPECT_TRUE(
        expected_trajs.is_approx(dut_->get_dof_trajectory(), 1e-3));
  }
}

TEST_F(ManipPlanTest, MoveEndEffectorInitializeTest) {
  dut_ = std::make_unique<ManipulatorMoveEndEffectorPlan<double>>();
  dut_->Initialize(*robot_status_, *params_, *alias_groups_);

  // There should be no contacts, 1 tracked body for move end effector plan.
  EXPECT_TRUE(dut_->get_planned_contact_state().empty());
  EXPECT_EQ(dut_->get_body_trajectories().size(), 1);

  // The desired position interpolated at any time should be equal to the
  // current posture.
  // The desired velocity and acceleration should be zero.
  std::vector<double> test_times = {robot_status_->time() - 0.5,
                                    robot_status_->time(),
                                    robot_status_->time() + 3};

  const PiecewiseCartesianTrajectory<double>& ee_traj =
      dut_->get_body_trajectory(ee_body_);
  const Isometry3<double> ee_pose = get_end_effector_pose();

  for (double time : test_times) {
    // Dof trajectory.
    EXPECT_TRUE(
        drake::CompareMatrices(robot_status_->position(),
                               dut_->get_dof_trajectory().get_position(time),
                               1e-12, drake::MatrixCompareType::absolute));

    EXPECT_TRUE(drake::CompareMatrices(
        VectorX<double>::Zero(robot_->get_num_velocities()),
        dut_->get_dof_trajectory().get_velocity(time), 1e-12,
        drake::MatrixCompareType::absolute));

    EXPECT_TRUE(drake::CompareMatrices(
        VectorX<double>::Zero(robot_->get_num_velocities()),
        dut_->get_dof_trajectory().get_acceleration(time), 1e-12,
        drake::MatrixCompareType::absolute));

    // End effector trajectory.
    EXPECT_TRUE(drake::CompareMatrices(ee_pose.matrix(),
                                       ee_traj.get_pose(time).matrix(), 1e-12,
                                       drake::MatrixCompareType::absolute));

    EXPECT_TRUE(drake::CompareMatrices(Vector6<double>::Zero(),
                                       ee_traj.get_velocity(time), 1e-12,
                                       drake::MatrixCompareType::absolute));

    EXPECT_TRUE(drake::CompareMatrices(Vector6<double>::Zero(),
                                       ee_traj.get_acceleration(time), 1e-12,
                                       drake::MatrixCompareType::absolute));
  }
}


}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

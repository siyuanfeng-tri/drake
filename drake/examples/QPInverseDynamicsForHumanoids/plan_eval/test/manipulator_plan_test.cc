#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/manipulator_move_end_effector_plan.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/manipulator_move_joints_plan.h"

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/test/test_common.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"

#include "robotlocomotion/robot_plan_t.hpp"
#include "drake/lcmt_manipulator_plan_move_end_effector.hpp"
#include "drake/util/lcmUtil.h"

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

// Makes a lcmt_manipulator_plan_move_end_effector message, where the waypoints
// are defined by @p times and @p poses.
lcmt_manipulator_plan_move_end_effector make_move_end_effector_message(
    const std::vector<double>& times,
    const std::vector<Isometry3<double>>& poses) {
  lcmt_manipulator_plan_move_end_effector msg;

  msg.num_steps = static_cast<int>(times.size());
  msg.utimes.resize(msg.num_steps);
  msg.poses.resize(msg.num_steps);

  for (int i = 0; i < msg.num_steps; i++) {
    msg.utimes[i] = static_cast<int64_t>(times[i] * 1e6);
    EncodePose(poses[i], msg.poses[i]);
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

    const std::string kAliasGroupsPath =
        drake::GetDrakePath() +
        "/examples/QPInverseDynamicsForHumanoids/"
        "config/iiwa.alias_groups";

    const std::string kControlConfigPath =
        drake::GetDrakePath() +
        "/examples/QPInverseDynamicsForHumanoids/"
        "config/iiwa.id_controller_config";

    std::default_random_engine generator(123);
    AllocateRescourse(kModelPath, kAliasGroupsPath, kControlConfigPath);
    SetRandomConfiguration(generator);

    ee_body_ = alias_groups_->get_body(
        ManipulatorMoveEndEffectorPlan<double>::kEndEffectorAliasGroupName);
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
    EXPECT_TRUE(expected_trajs.is_approx(dut_->get_dof_trajectory(), 1e-7));
  }

  /////////////////////////////////////////////////////////////////////////////
  // Makes another plan that starts with the first time stamp larger than zero.
  // The resulting trajectory should be starting from the q and v interpolated
  // from the current desired trajectory at t0, where t0 is the time when
  // HandlePlanMessage() is called.
  plan_times[0] = 0.3;
  msg = make_robot_plan_t_message(*robot_, plan_times, plan_knots);
  bytes.resize(msg.getEncodedSize());
  msg.encode(bytes.data(), 0, msg.getEncodedSize());

  // Update time.
  time_now += 0.5;
  robot_status_->UpdateKinematics(time_now, robot_status_->position(),
                                  robot_status_->velocity());

  // Get the current desired q and v.
  VectorX<double> q_d_now = dut_->get_dof_trajectory().get_position(time_now);
  VectorX<double> v_d_now = dut_->get_dof_trajectory().get_velocity(time_now);

  // Call handle again.
  dut_->HandlePlanMessage(*robot_status_, *params_, *alias_groups_,
                          bytes.data(), bytes.size());

  {
    // Makes the expected trajectory.
    std::vector<double> traj_times;
    std::vector<MatrixX<double>> traj_knots;
    traj_times.push_back(time_now);
    traj_knots.push_back(q_d_now);
    for (size_t i = 0; i < plan_times.size(); ++i) {
      traj_times.push_back(time_now + plan_times[i]);
      traj_knots.push_back(plan_knots[i]);
    }

    PiecewiseCubicTrajectory<double> expected_trajs(
        PiecewisePolynomial<double>::Cubic(traj_times, traj_knots, v_d_now,
                                           MatrixX<double>::Zero(dim, 1)));
    // Tolerance is low because the knot points are stored as floats in the
    // lcm message..
    EXPECT_TRUE(expected_trajs.is_approx(dut_->get_dof_trajectory(), 1e-7));
  }
}

// Tests Initialization from ManipulatorMoveEndEffectorPlan. Should generate
// a plan that holds at the current posture with one body tracking objective
// for the end effector.
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
  const Isometry3<double> ee_pose =
      robot_status_->robot().CalcBodyPoseInWorldFrame(robot_status_->cache(),
                                                      *ee_body_);

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

// Only testing the body motion objective part, the rest is covered in the
// base case.
TEST_F(ManipPlanTest, TestUpdateQpInput) {
  dut_ = std::make_unique<ManipulatorMoveEndEffectorPlan<double>>();
  dut_->Initialize(*robot_status_, *params_, *alias_groups_);

  // After initialization, desired body motion objective is set to hold
  // the current pose.
  const Isometry3<double> ee_pose_d =
      robot_status_->robot().CalcBodyPoseInWorldFrame(robot_status_->cache(),
                                                      *ee_body_);
  const Vector6<double> ee_vel_d = Vector6<double>::Zero();
  const Vector6<double> ee_acc_d = Vector6<double>::Zero();

  // Changes the current state, and compute acceleration target.
  robot_status_->UpdateKinematics(0.66, robot_status_->position() * 0.3,
                                  robot_status_->velocity());

  QpInput qp_input;
  dut_->UpdateQpInput(*robot_status_, *params_, *alias_groups_, &qp_input);

  // There should be only one body motion tracking objective.
  EXPECT_EQ(qp_input.desired_body_motions().size(), 1);
  const DesiredBodyMotion& ee_motion =
      qp_input.desired_body_motions().at(ee_body_->get_name());

  // Computes the desired acceleration for that body.
  Vector6<double> expected_pose_acc =
      ComputeExpectedBodyAcceleration(ee_body_, ee_pose_d, ee_vel_d, ee_acc_d);

  // Checks body acceleration.
  EXPECT_TRUE(drake::CompareMatrices(expected_pose_acc, ee_motion.values(),
                                     1e-12,
                                     drake::MatrixCompareType::absolute));
  for (int i = 0; i < 6; i++) {
    // Checks body constraint type.
    EXPECT_EQ(ee_motion.constraint_type(i), ConstraintType::Soft);
    // Checks body weight.
    EXPECT_EQ(ee_motion.weight(i), 1);
  }
}

// Tests the message handler for ManipulatorMoveEndEffectorPlan.
TEST_F(ManipPlanTest, MoveEndEffectorHandleMessageTest) {
  dut_ = std::make_unique<ManipulatorMoveEndEffectorPlan<double>>();
  dut_->Initialize(*robot_status_, *params_, *alias_groups_);

  // Makes a copy of the current dof tracking trajectory.
  const PiecewiseCubicTrajectory<double> expected_dof_traj =
      dut_->get_dof_trajectory();

  std::vector<double> plan_times = {0, 2};
  std::vector<Isometry3<double>> plan_poses(plan_times.size(),
                                            Isometry3<double>::Identity());

  std::vector<uint8_t> bytes;
  lcmt_manipulator_plan_move_end_effector msg =
      make_move_end_effector_message(plan_times, plan_poses);
  bytes.resize(msg.getEncodedSize());
  msg.encode(bytes.data(), 0, msg.getEncodedSize());

  // Handles the new plan.
  dut_->HandlePlanMessage(*robot_status_, *params_, *alias_groups_,
                          bytes.data(), bytes.size());
  {
    const PiecewiseCartesianTrajectory<double>& body_traj =
        dut_->get_body_trajectory(ee_body_);
    // The new body trajectory should run from cur_time, to cur_time +
    // plan_times[end]
    EXPECT_EQ(robot_status_->time(),
              body_traj.get_position_trajectory().get_start_time());
    EXPECT_EQ(robot_status_->time() + plan_times.back(),
              body_traj.get_position_trajectory().get_end_time());

    // There should be no contacts, but 1 tracked body.
    EXPECT_TRUE(dut_->get_planned_contact_state().empty());
    EXPECT_EQ(dut_->get_body_trajectories().size(), 1);
    // The dof trajectory should not change.
    EXPECT_TRUE(expected_dof_traj.is_approx(dut_->get_dof_trajectory(), 1e-12));

    // Constructs the expected body traj.
    std::vector<double> traj_times = plan_times;
    std::vector<Isometry3<double>> traj_poses = plan_poses;

    for (size_t i = 0; i < traj_times.size(); ++i) {
      traj_times[i] += robot_status_->time();
    }

    const Vector3<double> zero = Vector3<double>::Zero();
    PiecewiseCartesianTrajectory<double> expected_traj =
        PiecewiseCartesianTrajectory<
            double>::MakeCubicLinearWithEndLinearVelocity(traj_times,
                                                          traj_poses, zero,
                                                          zero);

    EXPECT_TRUE(
        expected_traj.is_approx(dut_->get_body_trajectory(ee_body_), 1e-12));
  }

  /////////////////////////////////////////////////////////////////////////////
  // Makes a different plan that starts with a non zero first time stamp. The
  // resulting trajectory should ramp from the pose and velocity interpolated
  // from the current desired trajectory at t0 to the first pose in the
  // message, where t0 is the time when HandlePlanMessage() is called.
  plan_times[0] = 0.8;
  msg = make_move_end_effector_message(plan_times, plan_poses);
  bytes.resize(msg.getEncodedSize());
  msg.encode(bytes.data(), 0, msg.getEncodedSize());

  // Moves the clock forward in time.
  robot_status_->UpdateKinematics(robot_status_->time() + 1,
                                  robot_status_->position(),
                                  robot_status_->velocity());

  // Gets the pose and velocity interpolated from the current desired
  // trajectory.
  Isometry3<double> ee_pose_d_now =
      dut_->get_body_trajectory(ee_body_).get_pose(robot_status_->time());
  Vector6<double> ee_vel_d_now =
      dut_->get_body_trajectory(ee_body_).get_velocity(robot_status_->time());

  // Handles the new plan.
  dut_->HandlePlanMessage(*robot_status_, *params_, *alias_groups_,
                          bytes.data(), bytes.size());
  {
    const PiecewiseCartesianTrajectory<double>& body_traj =
        dut_->get_body_trajectory(ee_body_);

    // There should be no contacts, 1 tracked body.
    EXPECT_TRUE(dut_->get_planned_contact_state().empty());
    EXPECT_EQ(dut_->get_body_trajectories().size(), 1);
    // The dof trajectory should not change.
    EXPECT_TRUE(expected_dof_traj.is_approx(dut_->get_dof_trajectory(), 1e-12));

    // The new body trajectory should run from cur_time, to cur_time +
    // plan_times[end]
    EXPECT_EQ(robot_status_->time(),
              body_traj.get_position_trajectory().get_start_time());
    EXPECT_EQ(robot_status_->time() + plan_times.back(),
              body_traj.get_position_trajectory().get_end_time());

    // Constructs the expected body traj.
    std::vector<double> traj_times;
    std::vector<Isometry3<double>> traj_poses;

    traj_times.push_back(robot_status_->time());
    traj_poses.push_back(ee_pose_d_now);

    for (size_t i = 0; i < plan_times.size(); ++i) {
      traj_times.push_back(plan_times[i] + robot_status_->time());
      traj_poses.push_back(plan_poses[i]);
    }

    PiecewiseCartesianTrajectory<double> expected_traj =
        PiecewiseCartesianTrajectory<double>::
            MakeCubicLinearWithEndLinearVelocity(traj_times, traj_poses,
                                                 ee_vel_d_now.tail<3>(),
                                                 Vector3<double>::Zero());

    EXPECT_TRUE(
        expected_traj.is_approx(dut_->get_body_trajectory(ee_body_), 1e-12));
  }
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

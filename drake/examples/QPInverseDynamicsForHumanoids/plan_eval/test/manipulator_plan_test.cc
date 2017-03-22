#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/test/test_common.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/manipulator_move_end_effector_plan.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/manipulator_move_joints_plan.h"

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"

#include "drake/lcmt_manipulator_plan_move_end_effector.hpp"
#include "drake/util/lcmUtil.h"
#include "robotlocomotion/robot_plan_t.hpp"


namespace drake {
namespace examples {
namespace qp_inverse_dynamics {
namespace {

class ManipMoveJointsPlanTest : public GenericPlanTest {
 protected:
  void SetUp() override {
    const std::string kModelPath = drake::GetDrakePath() +
        "/examples/kuka_iiwa_arm/models/iiwa14/"
        "iiwa14_simplified_collision.urdf";

    const std::string kAliasGroupsPath = drake::GetDrakePath() +
        "/examples/QPInverseDynamicsForHumanoids/"
        "config/iiwa_for_test.alias_groups";

    const std::string kControlConfigPath = drake::GetDrakePath() +
        "/examples/QPInverseDynamicsForHumanoids/"
        "config/iiwa_for_test.id_controller_config";

    robot_ = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        kModelPath, multibody::joints::kFixed, robot_.get());

    dut_ = std::unique_ptr<GenericPlan<double>>(
        new ManipulatorMoveJointsPlan<double>());

    Initialize(kAliasGroupsPath, kControlConfigPath);
  }
};

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

TEST_F(ManipMoveJointsPlanTest, MoveJointsHandleMessageTest) {
  const int dim = robot_->get_num_positions();
  double time_now = robot_status_->time();

  std::vector<uint8_t> bytes;
  robotlocomotion::robot_plan_t msg;
  // Makes up a plan that starts with the first time stamp being zero, meaning
  // the trajectory should start immediately from the first given knot point,
  // without any smoothing from the current setpoint.
  std::vector<double> plan_times = {0, 1, 2};
  std::vector<MatrixX<double>> plan_knots(plan_times.size());
  std::default_random_engine generator(456);
  for (size_t i = 0; i < plan_knots.size(); ++i) {
    plan_knots[i] = robot_->getRandomConfiguration(generator);
  }

  // Gets msg's raw bytes.
  msg = make_robot_plan_t_message(*robot_, plan_times, plan_knots);
  bytes.resize(msg.getEncodedSize());
  msg.encode(bytes.data(), 0, msg.getEncodedSize());

  // Handles a new plan.
  dut_->HandlePlanMessage(*robot_status_, *params_, *alias_groups_,
                          bytes.data(), bytes.size());

  // There shouldn't be any body trajectories.
  EXPECT_TRUE(dut_->get_body_trajectories().empty());

  // The contact state should not change.
  EXPECT_TRUE(dut_->get_contact_state().empty());

  {
    // Constructs the expected splines.
    std::vector<double> traj_times = plan_times;
    for (size_t i = 0; i < traj_times.size(); ++i) {
      traj_times[i] += time_now;
    }

    PiecewiseCubicTrajectory<double> expected_trajs(
        PiecewisePolynomial<double>::Cubic(traj_times, plan_knots,
                                           MatrixX<double>::Zero(dim, 1),
                                           MatrixX<double>::Zero(dim, 1)));
    // Tolerance is low because the lcm message's joint angles are stored as
    // floats.
    EXPECT_TRUE(
        expected_trajs.is_approx(dut_->get_dof_trajectory(), 1e-5));

    // The new body trajectory should run from cur_time, to cur_time + plan_times[end]
    EXPECT_EQ(robot_status_->time(), dut_->get_dof_trajectory().get_start_time());
    EXPECT_EQ(robot_status_->time() + plan_times.back(), dut_->get_dof_trajectory().get_end_time());
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

  VectorX<double> q_d_now =
      dut_->get_dof_trajectory().get_position(time_now);

  // Handles a new plan.
  dut_->HandlePlanMessage(*robot_status_, *params_, *alias_groups_,
                         bytes.data(), bytes.size());

  // There shouldn't be any body trajectories.
  EXPECT_TRUE(dut_->get_body_trajectories().empty());

  // The contact state should not change.
  EXPECT_TRUE(dut_->get_contact_state().empty());
  {
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
    // Tolerance is low because the lcm message's joint angles are stored as
    // floats.
    EXPECT_TRUE(
        expected_trajs.is_approx(dut_->get_dof_trajectory(), 1e-5));

    // The new body trajectory should run from cur_time, to cur_time + plan_times[end]
    EXPECT_EQ(robot_status_->time(), dut_->get_dof_trajectory().get_start_time());
    EXPECT_EQ(robot_status_->time() + plan_times.back(), dut_->get_dof_trajectory().get_end_time());
  }
}

class ManipMoveEndEffectorPlanTest : public GenericPlanTest {
 protected:
  void SetUp() override {
    const std::string kModelPath = drake::GetDrakePath() +
        "/examples/kuka_iiwa_arm/models/iiwa14/"
        "iiwa14_simplified_collision.urdf";

    const std::string kAliasGroupsPath = drake::GetDrakePath() +
        "/examples/QPInverseDynamicsForHumanoids/"
        "config/iiwa_for_test.alias_groups";

    const std::string kControlConfigPath = drake::GetDrakePath() +
        "/examples/QPInverseDynamicsForHumanoids/"
        "config/iiwa_for_test.id_controller_config";

    robot_ = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        kModelPath, multibody::joints::kFixed, robot_.get());

    dut_ = std::unique_ptr<GenericPlan<double>>(
        new ManipulatorMoveEndEffectorPlan<double>());

    Initialize(kAliasGroupsPath, kControlConfigPath);

    ee_body_ = alias_groups_->get_body(ManipulatorMoveEndEffectorPlan<double>::kEndEffectorAliasGroupName);
  }

  Isometry3<double> get_end_effector_pose() const {
    return robot_status_->robot().CalcBodyPoseInWorldFrame(robot_status_->cache(), *ee_body_);
  }

  Vector6<double> get_end_effector_velocity() const {
    return robot_status_->robot().CalcBodySpatialVelocityInWorldFrame(robot_status_->cache(), *ee_body_);
  }

  // End effector body pointer.
  const RigidBody<double>* ee_body_;
};

TEST_F(ManipMoveEndEffectorPlanTest, MoveEndEffectorInitializeTest) {
  // There should be no contacts, 1 tracked body for move end effector plan.
  EXPECT_TRUE(dut_->get_contact_state().empty());
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
    EXPECT_TRUE(drake::CompareMatrices(
        robot_status_->position(), dut_->get_dof_trajectory().get_position(time),
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
    EXPECT_TRUE(drake::CompareMatrices(
        ee_pose.matrix(),
        ee_traj.get_pose(time).matrix(), 1e-12,
        drake::MatrixCompareType::absolute));

    EXPECT_TRUE(drake::CompareMatrices(
        Vector6<double>::Zero(),
        ee_traj.get_velocity(time), 1e-12,
        drake::MatrixCompareType::absolute));

    EXPECT_TRUE(drake::CompareMatrices(
        Vector6<double>::Zero(),
        ee_traj.get_acceleration(time), 1e-12,
        drake::MatrixCompareType::absolute));
  }
}

TEST_F(ManipMoveEndEffectorPlanTest, TestUpdateQpInput) {
  QpInput qp_input;

  VectorX<double> q_d = robot_status_->position();

  const Isometry3<double> ee_pose_d = get_end_effector_pose();
  const Vector6<double> ee_vel_d = Vector6<double>::Zero();
  const Vector6<double> ee_acc_d = Vector6<double>::Zero();

  // Changes the current state.
  robot_status_->UpdateKinematics(0.66, robot_status_->position() * 0.3, robot_status_->velocity());
  dut_->UpdateQpInput(*robot_status_, *params_, *alias_groups_, &qp_input);

  // The expected dof acceleration should only contain the position and
  // velocity term.
  VectorX<double> kp, kd;
  params_->LookupDesiredDofMotionGains(&kp, &kd);

  VectorX<double> expected_vd =
      (kp.array() * (q_d - robot_status_->position()).array() -
       kd.array() * robot_status_->velocity().array())
          .matrix();

  // The weights / constraint types are hard coded in this test. They need to
  // match the numbers specified in
  EXPECT_TRUE(drake::CompareMatrices(
      expected_vd, qp_input.desired_dof_motions().values(), 1e-12,
      drake::MatrixCompareType::absolute));
  for (int i = 0; i < robot_->get_num_positions(); ++i) {
    // Checks dof constraint type.
    EXPECT_EQ(qp_input.desired_dof_motions().constraint_type(i),
              ConstraintType::Soft);
    // Checks dof weight.
    EXPECT_EQ(qp_input.desired_dof_motions().weight(i), 1e-1);
  }

  // Contact force basis regularization weight is irrelevant here since there
  // is not contacts, but its value should match params'.
  EXPECT_EQ(qp_input.w_basis_reg(), 1e-6);

  // There should be only one body motion tracking objective.
  EXPECT_EQ(qp_input.desired_body_motions().size(), 1);
  const DesiredBodyMotion& ee_motion =
      qp_input.desired_body_motions().at(ee_body_->get_name());

  // Computes current pose and velocity of the body.
  const Isometry3<double> ee_pose = get_end_effector_pose();
  const Vector6<double> ee_vel = get_end_effector_velocity();

  // Computes the desired acceleration for that body.
  Vector6<double> ee_kp, ee_kd;
  params_->LookupDesiredBodyMotionGains(*ee_body_, &ee_kp, &ee_kd);
  CartesianSetpoint<double> tracker(ee_pose_d, ee_vel_d, ee_acc_d, ee_kp, ee_kd);
  Vector6<double> expected_pose_acc = tracker.ComputeTargetAcceleration(ee_pose, ee_vel);

  // Checks body acceleration.
  EXPECT_TRUE(drake::CompareMatrices(
      expected_pose_acc, ee_motion.values(), 1e-12,
      drake::MatrixCompareType::absolute));
  for (int i = 0; i < 6; i++) {
    // Checks body constraint type.
    EXPECT_EQ(ee_motion.constraint_type(i), ConstraintType::Soft);
    // Checks body weight.
    EXPECT_EQ(ee_motion.weight(i), 1);
  }

  // No contacts.
  EXPECT_TRUE(qp_input.contact_information().empty());

  // No center of mass or angular momentum objective.
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(qp_input.desired_centroidal_momentum_dot().value(i), 0);
    EXPECT_EQ(qp_input.desired_centroidal_momentum_dot().weight(i), 0);
    EXPECT_EQ(qp_input.desired_centroidal_momentum_dot().constraint_type(i),
              ConstraintType::Skip);
  }
}

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


TEST_F(ManipMoveEndEffectorPlanTest, MoveEndEffectorHandleMessageTest) {
  // Makes a copy of the current dof tracking trajectory.
  const PiecewiseCubicTrajectory<double> expected_dof_traj =
      dut_->get_dof_trajectory();

  std::vector<double> plan_times = {0, 2};
  std::vector<Isometry3<double>> plan_poses(plan_times.size(), Isometry3<double>::Identity());

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
    // The new body trajectory should run from cur_time, to cur_time + plan_times[end]
    EXPECT_EQ(robot_status_->time(), body_traj.get_position_trajectory().get_start_time());
    EXPECT_EQ(robot_status_->time() + plan_times.back(), body_traj.get_position_trajectory().get_end_time());

    // There should be no contacts, 1 tracked body.
    EXPECT_TRUE(dut_->get_contact_state().empty());
    EXPECT_EQ(dut_->get_body_trajectories().size(), 1);
    // The dof trajectory should not change.
    EXPECT_EQ(expected_dof_traj, dut_->get_dof_trajectory());

    // Constructs the expected body traj.
    std::vector<double> traj_times = plan_times;
    std::vector<Isometry3<double>> traj_poses = plan_poses;

    for (size_t i = 0; i < traj_times.size(); ++i) {
      traj_times[i] += robot_status_->time();
    }

    PiecewiseCartesianTrajectory<double> expected_traj =
        PiecewiseCartesianTrajectory<double>::MakeCubicLinearWithZeroEndVelocity(traj_times, traj_poses);

    EXPECT_TRUE(
        expected_traj.is_approx(dut_->get_body_trajectory(ee_body_), 1e-12));
  }

  // Makes a different plan that starts with ramping.
  plan_times[0] = 0.8;
  msg = make_move_end_effector_message(plan_times, plan_poses);
  bytes.resize(msg.getEncodedSize());
  msg.encode(bytes.data(), 0, msg.getEncodedSize());

  // Moves the clock forward in time.
  robot_status_->UpdateKinematics(
      robot_status_->time() + 1, robot_status_->position(), robot_status_->velocity());

  Isometry3<double> ee_pose_d_now = dut_->get_body_trajectory(ee_body_).get_pose(robot_status_->time());

  // Handles the new plan.
  dut_->HandlePlanMessage(*robot_status_, *params_, *alias_groups_,
                         bytes.data(), bytes.size());
  {
    const PiecewiseCartesianTrajectory<double>& body_traj =
        dut_->get_body_trajectory(ee_body_);

    // There should be no contacts, 1 tracked body.
    EXPECT_TRUE(dut_->get_contact_state().empty());
    EXPECT_EQ(dut_->get_body_trajectories().size(), 1);
    // The dof trajectory should not change.
    EXPECT_EQ(expected_dof_traj, dut_->get_dof_trajectory());

    // The new body trajectory should run from cur_time, to cur_time + plan_times[end]
    EXPECT_EQ(robot_status_->time(), body_traj.get_position_trajectory().get_start_time());
    EXPECT_EQ(robot_status_->time() + plan_times.back(), body_traj.get_position_trajectory().get_end_time());

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
        PiecewiseCartesianTrajectory<double>::MakeCubicLinearWithZeroEndVelocity(traj_times, traj_poses);

    EXPECT_TRUE(
        expected_traj.is_approx(dut_->get_body_trajectory(ee_body_), 1e-12));
  }
}

}  // namespace
}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

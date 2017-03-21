#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/manipulator_move_end_effector_plan.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/manipulator_move_joints_plan.h"

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"

#include "robotlocomotion/robot_plan_t.hpp"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

class ManipPlanTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string kModelPath = drake::GetDrakePath() +
        "/examples/kuka_iiwa_arm/models/iiwa14/"
        "iiwa14_simplified_collision.urdf";

    const std::string kAliasGroupsPath = drake::GetDrakePath() +
        "/examples/QPInverseDynamicsForHumanoids/"
        "config/iiwa.alias_groups";

    const std::string kControlConfigPath = drake::GetDrakePath() +
        "/examples/QPInverseDynamicsForHumanoids/"
        "config/iiwa.id_controller_config";

    robot_ = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        kModelPath, multibody::joints::kFixed, robot_.get());

    alias_groups_ =
        std::make_unique<param_parsers::RigidBodyTreeAliasGroups<double>>(
            *robot_);
    alias_groups_->LoadFromFile(kAliasGroupsPath);

    params_ = std::make_unique<param_parsers::ParamSet>();
    params_->LoadFromFile(kControlConfigPath, *alias_groups_);

    robot_status_ = std::make_unique<HumanoidStatus>(*robot_, *alias_groups_);
    double time_now = 0.2;
    VectorX<double> q(robot_->get_num_positions());
    VectorX<double> v(robot_->get_num_velocities());
    q << 0.3, 0.5, -0.2, 1, 0.99, -0.55, 0.233;
    v << -1, -3, 2.4, 0.66, 0.77, 0.456, -0.237;
    robot_status_->UpdateKinematics(time_now, q, v);
  }

  void TestMoveJointQpInput(const QpInput& qp_input,
                            const VectorX<double>& expected_vd) {
    // Desired generalized acceleration should match expected.
    EXPECT_EQ(qp_input.desired_dof_motions().size(),
              robot_->get_num_positions());

    EXPECT_TRUE(drake::CompareMatrices(
        expected_vd, qp_input.desired_dof_motions().values(), 1e-12,
        drake::MatrixCompareType::absolute));
    VectorX<double> expected_weights =
        params_->MakeDesiredDofMotions().weights();
    EXPECT_TRUE(drake::CompareMatrices(
        expected_weights, qp_input.desired_dof_motions().weights(), 1e-12,
        drake::MatrixCompareType::absolute));
    for (int i = 0; i < robot_->get_num_positions(); ++i) {
      EXPECT_EQ(qp_input.desired_dof_motions().constraint_type(i),
                ConstraintType::Soft);
    }

    // Contact force basis regularization weight is irrelevant here since there
    // is not contacts, but its value should match params'.
    EXPECT_EQ(qp_input.w_basis_reg(),
              params_->get_basis_regularization_weight());

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

  std::unique_ptr<RigidBodyTree<double>> robot_;
  std::unique_ptr<param_parsers::RigidBodyTreeAliasGroups<double>>
      alias_groups_;
  std::unique_ptr<param_parsers::ParamSet> params_;
  std::unique_ptr<HumanoidStatus> robot_status_;
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

TEST_F(ManipPlanTest, MoveJointsInitTest) {
  // The plan is set to hold the current posture.
  ManipulatorMoveJointsPlan<double> move_joints;
  move_joints.Initialize(*robot_status_, *params_, *alias_groups_);

  // There should be no contacts, no tracked bodies for move joints plan.
  EXPECT_TRUE(move_joints.get_contact_state().empty());
  EXPECT_TRUE(move_joints.get_body_trajectories().empty());

  // The desired position interpolated at any time should be equal to the current posture.
  // The desired velocity and acceleration should be zero.
  std::vector<double> test_times = {
      robot_status_->time() - 0.5,
      robot_status_->time(),
      robot_status_->time() + 3};

  for (double time : test_times) {
    EXPECT_TRUE(drake::CompareMatrices(
          robot_status_->position(),
          move_joints.get_dof_trajectory().get_position(time),
          1e-12, drake::MatrixCompareType::absolute));

    EXPECT_TRUE(drake::CompareMatrices(
          VectorX<double>::Zero(robot_->get_num_velocities()),
          move_joints.get_dof_trajectory().get_velocity(time),
          1e-12, drake::MatrixCompareType::absolute));

    EXPECT_TRUE(drake::CompareMatrices(
          VectorX<double>::Zero(robot_->get_num_velocities()),
          move_joints.get_dof_trajectory().get_acceleration(time),
          1e-12, drake::MatrixCompareType::absolute));
  }


  // The expected dof acceleration should be pure damping over the current
  // velocity.
  VectorX<double> kp, kd;
  params_->LookupDesiredDofMotionGains(&kp, &kd);
  VectorX<double> expected_vd =
       (-kd.array() * robot_status_->velocity().array()).matrix();

  QpInput qp_input;
  move_joints.UpdateQpInput(*robot_status_, *params_, *alias_groups_, &qp_input);
  TestMoveJointQpInput(qp_input, expected_vd);
}

TEST_F(ManipPlanTest, MoveJointsHandleMessageTest) {
  ManipulatorMoveJointsPlan<double> move_joints;
  move_joints.Initialize(*robot_status_, *params_, *alias_groups_);

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
  move_joints.HandlePlanMessage(*robot_status_, *params_, *alias_groups_,
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
        expected_trajs.is_approx(move_joints.get_dof_trajectory(), 1e-7));
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
  move_joints.HandlePlanMessage(*robot_status_, *params_, *alias_groups_,
                                bytes.data(), bytes.size());

  {
    VectorX<double> q_d_now =
        move_joints.get_dof_trajectory().get_position(time_now);
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
        expected_trajs.is_approx(move_joints.get_dof_trajectory(), 1e-7));
  }

  /////////////////////////////////////////////////////////////////////////////
  // Checks QpInput at t = 1.5.
  time_now = 1.5;
  robot_status_->UpdateKinematics(
      time_now, robot_status_->position(), robot_status_->velocity());

  // Computes from the plan.
  QpInput qp_input;
  move_joints.UpdateQpInput(*robot_status_, *params_, *alias_groups_, &qp_input);

  // Computes expected dof accelerations computed by hand:
  VectorX<double> q_d = move_joints.get_dof_trajectory().get_position(robot_status_->time());
  VectorX<double> v_d = move_joints.get_dof_trajectory().get_velocity(robot_status_->time());
  VectorX<double> vd_d = move_joints.get_dof_trajectory().get_acceleration(robot_status_->time());
  VectorX<double> kp, kd;
  params_->LookupDesiredDofMotionGains(&kp, &kd);

  VectorX<double> expected_vd =
      (kp.array() * (q_d - robot_status_->position()).array() +
       kd.array() * (v_d - robot_status_->velocity()).array() +
       vd_d.array()).matrix();

  TestMoveJointQpInput(qp_input, expected_vd);
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

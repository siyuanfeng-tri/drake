#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/generic_plan.h"

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

// This is a derived class from GenericPlan with no additional features.
// It sets up a plan that does not have any Cartesian tracking objectives,
// no contacts, and holds the current generalized position for ever.
template <typename T>
class GenericTestPlan : public GenericPlan<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GenericTestPlan<T>)

  GenericTestPlan() {}

 private:
  GenericPlan<T>* CloneGenericPlanDerived() const {
    return new GenericTestPlan<T>();
  }

  void InitializeGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {}

  void ExecutePlanGenericPlanDerived(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {}

  void HandlePlanMessageGenericPlanDerived(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      const void* message_bytes, int message_length) {}

  void UpdateQpInputGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      QpInput* qp_input) const {}
};

class GenericPlanTest : public ::testing::Test {
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

  std::unique_ptr<RigidBodyTree<double>> robot_;
  std::unique_ptr<param_parsers::RigidBodyTreeAliasGroups<double>>
      alias_groups_;
  std::unique_ptr<param_parsers::ParamSet> params_;
  std::unique_ptr<HumanoidStatus> robot_status_;
};

TEST_F(GenericPlanTest, TestInitialize) {
  // The plan is set to hold the current posture.
  GenericTestPlan<double> plan;
  plan.Initialize(*robot_status_, *params_, *alias_groups_);

  // There should be no contacts, no tracked bodies for move joints plan.
  EXPECT_TRUE(plan.get_contact_state().empty());
  EXPECT_TRUE(plan.get_body_trajectories().empty());

  // The desired position interpolated at any time should be equal to the current posture.
  // The desired velocity and acceleration should be zero.
  std::vector<double> test_times = {
      robot_status_->time() - 0.5,
      robot_status_->time(),
      robot_status_->time() + 3};

  for (double time : test_times) {
    EXPECT_TRUE(drake::CompareMatrices(
          robot_status_->position(),
          plan.get_dof_trajectory().get_position(time),
          1e-12, drake::MatrixCompareType::absolute));

    EXPECT_TRUE(drake::CompareMatrices(
          VectorX<double>::Zero(robot_->get_num_velocities()),
          plan.get_dof_trajectory().get_velocity(time),
          1e-12, drake::MatrixCompareType::absolute));

    EXPECT_TRUE(drake::CompareMatrices(
          VectorX<double>::Zero(robot_->get_num_velocities()),
          plan.get_dof_trajectory().get_acceleration(time),
          1e-12, drake::MatrixCompareType::absolute));
  }
}

TEST_F(GenericPlanTest, TestClone) {
  GenericTestPlan<double> plan;
  plan.Initialize(*robot_status_, *params_, *alias_groups_);

  std::unique_ptr<GenericPlan<double>> clone = plan.Clone();
  EXPECT_TRUE(plan.get_contact_state() == clone->get_contact_state());
  EXPECT_TRUE(plan.get_body_trajectories() == clone->get_body_trajectories());
  EXPECT_TRUE(plan.get_dof_trajectory() == clone->get_dof_trajectory());
}

TEST_F(GenericPlanTest, TestUpdateQpInput) {
  // The plan is set to hold the current posture.
  GenericTestPlan<double> plan;
  plan.Initialize(*robot_status_, *params_, *alias_groups_);

  // The expected dof acceleration should only contain the position and
  // velocity term.
  VectorX<double> kp, kd;
  params_->LookupDesiredDofMotionGains(&kp, &kd);

  VectorX<double> q_d = robot_status_->position();
  robot_status_->UpdateKinematics(0.66, q_d * 0.3, robot_status_->velocity());

  VectorX<double> expected_vd =
       (kp.array() * (q_d - robot_status_->position()).array() -
        kd.array() * robot_status_->velocity().array()).matrix();

  QpInput qp_input;
  plan.UpdateQpInput(*robot_status_, *params_, *alias_groups_, &qp_input);

  // Desired generalized acceleration should match expected.
  EXPECT_EQ(qp_input.desired_dof_motions().size(),
      robot_->get_num_positions());

  EXPECT_TRUE(drake::CompareMatrices(
        expected_vd, qp_input.desired_dof_motions().values(), 1e-12,
        drake::MatrixCompareType::absolute));
  VectorX<double> expected_weights = params_->MakeDesiredDofMotions().weights();
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

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

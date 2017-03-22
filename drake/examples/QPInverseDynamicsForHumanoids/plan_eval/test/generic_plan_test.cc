#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/test/test_common.h"

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
class DummyPlan : public GenericPlan<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummyPlan<T>)

  DummyPlan() {}

 private:
  GenericPlan<T>* CloneGenericPlanDerived() const {
    return new DummyPlan<T>();
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

class DummyPlanTest : public GenericPlanTest {
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
        new DummyPlan<double>());

    Initialize(kAliasGroupsPath, kControlConfigPath);
  }
};

TEST_F(DummyPlanTest, TestInitialize) {
  // There should be no contacts, no tracked bodies for move joints plan.
  EXPECT_TRUE(dut_->get_contact_state().empty());
  EXPECT_TRUE(dut_->get_body_trajectories().empty());

  // The desired position interpolated at any time should be equal to the
  // current posture.
  // The desired velocity and acceleration should be zero.
  std::vector<double> test_times = {robot_status_->time() - 0.5,
                                    robot_status_->time(),
                                    robot_status_->time() + 3};

  for (double time : test_times) {
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
  }
}

// Tests the cloned fields are the same.
TEST_F(DummyPlanTest, TestClone) {
  std::unique_ptr<GenericPlan<double>> clone = dut_->Clone();
  EXPECT_TRUE(dut_->get_contact_state() == clone->get_contact_state());
  EXPECT_TRUE(dut_->get_body_trajectories() == clone->get_body_trajectories());
  EXPECT_TRUE(dut_->get_dof_trajectory() == clone->get_dof_trajectory());
}

// Checks the generated QpInput vs expected.
TEST_F(DummyPlanTest, TestUpdateQpInput) {
  QpInput qp_input;

  VectorX<double> q_d = robot_status_->position();

  // Changes the current state.
  robot_status_->UpdateKinematics(
      0.66, 0.3 * robot_status_->position(), robot_status_->velocity());

  // Computes QpInput.
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
  // "config/iiwa_for_test.id_controller_config".
  // Checks dof acceleration.
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

  // Checks force basis regularization weight.
  EXPECT_EQ(qp_input.w_basis_reg(), 1e-6);

  // No body motion objectives.
  EXPECT_TRUE(qp_input.desired_body_motions().empty());

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

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

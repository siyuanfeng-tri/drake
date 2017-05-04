#include "drake/manipulation/util/robot_state_msg_translator.h"
#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/math/quaternion.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree_construction.h"

namespace drake {
namespace manipulation {

namespace {
constexpr int kNumTests = 100;
}  // namespace

class RobotStateLcmMessageTranslatorTest : public ::testing::Test {
 protected:
  void SetUp() override { rand_.seed(1234); }

  void Initialize(const std::string& model,
                  multibody::joints::FloatingBaseType floating_base_type) {
    base_type_ = floating_base_type;
    robot_ = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        model, floating_base_type, nullptr /* weld to frame */, robot_.get());

    dut_ = std::make_unique<RobotStateLcmMessageTranslator>(*robot_);
    dut_->ResizeMessage(&message_);
  }

  void RunEncodeDecode() {
    // Low tolerance because bot_core::robot_state_t uses float for storage.
    double tolerance = 10. * std::numeric_limits<float>::epsilon();

    std::normal_distribution<double> distribution;
    VectorX<double> q, v, q_expected, v_expected;
    v.resize(robot_->get_num_velocities());

    for (int i = 0; i < kNumTests; ++i) {
      q = robot_->getRandomConfiguration(rand_);
      for (int i = 0; i < robot_->get_num_velocities(); i++) {
        v[i] = distribution(rand_);
      }

      dut_->EncodeMessageKinematics(q, v, &message_);
      dut_->DecodeMessageKinematics(message_, &q_expected, &v_expected);

      if (base_type_ == multibody::joints::FloatingBaseType::kFixed ||
          base_type_ == multibody::joints::FloatingBaseType::kRollPitchYaw) {
        EXPECT_TRUE(CompareMatrices(q_expected, q, tolerance,
                                    MatrixCompareType::absolute));
      } else if (base_type_ ==
                 multibody::joints::FloatingBaseType::kQuaternion) {
        // Check for -quat = quat.
        VectorX<double> q_expected_w_neg_quat = q_expected;
        q_expected_w_neg_quat.segment<4>(3) *= -1;
        bool equal = CompareMatrices(q_expected, q, tolerance,
                                     MatrixCompareType::absolute) ||
                     CompareMatrices(q_expected_w_neg_quat, q, tolerance,
                                     MatrixCompareType::absolute);
        EXPECT_TRUE(equal);
      }

      EXPECT_TRUE(CompareMatrices(v_expected, v, tolerance,
                                  MatrixCompareType::absolute));
    }
  }

  multibody::joints::FloatingBaseType base_type_;
  bot_core::robot_state_t message_;
  // Same seed every time, but that's OK.
  std::default_random_engine rand_;
  std::unique_ptr<RobotStateLcmMessageTranslator> dut_;
  std::unique_ptr<RigidBodyTree<double>> robot_;
};

TEST_F(RobotStateLcmMessageTranslatorTest, TestEncodeDecodeFixedBase) {
  std::vector<std::string> models = {
      drake::GetDrakePath() +
          "/manipulation/models/iiwa_description/urdf/"
          "iiwa14_polytope_collision.urdf",
      drake::GetDrakePath() +
          "/examples/Valkyrie/urdf/urdf/"
          "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf"};

  for (const auto& model : models) {
    Initialize(model, multibody::joints::FloatingBaseType::kFixed);
    RunEncodeDecode();
  }
}

TEST_F(RobotStateLcmMessageTranslatorTest, TestEncodeDecodeRPY) {
  std::vector<std::string> models = {
      drake::GetDrakePath() +
          "/manipulation/models/iiwa_description/urdf/"
          "iiwa14_polytope_collision.urdf",
      drake::GetDrakePath() +
          "/examples/Valkyrie/urdf/urdf/"
          "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf"};

  for (const auto& model : models) {
    Initialize(model, multibody::joints::FloatingBaseType::kRollPitchYaw);
    RunEncodeDecode();
  }
}

TEST_F(RobotStateLcmMessageTranslatorTest, TestEncodeDecodeQuat) {
  std::vector<std::string> models = {
      drake::GetDrakePath() +
          "/manipulation/models/iiwa_description/urdf/"
          "iiwa14_polytope_collision.urdf",
      drake::GetDrakePath() +
          "/examples/Valkyrie/urdf/urdf/"
          "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf"};

  for (const auto& model : models) {
    Initialize(model, multibody::joints::FloatingBaseType::kQuaternion);
    RunEncodeDecode();
  }
}

}  // namespace manipulation
}  // namespace drake

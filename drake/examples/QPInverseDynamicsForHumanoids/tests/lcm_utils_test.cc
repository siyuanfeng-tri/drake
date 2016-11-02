#include <cstdlib>

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

namespace {

static std::string urdf =
      drake::GetDrakePath() +
      std::string(
          "/examples/Valkyrie/urdf/urdf/"
          "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf");
static RigidBodyTree robot(urdf, drake::systems::plants::joints::kRollPitchYaw);

// Resize val to the given dimension, then set value and weight to alternating
// positive and negative values, finally set the constraint type based on the
// weight's sign.
static void SetTestConstrainedValues(ConstrainedValues* val, int dim) {
  if (!val) return;
  val->mutable_values() = Eigen::VectorXd::Random(dim);
  val->mutable_weights() = Eigen::VectorXd::Random(dim);
  val->mutable_constraint_types().resize(dim);
  for (int i = 0; i < dim; ++i) {
    val->mutable_value(i) = i * M_PI;
    val->mutable_weight(i) = i;
    if (i % 2 == 1) {
      val->mutable_weight(i) *= -1;
      val->mutable_value(i) *= -1;
    }

    if (val->weight(i) < 0)
      val->mutable_constraint_type(i) = ConstraintType::Hard;
    else if (val->weight(i) > 0)
      val->mutable_constraint_type(i) = ConstraintType::Soft;
    else
      val->mutable_constraint_type(i) = ConstraintType::Skip;
  }
}

// Test equality of the given ConstraintValues and lcmt_constrained_values
// message.
static void TestConstrainedValuesMsg(const ConstrainedValues& val, const lcmt_constrained_values& msg) {
  EXPECT_TRUE(msg.size == val.size());
  EXPECT_TRUE(msg.types.size() == msg.weights.size());
  EXPECT_TRUE(msg.types.size() == msg.values.size());
  EXPECT_TRUE(static_cast<int>(msg.types.size()) == msg.size);
  for (int i = 0; i < val.size(); i++) {
    EXPECT_TRUE(msg.types[i] == EncodeConstraintType(val.constraint_type(i)));
    EXPECT_TRUE(msg.weights[i] == val.weight(i));
    EXPECT_TRUE(msg.values[i] == val.value(i));
  }

  ConstrainedValues decoded_val;
  DecodeConstrainedValues(msg, &decoded_val);
  EXPECT_TRUE(val == decoded_val);
}

// Test encode / decode of ConstrainedValues.
GTEST_TEST(testLcmUtils, testEncodeDecodeConstrainedValues) {
  ConstrainedValues val;
  SetTestConstrainedValues(&val, 7);

  lcmt_constrained_values msg;
  EncodeConstrainedValues(val, &msg);

  TestConstrainedValuesMsg(val, msg);
}

// Test encode / decode of ContactInformation.
GTEST_TEST(testLcmUtils, testEncodeDecodeContactInformation) {
  ContactInformation info(*robot.FindBody("leftFoot"), 5);
  info.mutable_contact_points().push_back(Eigen::Vector3d(0.3, -0.1, 1));
  info.mutable_contact_points().push_back(Eigen::Vector3d(-0.3, 0.1, -1));
  info.mutable_acceleration_constraint_type() = ConstraintType::Soft;
  info.mutable_weight() = M_PI;
  info.mutable_Kd() = 0.3;

  // Test encode.
  lcmt_contact_information msg;
  EncodeContactInformation(info, &msg);
  EXPECT_TRUE(msg.body_name.compare("leftFoot") == 0);
  EXPECT_TRUE(msg.num_contact_points == static_cast<int>(info.contact_points().size()));
  EXPECT_TRUE(msg.num_basis_per_contact_point == 5);
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(static_cast<int>(msg.contact_points[i].size()) == msg.num_contact_points);
    for (int j = 0; j < msg.num_contact_points; ++j) {
      EXPECT_TRUE(msg.contact_points[i][j] == info.contact_points()[j][i]);
    }
  }
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(msg.normal[i] == info.normal()[i]);
  }
  EXPECT_TRUE(msg.mu == info.mu());
  EXPECT_TRUE(msg.Kd == info.Kd());
  EXPECT_TRUE(msg.weight == info.weight());
  EXPECT_TRUE(msg.acceleration_constraint_type == lcmt_constrained_values::SOFT);

  // Test decode.
  ContactInformation decoded_info(*robot.FindBody("rightFoot"));
  DecodeContactInformation(robot, msg, &decoded_info);
  EXPECT_TRUE(decoded_info == info);
}

// Test encode / decode of DesiredBodyMotion
GTEST_TEST(testLcmUtils, testEncodeDecodeDesiredBodyMotion) {
  DesiredBodyMotion mot(*robot.FindBody("leftFoot"));
  mot.mutable_control_during_contact() = true;
  SetTestConstrainedValues(&mot, mot.size());

  // Test encode.
  lcmt_desired_body_motion msg;
  EncodeDesiredBodyMotion(mot, &msg);
  // Test the ConstrainedValues part.
  TestConstrainedValuesMsg(mot, msg.constrained_accelerations);
  // Test extra fields.
  EXPECT_TRUE(mot.body_name().compare(msg.body_name) == 0);
  EXPECT_TRUE(mot.control_during_contact() == msg.control_during_contact);

  // Test decode
  DesiredBodyMotion decoded_mot(*robot.FindBody("rightFoot"));
  DecodeDesiredBodyMotion(robot, msg, &decoded_mot);
  EXPECT_TRUE(decoded_mot == mot);
}

// Test encode / decode of DesiredJointMotions
GTEST_TEST(testLcmUtils, testEncodeDecodeDesiredJointMotions) {
  DesiredJointMotions mot({"a", "b", "c", "d"});
  SetTestConstrainedValues(&mot, mot.size());

  // Test encode.
  lcmt_desired_joint_motions msg;
  EncodeDesiredJointMotions(mot, &msg);

  // Test the ConstrainedValues part.
  TestConstrainedValuesMsg(mot, msg.constrained_accelerations);
  // Test extra fields.
  EXPECT_TRUE(msg.num_joints == mot.size());
  EXPECT_TRUE(static_cast<int>(msg.joint_names.size()) == mot.size());
  for (int i = 0; i < mot.size(); ++i) {
    EXPECT_TRUE(msg.joint_names[i].compare(mot.joint_name(i)) == 0);
  }

  // Test decode.
  DesiredJointMotions decoded_mot;
  DecodeDesiredJointMotions(msg, &decoded_mot);
  EXPECT_TRUE(decoded_mot == mot);
}

// Test encode / decode of DesiredCentroidalMomentumDot.
GTEST_TEST(testLcmUtils, testEncodeDecodeDesiredCentroidalMomentumDot) {
  DesiredCentroidalMomentumDot Ld;
  SetTestConstrainedValues(&Ld, Ld.size());

  // Test encode.
  lcmt_desired_centroidal_momentum_dot msg;
  EncodeDesiredCentroidalMomentumDot(Ld, &msg);
  TestConstrainedValuesMsg(Ld, msg.centroidal_momentum_dot);

  // Test decode.
  DesiredCentroidalMomentumDot decoded_Ld;
  DecodeDesiredCentroidalMomentumDot(msg, &decoded_Ld);
  EXPECT_TRUE(Ld == decoded_Ld);
}

}  // namespace

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

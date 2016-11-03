#include <cstdlib>

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/example_qp_input_for_valkyrie.h"

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

template <typename Derived, typename Scalar>
void TestEigenVectorAndStdVector(const Eigen::MatrixBase<Derived>& eigvec, const std::vector<Scalar>& stdvec) {
  EXPECT_TRUE(eigvec.rows() == 1 || eigvec.cols() == 1);
  EXPECT_TRUE(static_cast<size_t>(eigvec.size()) == stdvec.size());
  for (size_t i = 0; i < stdvec.size(); ++i) {
    EXPECT_TRUE(static_cast<Scalar>(eigvec(i)) == stdvec[i]);
  }
}

template <typename Derived, typename Scalar, size_t Size>
void TestEigenVectorAndCArray(const Eigen::MatrixBase<Derived>& eigvec, const Scalar (&array)[Size]) {
  EXPECT_TRUE(eigvec.rows() == 1 || eigvec.cols() == 1);
  EXPECT_TRUE(static_cast<size_t>(eigvec.size()) == Size);
  for (size_t i = 0; i < Size; ++i) {
    EXPECT_TRUE(static_cast<Scalar>(eigvec(i)) == array[i]);
  }
}

template <typename Derived, typename Scalar>
void TestEigenMatrixAndStdVectorOfStdVector(const Eigen::MatrixBase<Derived>& eigmat, const std::vector<std::vector<Scalar>>& stdvecvec) {
  EXPECT_TRUE(static_cast<size_t>(eigmat.rows()) == stdvecvec.size());
  for (int row = 0; row < eigmat.rows(); ++row) {
    TestEigenVectorAndStdVector(eigmat.row(row), stdvecvec[row]);
  }
}

// Resize val to the given dimension, then set value and weight to alternating
// positive and negative values, finally set the constraint type based on the
// weight's sign.
static void SetConstrainedValues(ConstrainedValues* val, int dim) {
  if (!val) return;
  val->mutable_values() = VectorX<double>::Random(dim);
  val->mutable_weights() = VectorX<double>::Random(dim);
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
  }
  TestEigenVectorAndStdVector(val.weights(), msg.weights);
  TestEigenVectorAndStdVector(val.values(), msg.values);

  ConstrainedValues decoded_val;
  DecodeConstrainedValues(msg, &decoded_val);
  EXPECT_TRUE(val == decoded_val);
}

static void TestEncodeContactInformation(const ContactInformation& info, const lcmt_contact_information& msg) {
  EXPECT_TRUE(msg.body_name.compare(info.body_name()) == 0);
  EXPECT_TRUE(msg.num_contact_points == info.num_contact_points());
  EXPECT_TRUE(msg.num_basis_per_contact_point == info.num_basis_per_contact_point());
  TestEigenMatrixAndStdVectorOfStdVector(info.contact_points(), msg.contact_points);
  TestEigenVectorAndCArray(info.normal(), msg.normal);
  EXPECT_TRUE(msg.mu == info.mu());
  EXPECT_TRUE(msg.Kd == info.Kd());
  EXPECT_TRUE(msg.weight == info.weight());
  EXPECT_TRUE(msg.acceleration_constraint_type == EncodeConstraintType(info.acceleration_constraint_type()));
}

static void TestEncodeDesiredBodyMotion(const DesiredBodyMotion& mot, const lcmt_desired_body_motion& msg) {
  EXPECT_TRUE(mot.body_name().compare(msg.body_name) == 0);
  EXPECT_TRUE(mot.control_during_contact() == msg.control_during_contact);
  TestConstrainedValuesMsg(mot, msg.constrained_accelerations);
}

static void TestEncodeDesiredDoFMotions(const DesiredDoFMotions& mot, const lcmt_desired_dof_motions& msg) {
  EXPECT_TRUE(msg.num_dof == mot.size());
  EXPECT_TRUE(static_cast<int>(msg.dof_names.size()) == mot.size());
  for (int i = 0; i < mot.size(); ++i) {
    EXPECT_TRUE(msg.dof_names[i].compare(mot.dof_name(i)) == 0);
  }
  TestConstrainedValuesMsg(mot, msg.constrained_accelerations);
}

static void TestEncodeQPInput(const QPInput& qp_input, const lcmt_qp_input& msg) {
  // contacts
  EXPECT_TRUE(msg.num_contacts == static_cast<int>(qp_input.contact_information().size()));
  EXPECT_TRUE(msg.contact_information.size() == qp_input.contact_information().size());
  for (const auto& msg_contact : msg.contact_information) {
    auto it = qp_input.contact_information().find(msg_contact.body_name);
    EXPECT_TRUE(it != qp_input.contact_information().end());
    TestEncodeContactInformation(it->second, msg_contact);
  }
  // body motions
  EXPECT_TRUE(msg.num_desired_body_motions == static_cast<int>(qp_input.desired_body_motions().size()));
  EXPECT_TRUE(msg.desired_body_motions.size() == qp_input.desired_body_motions().size());
  for (const auto& msg_mot : msg.desired_body_motions) {
    auto it = qp_input.desired_body_motions().find(msg_mot.body_name);
    EXPECT_TRUE(it != qp_input.desired_body_motions().end());
    TestEncodeDesiredBodyMotion(it->second, msg_mot);
  }
  // dof motions
  TestEncodeDesiredDoFMotions(qp_input.desired_dof_motions(), msg.desired_dof_motions);

  // centroidal momentum
  TestConstrainedValuesMsg(qp_input.desired_centroidal_momentum_dot(), msg.desired_centroidal_momentum_dot.centroidal_momentum_dot);

  // basis regularization weight
  EXPECT_TRUE(msg.w_basis_reg == qp_input.w_basis_reg());
}

static void TestResolvedContact(const ResolvedContact& contact, const lcmt_resolved_contact& msg) {
  EXPECT_TRUE(contact.body_name().compare(msg.body_name) == 0);
  EXPECT_TRUE(contact.basis().size() == msg.num_all_basis);
  EXPECT_TRUE(contact.num_basis_per_contact_point() == msg.num_basis_per_contact_point);
  TestEigenVectorAndStdVector(contact.basis(), msg.basis);
  TestEigenMatrixAndStdVectorOfStdVector(contact.point_forces(), msg.point_forces);
  TestEigenMatrixAndStdVectorOfStdVector(contact.contact_points(), msg.contact_points);
  TestEigenVectorAndCArray(contact.equivalent_wrench(), msg.equivalent_wrench);
  TestEigenVectorAndCArray(contact.reference_point(), msg.reference_point);
}

static void TestBodyAcceleration(const BodyAcceleration& acc, const lcmt_body_acceleration& msg) {
  EXPECT_TRUE(acc.body_name().compare(msg.body_name) == 0);
  TestEigenVectorAndCArray(acc.accelerations(), msg.accelerations);
}

GTEST_TEST(testLcmUtils, testEncodeDecodeResolvedContact) {
  ResolvedContact contact(*robot.FindBody("leftFoot"));
  contact.mutable_num_basis_per_contact_point() = 4;
  contact.mutable_basis().resize(4);
  contact.mutable_basis() << 0.1, 0.0, 0.2, 0.3;
  contact.mutable_point_forces().resize(3, 1);
  contact.mutable_point_forces() << 0.1, 0.2, 0.3;
  contact.mutable_contact_points().resize(3, 1);
  contact.mutable_contact_points() << -0.1, -0.2, -0.3;
  contact.mutable_equivalent_wrench() << 1, 2, 3, 4, 5, 6;
  contact.mutable_reference_point() << -1, -2, -3;

  lcmt_resolved_contact msg;
  EncodeResolvedContact(contact, &msg);
  TestResolvedContact(contact, msg);

  ResolvedContact decoded_contact(*robot.FindBody("world"));
  DecodeResolvedContact(robot, msg, &decoded_contact);
  EXPECT_TRUE(decoded_contact == contact);
}

GTEST_TEST(testLcmUtils, testEncodeDecodeBodyAcceleration) {
  BodyAcceleration acc(*robot.FindBody("leftFoot"));
  acc.mutable_accelerations() << 1, 2, 3, 4, 5, 6;

  lcmt_body_acceleration msg;
  EncodeBodyAcceleration(acc, &msg);
  TestBodyAcceleration(acc, msg);

  BodyAcceleration decoded_acc(*robot.FindBody("world"));
  DecodeBodyAcceleration(robot, msg, &decoded_acc);
  EXPECT_TRUE(decoded_acc == acc);
}

// Test encode / decode of ConstrainedValues.
GTEST_TEST(testLcmUtils, testEncodeDecodeConstrainedValues) {
  ConstrainedValues val;
  SetConstrainedValues(&val, 7);

  // Test encode.
  lcmt_constrained_values msg;
  EncodeConstrainedValues(val, &msg);
  TestConstrainedValuesMsg(val, msg);

  // Test decode.
  ConstrainedValues decoded_val;
  DecodeConstrainedValues(msg, &decoded_val);
  EXPECT_TRUE(decoded_val == val);
}

// Test encode / decode of ContactInformation.
GTEST_TEST(testLcmUtils, testEncodeDecodeContactInformation) {
  ContactInformation info(*robot.FindBody("leftFoot"), 5);
  info.mutable_contact_points().resize(3, 2);
  info.mutable_contact_points().col(0) = Vector3<double>(0.3, -0.1, 1);
  info.mutable_contact_points().col(1) = Vector3<double>(-0.3, 0.1, -1);
  info.mutable_acceleration_constraint_type() = ConstraintType::Soft;
  info.mutable_weight() = M_PI;
  info.mutable_Kd() = 0.3;

  // Test encode.
  lcmt_contact_information msg;
  EncodeContactInformation(info, &msg);
  TestEncodeContactInformation(info, msg);

  // Test decode.
  ContactInformation decoded_info(*robot.FindBody("world"));
  DecodeContactInformation(robot, msg, &decoded_info);
  EXPECT_TRUE(decoded_info == info);
}

// Test encode / decode of DesiredBodyMotion
GTEST_TEST(testLcmUtils, testEncodeDecodeDesiredBodyMotion) {
  DesiredBodyMotion mot(*robot.FindBody("leftFoot"));
  mot.mutable_control_during_contact() = true;
  SetConstrainedValues(&mot, mot.size());

  // Test encode.
  lcmt_desired_body_motion msg;
  EncodeDesiredBodyMotion(mot, &msg);
  TestEncodeDesiredBodyMotion(mot, msg);

  // Test decode
  DesiredBodyMotion decoded_mot(*robot.FindBody("world"));
  DecodeDesiredBodyMotion(robot, msg, &decoded_mot);
  EXPECT_TRUE(decoded_mot == mot);
}

// Test encode / decode of DesiredDoFMotions
GTEST_TEST(testLcmUtils, testEncodeDecodeDesiredDoFMotions) {
  DesiredDoFMotions mot({"a", "b", "c", "d"});
  SetConstrainedValues(&mot, mot.size());

  // Test encode.
  lcmt_desired_dof_motions msg;
  EncodeDesiredDoFMotions(mot, &msg);
  TestEncodeDesiredDoFMotions(mot, msg);

  // Test decode.
  DesiredDoFMotions decoded_mot;
  DecodeDesiredDoFMotions(msg, &decoded_mot);
  EXPECT_TRUE(decoded_mot == mot);
}

// Test encode / decode of DesiredCentroidalMomentumDot.
GTEST_TEST(testLcmUtils, testEncodeDecodeDesiredCentroidalMomentumDot) {
  DesiredCentroidalMomentumDot Ld;
  SetConstrainedValues(&Ld, Ld.size());

  // Test encode.
  lcmt_desired_centroidal_momentum_dot msg;
  EncodeDesiredCentroidalMomentumDot(Ld, &msg);
  TestConstrainedValuesMsg(Ld, msg.centroidal_momentum_dot);

  // Test decode.
  DesiredCentroidalMomentumDot decoded_Ld;
  DecodeDesiredCentroidalMomentumDot(msg, &decoded_Ld);
  EXPECT_TRUE(Ld == decoded_Ld);
}

GTEST_TEST(testLcmUtils, testEncodeDecodeQPInput) {
  HumanoidStatus robot_status(robot);
  QPInput qp_input = MakeExampleQPInput(robot_status);

  // Test encode.
  lcmt_qp_input msg;
  EncodeQPInput(qp_input, &msg);
  TestEncodeQPInput(qp_input, msg);

  // Test decode.
  QPInput decoded_qp_input(robot);
  DecodeQPInput(robot, msg, &decoded_qp_input);
  EXPECT_TRUE(qp_input == decoded_qp_input);
}


}  // namespace

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

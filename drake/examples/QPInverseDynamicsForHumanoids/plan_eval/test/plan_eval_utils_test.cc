#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/plan_eval_utils.h"

#include <gtest/gtest.h>
#include "drake/common/eigen_matrix_compare.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

class PiecewiseCubicTrajectoryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::vector<double> times = {0, 2, 3, 4};
    std::vector<MatrixX<double>> knots(times.size(), MatrixX<double>::Zero(2, 1));
    knots[0] << 0, 1;
    knots[1] << 2, -3;
    knots[2] << 1.2, 5;
    knots[3] << -1, 6;

    test_times_ = {times.front() - 0.2,
                   times.front(),
                   (times.front() + times.back()) / 2.,
                   times.back(),
                   times.back() + 0.3};

    pos_ = PiecewisePolynomial<double>::Cubic(times, knots);
    dut_ = PiecewiseCubicTrajectory<double>(pos_);
    vel_ = pos_.derivative();
    acc_ = vel_.derivative();
  }

  PiecewiseCubicTrajectory<double> dut_;
  std::vector<double> test_times_;

  PiecewisePolynomial<double> pos_;
  PiecewisePolynomial<double> vel_;
  PiecewisePolynomial<double> acc_;
};

TEST_F(PiecewiseCubicTrajectoryTest, GetPosition) {
  for (double time : test_times_) {
    EXPECT_TRUE(drake::CompareMatrices(
          pos_.value(time),
          dut_.get_position(time),
          1e-12, drake::MatrixCompareType::absolute));
  }
}

TEST_F(PiecewiseCubicTrajectoryTest, GetVelocity) {
  for (double time : test_times_) {
    VectorX<double> expected = vel_.value(time);
    if (time < pos_.getStartTime() || time > pos_.getEndTime())
      expected.setZero();

    EXPECT_TRUE(drake::CompareMatrices(
          expected,
          dut_.get_velocity(time),
          1e-12, drake::MatrixCompareType::absolute));
  }
}

TEST_F(PiecewiseCubicTrajectoryTest, GetAcceleration) {
  for (double time : test_times_) {
    VectorX<double> expected = acc_.value(time);
    if (time < pos_.getStartTime() || time > pos_.getEndTime())
      expected.setZero();

    EXPECT_TRUE(drake::CompareMatrices(
          expected,
          dut_.get_acceleration(time),
          1e-12, drake::MatrixCompareType::absolute));
  }
}

TEST_F(PiecewiseCubicTrajectoryTest, IsApprox) {
  PiecewisePolynomial<double> poly = pos_;
  PiecewiseCubicTrajectory<double> same(poly);

  EXPECT_TRUE(dut_.is_approx(same, 1e-12));

  auto poly_matrix = poly.getPolynomialMatrix(0);
  Polynomial<double> eps(1e-6);
  poly_matrix(0, 0) += eps;
  poly.setPolynomialMatrixBlock(poly_matrix, 0);

  PiecewiseCubicTrajectory<double> diff(poly);
  EXPECT_TRUE(dut_.is_approx(diff, 1e-5));
  EXPECT_FALSE(dut_.is_approx(diff, 1e-7));
}


}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

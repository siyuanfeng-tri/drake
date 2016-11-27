#include "drake/common/drake_assert.h"
#include "drake/math/continuous_algebraic_riccati_equation.h"

namespace drake {
namespace math {

Eigen::MatrixXd ContinuousAlgebraicRiccatiEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R) {
  const Eigen::Index n = B.rows(), m = B.cols();

  DRAKE_DEMAND(A.rows() == n && A.cols() == n);
  DRAKE_DEMAND(Q.rows() == n && Q.cols() == n);
  DRAKE_DEMAND(R.rows() == m && R.cols() == m);
  DRAKE_DEMAND((Q - Q.transpose()).lpNorm<Eigen::Infinity>() < 1e-10);
  DRAKE_DEMAND((R - R.transpose()).lpNorm<Eigen::Infinity>() < 1e-10);

  Eigen::LLT<Eigen::MatrixXd> R_cholesky(R);

  Eigen::MatrixXd H(2 * n, 2 * n);
  H << A, B * R_cholesky.solve(B.transpose()), Q, -A.transpose();
  if (R_cholesky.info() != Eigen::Success)
    throw std::runtime_error("R must be positive definite");

  Eigen::MatrixXd Z = H;
  Eigen::MatrixXd Z_old;

  // these could be options
  const double tolerance = 1e-9;
  const double max_iterations = 100;

  double relative_norm;
  size_t iteration = 0;

  const double p = static_cast<double>(Z.rows());

  do {
    Z_old = Z;
    // R. Byers. Solving the algebraic Riccati equation with the matrix sign
    // function. Linear Algebra Appl., 85:267–279, 1987
    // Added determinant scaling to improve convergence (converges in rough half
    // the iterations with this)
    double ck = std::pow(std::abs(Z.determinant()), -1.0 / p);
    Z *= ck;
    Z = Z - 0.5 * (Z - Z.inverse());
    relative_norm = (Z - Z_old).norm();
    iteration++;
  } while (iteration < max_iterations && relative_norm > tolerance);

  Eigen::MatrixXd W11 = Z.block(0, 0, n, n);
  Eigen::MatrixXd W12 = Z.block(0, n, n, n);
  Eigen::MatrixXd W21 = Z.block(n, 0, n, n);
  Eigen::MatrixXd W22 = Z.block(n, n, n, n);

  Eigen::MatrixXd lhs(2 * n, n);
  Eigen::MatrixXd rhs(2 * n, n);
  Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(n, n);
  lhs << W12, W22 + eye;
  rhs << W11 + eye, W21;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      lhs, Eigen::ComputeThinU | Eigen::ComputeThinV);

  return svd.solve(rhs);
}

Eigen::MatrixXd ContinuousAlgebraicRiccatiEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N) {
  DRAKE_DEMAND((R - R.transpose()).lpNorm<Eigen::Infinity>() < 1e-10);
  DRAKE_DEMAND(Q.cols() == N.rows() && R.rows() == N.cols());

  Eigen::LLT<Eigen::MatrixXd> R_cholesky(R);

  Eigen::MatrixXd Q1 = Q - N * R_cholesky.solve(N.transpose());
  Eigen::MatrixXd A1 = A - B * R_cholesky.solve(N.transpose());
  return ContinuousAlgebraicRiccatiEquation(A1, B, Q1, R);
}

}  // namespace math
}  // namespace drake

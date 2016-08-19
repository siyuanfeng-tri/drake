#include "drake/solvers/optimization.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/gurobi_solver.h"

#include "qp_controller.h"

using namespace drake::solvers;

// TODO(siyuan.feng@tri.global): some version of this should go to
// optimization.h
static VectorXd VariableList2VectorXd(VariableList const& vlist) {
  size_t dim = 0;
  for (auto var : vlist) {
    dim += var.size();
  }
  VectorXd X(dim);
  dim = 0;
  for (auto var : vlist) {
    X.segment(dim, var.size()) = var.value();
    dim += var.size();
  }
  return X;
}

void QPControllerNew::ResizeQP(const HumanoidStatus& rs, const std::vector<SupportElement> &all_contacts) {
  num_contacts_ = all_contacts.size();
  num_vd_ = rs.robot().number_of_velocities();
  num_basis_ = 0;
  int num_point_force = 0;
  for (size_t i = 0; i < all_contacts.size(); i++) {
    num_point_force += all_contacts[i].contact_points().size();
    num_basis_ += n_basis_per_contact_point * all_contacts[i].contact_points().size();
  }
  num_torque_ = rs.robot().actuators.size();
  num_variable_ = num_vd_ + num_basis_;

  stacked_contact_jacobians_.resize(3 * num_point_force, num_vd_);
  basis_to_force_matrix_.resize(num_point_force, num_basis_);

  // this order is important
  prog_ = OptimizationProblem();
  DecisionVariableView vd = prog_.AddContinuousVariables(num_vd_, "vd");
  DecisionVariableView basis = prog_.AddContinuousVariables(num_basis_, "basis");

  // allocate equality constraints
  // dyanmics
  eq_dynamics_ = prog_.AddLinearEqualityConstraint(MatrixXd::Zero(6, num_variable_), Matrix<double, 6, 1>::Zero(), {vd, basis});
  eq_dynamics_->set_description("dynamics eq");
  eq_contacts_.resize(num_contacts_);
  // contact constraints, 6 rows per contact
  for (int i = 0; i < num_contacts_; i++) {
    eq_contacts_[i] = prog_.AddLinearEqualityConstraint(MatrixXd::Zero(6, num_vd_), Matrix<double, 6, 1>::Zero(), {vd});
    eq_contacts_[i]->set_description("contact eq");
  }

  // allocate inequality constraints
  // contact ft
  ineq_contact_wrench_ = prog_.AddLinearConstraint(MatrixXd::Zero(num_basis_, num_basis_), VectorXd::Zero(num_basis_), VectorXd::Zero(num_basis_), {basis});
  ineq_contact_wrench_->set_description("contact force basis ineq");
  // trq lim
  ineq_torque_limit_ = prog_.AddLinearConstraint(MatrixXd::Zero(num_torque_, num_variable_), VectorXd::Zero(num_torque_), VectorXd::Zero(num_torque_), {vd, basis});
  ineq_torque_limit_->set_description("torque limit ineq");

  // allocate cost func:
  Eigen::MatrixXd tmp_matrix_vd(num_vd_, num_vd_);
  Eigen::VectorXd tmp_vector_vd(num_vd_);

  // com
  cost_comdd_ = prog_.AddQuadraticCost(tmp_matrix_vd, tmp_vector_vd, {vd});
  cost_comdd_->set_description("com cost");
  // pelv
  cost_pelvdd_ = prog_.AddQuadraticCost(tmp_matrix_vd, tmp_vector_vd, {vd});
  cost_pelvdd_->set_description("pelv cost");
  // torso
  cost_torsodd_ = prog_.AddQuadraticCost(tmp_matrix_vd, tmp_vector_vd, {vd});
  cost_torsodd_->set_description("torso cost");
  // l foot
  //prog_.AddQuadraticCost(tmp_matrix_vd, tmp_vector_vd, {vd_});
  // r foot
  //prog_.AddQuadraticCost(tmp_matrix_vd, tmp_vector_vd, {vd_});
  // reg vd
  cost_vd_reg_ = prog_.AddQuadraticCost(tmp_matrix_vd, tmp_vector_vd, {vd});
  cost_vd_reg_->set_description("vd cost");
  // reg lambda
  //cost_lambda_reg_ = prog_.AddQuadraticCost(MatrixXd::Identity(num_basis_, num_basis_), VectorXd::Zero(num_basis_), {lambda_});
  //cost_lambda_reg_->set_description("lambda cost");
}

int QPControllerNew::Control(const HumanoidStatus& rs, const QPInput& input,
                          QPOutput* output) {
  if (!is_qp_input_sane(input)) {
    std::cerr << "input is invalid\n";
    return -1;
  }

  ////////////////////////////////////////////////////////////////////
  // The equations of motion look like:
  // M(q) * vd + h(q,v) = S * tau + J^T * lambda_
  // M(q) is the inertia matrix, h(q,v) is the gravitational and centrifugal
  // force, vd is acceleration, S is the selection matrix (top 6 rows are
  // zeros due to the floating base), tau is joint torque, J^T is the transpose
  // of all contact Jacobian, and lambda_ is the contact wrench in the world
  // frame.
  //
  // For inverse dynamics, we are usually given desired motions, and
  // we want to solve for tau to achieve those motions.
  // Desired motions can be directly specified as vd_d, or as xdd_d in Cartesian
  // space, which is linear w.r.t. vd as well: xdd_d = J * vd + Jd * v.
  //
  // Note that since S.topRows(6) is zero,
  // tau = M_l * vd + h_l - J^T_l * lamda,
  // where _l means the lower num_torque_ rows of those matrices.
  // So we just need to solve for vd and lambda_, and tau can be computed as
  // above. We can formulate inverse dynamics a QP problem.
  //
  // For the QP problem:
  // the unknown is _X = [vd, lambda_]
  // equality constraints:
  //  M_u * vd + h_u = J^T_u * lambda_ (equations of motion)
  //  J * vd + Jd * v = 0, (contact constraints)
  // inEquality: a bunch, joint torque limit, limits on lambda_, etc
  // cost func:
  //  min (Jcom*vd + Jcomd*v - comdd_d)^2
  //    + (vd - vd_d)^2
  //    + (lambda_ - lambda_d)^2
  //    + all_kinds_of_body_acceleration_cost_terms
  //
  // I made the dynamics and stationary contact equality constraints.
  // Alternatively, they can be setup as high weight cost terms. This is
  // sometimes preferred as it introduce slacks for better stability.
  //
  // We are assuming two foot contacts in this example.
  const DecisionVariableView vd = prog_.get_variable("vd");
  const DecisionVariableView basis = prog_.get_variable("basis");

  int basis_start = basis.index();
  int vd_start = vd.index();

  // stack the contact jacobian together
  std::vector<SupportElement> all_contacts;

  int ctr = 0;
  for (auto b : all_contacts) {
    b.ComputeBasisMatrix(rs.robot(), rs.cache(), n_basis_per_contact_point, basis_to_force_matrix_.block(3 * ctr, 0, 3 * b.contact_points().size(), n_basis_per_contact_point));
    b.ComputeJacobianAtContactPoints(rs.robot(), rs.cache(), stacked_contact_jacobians_.block(3 * ctr, 0, 3, num_vd_));
    ctr++;
  }

  // tau = M_l * vd + h_l - J^T_l * lambda_,
  // tau = torque_linear_ * _X + torque_constant_
  torque_linear_ = MatrixXd::Zero(num_torque_, num_variable_);
  torque_linear_.block(0, vd_start, num_torque_, num_vd_) =
      rs.M().bottomRows(num_torque_);
  for (int i = 0; i < num_contacts_; i++) {
    torque_linear_.block(0, basis_start + i * 6, num_torque_, 6) =
        -rs.foot(i).J().block(0, 6, 6, num_torque_).transpose();
  }
  torque_constant_ = rs.bias_term().tail(num_torque_);

  ////////////////////////////////////////////////////////////////////
  // equality constraints:
  // equations of motion part, 6 rows
  dynamics_linear_ = MatrixXd::Zero(6, num_variable_);
  dynamics_linear_.block(0, vd_start, 6, num_vd_) = rs.M().topRows(6);
  for (int i = 0; i < num_contacts_; i++) {
    dynamics_linear_.block(0, basis_start + i * 6, 6, 6) =
        -rs.foot(i).J().block<6, 6>(0, 0).transpose();
  }
  dynamics_constant_ = -rs.bias_term().head(6);
  eq_dynamics_->UpdateConstraint(dynamics_linear_, dynamics_constant_);

  // contact constraints, 6 rows per contact
  for (int i = 0; i < num_contacts_; i++) {
    eq_contacts_[i]->UpdateConstraint(
        rs.foot(i).J(), -(rs.foot(i).Jdot_times_v() - input.footdd_d[i]));
  }

  ////////////////////////////////////////////////////////////////////
  // set up inequality constraints
  // these are for contact wrench, 11 rows per contact.
  // NOTE: these constraints are specified for the contact wrench in the body
  // frame, but lambda_ are in world frame. So need to transform it by R^-1
  // 2 for Fx, Fy, Mz within friction cone, 6
  // 2 for CoP x within foot, 2
  // 2 for CoP y within foot, 2
  // 1 for Fz >= 0,
  int row_idx = 0;
  inequality_linear_ = MatrixXd::Zero(11 * num_contacts_, num_basis_);
  inequality_upper_bound_ = VectorXd::Constant(
      11 * num_contacts_, std::numeric_limits<double>::infinity());
  inequality_lower_bound_ = VectorXd::Constant(
      11 * num_contacts_, -std::numeric_limits<double>::infinity());

  // Fz >= 0;
  for (int i = 0; i < num_contacts_; i++) {
    inequality_linear_(row_idx, i * 6 + 5) = 1;
    inequality_lower_bound_(row_idx) = 0;
    row_idx++;
  }
  // This is a very crude model for approximating friction.
  // |Fx| < muFz and |Fy| < muFz is a "box" rather than a "cone" approximation
  // of friction.
  // The magnitude is off by a factor or sqrt(2).
  // Fx <= mu * Fz, Fx - mu * Fz <= 0
  for (int i = 0; i < num_contacts_; i++) {
    inequality_linear_(row_idx, i * 6 + 3) = 1;
    inequality_linear_(row_idx, i * 6 + 5) = -param.mu;
    inequality_upper_bound_(row_idx) = 0;
    row_idx++;
  }
  // Fx >= -mu * Fz, Fx + mu * Fz >= 0
  for (int i = 0; i < num_contacts_; i++) {
    inequality_linear_(row_idx, i * 6 + 3) = 1;
    inequality_linear_(row_idx, i * 6 + 5) = param.mu;
    inequality_lower_bound_(row_idx) = 0;
    row_idx++;
  }
  // Fy <= mu * Fz, Fy - mu * Fz <= 0
  for (int i = 0; i < num_contacts_; i++) {
    inequality_linear_(row_idx, i * 6 + 4) = 1;
    inequality_linear_(row_idx, i * 6 + 5) = -param.mu;
    inequality_upper_bound_(row_idx) = 0;
    row_idx++;
  }
  // Fy >= -mu * Fz, Fy + mu * Fz >= 0
  for (int i = 0; i < num_contacts_; i++) {
    inequality_linear_(row_idx, i * 6 + 4) = 1;
    inequality_linear_(row_idx, i * 6 + 5) = param.mu;
    inequality_lower_bound_(row_idx) = 0;
    row_idx++;
  }
  // These are poor approximations for the normal torque especially when the
  // center of pressure is at a corner of the foot. I used this in this
  // example for its simplicity. A better approximation is to use point contact
  // forces at multiple contact positions.
  // Mz <= mu * Fz, Mz - mu * Fz <= 0
  for (int i = 0; i < num_contacts_; i++) {
    inequality_linear_(row_idx, i * 6 + 2) = 1;
    inequality_linear_(row_idx, i * 6 + 5) = -param.mu_Mz;
    inequality_upper_bound_(row_idx) = 0;
    row_idx++;
  }
  // Mz >= -mu * Fz, Mz + mu * Fz >= 0
  for (int i = 0; i < num_contacts_; i++) {
    inequality_linear_(row_idx, i * 6 + 2) = 1;
    inequality_linear_(row_idx, i * 6 + 5) = param.mu_Mz;
    inequality_lower_bound_(row_idx) = 0;
    row_idx++;
  }
  // cop_x <= x_max, -My / Fz <= x_max, -My - x_max * Fz <= 0
  for (int i = 0; i < num_contacts_; i++) {
    inequality_linear_(row_idx, i * 6 + 1) = -1;
    inequality_linear_(row_idx, i * 6 + 5) = -param.x_max;
    inequality_upper_bound_(row_idx) = 0;
    row_idx++;
  }
  // cop_x >= x_min, -My / Fz >= x_min, -My - x_min * Fz >= 0
  for (int i = 0; i < num_contacts_; i++) {
    inequality_linear_(row_idx, i * 6 + 1) = -1;
    inequality_linear_(row_idx, i * 6 + 5) = -param.x_min;
    inequality_lower_bound_(row_idx) = 0;
    row_idx++;
  }
  // cop_y <= y_max, Mx / Fz <= y_max, Mx - y_max * Fz <= 0
  for (int i = 0; i < num_contacts_; i++) {
    inequality_linear_(row_idx, i * 6 + 0) = 1;
    inequality_linear_(row_idx, i * 6 + 5) = -param.y_max;
    inequality_upper_bound_(row_idx) = 0;
    row_idx++;
  }
  // cop_y >= y_min, Mx / Fz >= y_min, Mx - y_min * Fz >= 0
  for (int i = 0; i < num_contacts_; i++) {
    inequality_linear_(row_idx, i * 6 + 0) = 1;
    inequality_linear_(row_idx, i * 6 + 5) = -param.y_min;
    inequality_lower_bound_(row_idx) = 0;
    row_idx++;
  }
  // Since all of the above are constraints on wrench expressed in the body
  // frame, we need to rotate the lambda_ into body frame.
  MatrixXd world_to_foot(MatrixXd::Zero(num_basis_, num_basis_));
  for (int i = 0; i < num_contacts_; i++) {
    world_to_foot.block<3, 3>(i * 6, i * 6) =
        rs.foot(i).pose().linear().transpose();
    world_to_foot.block<3, 3>(i * 6 + 3, i * 6 + 3) =
        world_to_foot.block<3, 3>(i * 6, i * 6);
  }
  inequality_linear_ = inequality_linear_ * world_to_foot;
  ineq_contact_wrench_->UpdateConstraint(inequality_linear_, inequality_lower_bound_,
                           inequality_upper_bound_);

  // torque limits: min <= tau <= max, num_torque_ rows
  // min <= M_l * vd + h_l - J^T_l * lambda_ <= max
  // min - h_l <= M_l * vd - J^T_l * lambda_ <= max - h_l
  // tau = rs.robot->B.bottomRows(num_torque_) * u,
  // u = rs.robot->B.bottomRows(num_torque_).transpose() * tau
  // since B should be orthonormal.
  // tau is joint space indexed, and u is actuator space indexed.
  // constraints are specified with u index.
  inequality_linear_ =
      rs.robot().B.bottomRows(num_torque_).transpose() * torque_linear_;
  inequality_upper_bound_ = inequality_lower_bound_ =
      -rs.robot().B.bottomRows(num_torque_).transpose() * torque_constant_;
  for (size_t i = 0; i < rs.robot().actuators.size(); i++) {
    inequality_lower_bound_[i] += rs.robot().actuators[i].effort_limit_min_;
    inequality_upper_bound_[i] += rs.robot().actuators[i].effort_limit_max_;
  }
  ineq_torque_limit_->UpdateConstraint(inequality_linear_, inequality_lower_bound_,
                           inequality_upper_bound_);

  ////////////////////////////////////////////////////////////////////
  // cost function:
  // CoM term (task space acceleration costs)
  // w * (J * vd + Jdv - comdd_d)^T * (J * vd + Jdv - comdd_d)
  cost_comdd_->UpdateConstraint(
      input.w_com * rs.J_com().transpose() * rs.J_com(),
      input.w_com * (rs.Jdot_times_v_com() - input.comdd_d).transpose() *
          rs.J_com());

  cost_pelvdd_->UpdateConstraint(
      input.w_pelv * rs.pelv().J().transpose() * rs.pelv().J(),
      input.w_pelv * (rs.pelv().Jdot_times_v() - input.pelvdd_d).transpose() *
          rs.pelv().J());

  cost_torsodd_->UpdateConstraint(
      input.w_torso * rs.torso().J().transpose() * rs.torso().J(),
      input.w_torso * (rs.torso().Jdot_times_v() - input.torsodd_d).transpose() *
          rs.torso().J());

  // regularize vd to vd_d
  cost_vd_reg_->UpdateConstraint(input.w_vd * MatrixXd::Identity(num_vd_, num_vd_),
                        input.w_vd * (-input.vd_d));

  /*
  // regularize lambda_ to lambda_d
  VectorXd lambda_d(VectorXd::Zero(num_basis_));
  for (int i = 0; i < 2; i++) lambda_d.segment<6>(6 * i) = input.wrench_d[i];
  cost_lambda_reg_->UpdateConstraint(
      input.w_wrench_reg * MatrixXd::Identity(num_basis_, num_basis_),
      input.w_wrench_reg * (-lambda_d));
  */

  ////////////////////////////////////////////////////////////////////
  // solve
  SolutionResult result;
  //GurobiSolver solver_;
  SnoptSolver solver_;
  if (!solver_.available()) {
    std::cerr << "Solver not available.\n";
    return -1;
  }
  result = solver_.Solve(prog_);
  if (result != drake::solvers::SolutionResult::kSolutionFound) {
    std::cerr << "solution not found\n";
    return -1;
  }

  ////////////////////////////////////////////////////////////////////
  // example of inspecting each cost / eq, ineq term
  // These will not be called in a real controller
  auto costs = prog_.quadratic_costs();
  auto eqs = prog_.linear_equality_constraints();
  auto ineqs = prog_.linear_constraints();

  for (auto cost_b : costs) {
    VectorXd val;
    std::shared_ptr<Constraint> cost = cost_b.constraint();
    cost->Eval(VariableList2VectorXd(cost_b.variable_list()), val);
    std::cout << cost->get_description() << ": " << val.transpose()
              << std::endl;
  }

  for (auto eq_b : eqs) {
    std::shared_ptr<LinearEqualityConstraint> eq = eq_b.constraint();
    VectorXd X = VariableList2VectorXd(eq_b.variable_list());
    DRAKE_ASSERT((eq->A() * X - eq->lower_bound()).isZero(EPSILON));
    //std::cout << eq->get_description() << ": " << (eq->A() * X - eq->lower_bound()).transpose() << std::endl;
  }

  for (auto ineq_b : ineqs) {
    std::shared_ptr<LinearConstraint> ineq = ineq_b.constraint();
    VectorXd X = VariableList2VectorXd(ineq_b.variable_list());
    X = ineq->A() * X;
    for (int i = 0; i < X.size(); i++) {
      DRAKE_ASSERT(X[i] >= ineq->lower_bound()[i] - EPSILON && X[i] <= ineq->upper_bound()[i] + EPSILON);
    }
  }

  ////////////////////////////////////////////////////////////////////
  // parse result
  output->vd = vd.value();
  output->comdd = rs.J_com() * output->vd + rs.Jdot_times_v_com();
  output->pelvdd = rs.pelv().J() * output->vd + rs.pelv().Jdot_times_v();
  output->torsodd = rs.torso().J() * output->vd + rs.torso().Jdot_times_v();

  for (int i = 0; i < num_contacts_; i++) {
    output->footdd[i] = (rs.foot(i).J() * output->vd + rs.foot(i).Jdot_times_v());
    output->foot_wrench_in_world_frame[i] = basis.value().segment<6>(i * 6);
  }

  output->joint_torque = rs.M().bottomRows(num_torque_) * output->vd +
                         rs.bias_term().tail(num_torque_);
  for (int i = 0; i < num_contacts_; i++) {
    output->joint_torque -=
        rs.foot(i).J().block(0, 6, 6, num_torque_).transpose() *
        output->foot_wrench_in_world_frame[i];
  }

  for (int i = 0; i < num_contacts_; i++) {
    Isometry3d T(Isometry3d::Identity());
    T.translation() =
        rs.foot(i).pose().translation() - rs.foot_sensor(i).pose().translation();

    output->foot_wrench_in_sensor_frame[i] =
        transformSpatialForce(T, output->foot_wrench_in_world_frame[i]);

    output->foot_wrench_in_sensor_frame[i].head<3>() =
        rs.foot_sensor(i).pose().linear().transpose() *
        output->foot_wrench_in_sensor_frame[i].head<3>();
    output->foot_wrench_in_sensor_frame[i].tail<3>() =
        rs.foot_sensor(i).pose().linear().transpose() *
        output->foot_wrench_in_sensor_frame[i].tail<3>();
  }

  if (!is_qp_output_sane(*output)) {
    std::cerr << "output is invalid\n";
    return -1;
  }

  return 0;
}

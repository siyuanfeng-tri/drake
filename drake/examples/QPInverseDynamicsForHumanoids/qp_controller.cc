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

void QPController::ResizeQP(const HumanoidStatus& rs, const std::vector<SupportElement> &all_supports) {
  // figure out space
  int num_contacts = all_supports.size();
  int num_vd = rs.robot().number_of_velocities();
  int num_basis = 0;
  int num_point_forces = 0;
  for (size_t i = 0; i < all_supports.size(); i++) {
    num_point_forces += all_supports[i].contact_points().size();
    num_basis += n_basis_per_contact_point * all_supports[i].contact_points().size();
  }
  int num_torque = rs.robot().actuators.size();
  int num_variable = num_vd + num_basis;

  if (num_contacts == num_supports_ &&
      num_vd == num_vd_ &&
      num_basis == num_basis_ &&
      num_point_forces == num_point_forces_ &&
      num_torque == num_torque_ &&
      num_variable == num_variable_)
    return;

  num_supports_ = num_contacts;
  num_vd_ = num_vd;
  num_basis_ = num_basis;
  num_point_forces_ = num_point_forces;
  num_torque_ = num_torque;
  num_variable_ = num_variable;

  // this order is important
  prog_ = OptimizationProblem();
  DecisionVariableView vd = prog_.AddContinuousVariables(num_vd_, "vd");
  DecisionVariableView basis = prog_.AddContinuousVariables(num_basis_, "basis");

  // allocate space for contact force jacobian and basis matrix
  stacked_contact_jacobians_.resize(3 * num_point_forces_, num_vd_);
  basis_to_force_matrix_.resize(3 * num_point_forces_, num_basis_);
  stacked_contact_jacobians_dot_times_v_.resize(3 * num_point_forces_);

  // allocate equality constraints
  // dyanmics
  eq_dynamics_ = prog_.AddLinearEqualityConstraint(MatrixXd::Zero(6, num_variable_), Matrix<double, 6, 1>::Zero(), {vd, basis});
  eq_dynamics_->set_description("dynamics eq");
  eq_contacts_.resize(num_supports_);
  // contact constraints, 6 rows per contact
  for (int i = 0; i < num_supports_; i++) {
    eq_contacts_[i] = prog_.AddLinearEqualityConstraint(MatrixXd::Zero(3 * all_supports[i].contact_points().size(), num_vd_), VectorXd::Zero(3 * all_supports[i].contact_points().size()), {vd});
    eq_contacts_[i]->set_description("contact eq");
  }

  // allocate inequality constraints
  // contact ft
  ineq_contact_wrench_ = prog_.AddLinearConstraint(MatrixXd::Identity(num_basis_, num_basis_), VectorXd::Zero(num_basis_), VectorXd::Constant(num_basis_, 1000), {basis});
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
  // left foot
  cost_footdd_[Side::LEFT] = prog_.AddQuadraticCost(tmp_matrix_vd, tmp_vector_vd, {vd});
  cost_footdd_[Side::LEFT]->set_description("left foot cost");
  // right foot
  cost_footdd_[Side::RIGHT] = prog_.AddQuadraticCost(tmp_matrix_vd, tmp_vector_vd, {vd});
  cost_footdd_[Side::RIGHT]->set_description("right foot cost");

  // reg vd
  cost_vd_reg_ = prog_.AddQuadraticCost(tmp_matrix_vd, tmp_vector_vd, {vd});
  cost_vd_reg_->set_description("vd cost");
  // reg lambda
  //cost_lambda_reg_ = prog_.AddQuadraticCost(MatrixXd::Identity(num_basis_, num_basis_), VectorXd::Zero(num_basis_), {lambda_});
  //cost_lambda_reg_->set_description("lambda cost");

  // resize tmp matrices
  torque_linear_.resize(num_torque_, num_variable_);
  dynamics_linear_.resize(6, num_variable_);

  point_force_to_wrench_.resize(6 * num_supports_, 3 * num_point_forces_);
}

int QPController::Control(const HumanoidStatus& rs, const QPInput& input,
                          QPOutput* output) {
  if (!input.is_valid()) {
    std::cerr << "input is invalid\n";
    return -1;
  }
  ResizeQP(rs, input.supports());
  SetZero();

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
  int rowIdx = 0;
  int colIdx = 0;
  for (int b = 0; b < num_supports_; b++) {
    const SupportElement &support = input.support(b);
    int force_dim = 3 * support.contact_points().size();
    int basis_dim = n_basis_per_contact_point * support.contact_points().size();
    basis_to_force_matrix_.block(rowIdx, colIdx, force_dim, basis_dim) = support.ComputeBasisMatrix(rs.robot(), rs.cache(), n_basis_per_contact_point);
    stacked_contact_jacobians_.block(rowIdx, 0, force_dim, num_vd_) = support.ComputeJacobianAtContactPoints(rs.robot(), rs.cache());
    stacked_contact_jacobians_dot_times_v_.segment(rowIdx, force_dim) = support.ComputeJacobianDotTimesVAtContactPoints(rs.robot(), rs.cache());
    rowIdx += force_dim;
    colIdx += basis_dim;
  }
  JB_ = stacked_contact_jacobians_.transpose() * basis_to_force_matrix_;
  DRAKE_ASSERT(rowIdx == num_point_forces_ * 3);
  DRAKE_ASSERT(colIdx == num_basis_);

  // tau = M_l * vd + h_l - J^T_l * lambda_,
  // tau = torque_linear_ * X + torque_constant_
  torque_linear_.block(0, vd_start, num_torque_, num_vd_) =
      rs.M().bottomRows(num_torque_);
  torque_linear_.block(0, basis_start, num_torque_, num_basis_) = -JB_.bottomRows(num_torque_);
  torque_constant_ = rs.bias_term().tail(num_torque_);

  ////////////////////////////////////////////////////////////////////
  // equality constraints:
  // equations of motion part, 6 rows
  dynamics_linear_.block(0, vd_start, 6, num_vd_) = rs.M().topRows(6);
  dynamics_linear_.block(0, basis_start, 6, num_basis_) = -JB_.topRows(6);
  dynamics_constant_ = -rs.bias_term().head(6);
  eq_dynamics_->UpdateConstraint(dynamics_linear_, dynamics_constant_);

  // contact constraints, 6 rows per contact
  rowIdx = 0;
  for (int i = 0; i < num_supports_; i++) {
    //eq_contacts_[i]->UpdateConstraint(
    //    rs.foot(i).J(), -(rs.foot(i).Jdot_times_v() - input.footdd_d(i)));
    int force_dim = 3 * input.support(i).contact_points().size();
    eq_contacts_[i]->UpdateConstraint(
        stacked_contact_jacobians_.block(rowIdx, 0, force_dim, num_vd_),
        -stacked_contact_jacobians_dot_times_v_.segment(rowIdx, force_dim));
    rowIdx += force_dim;
  }
  DRAKE_ASSERT(rowIdx == num_point_forces_ * 3);

  ////////////////////////////////////////////////////////////////////
  // set up inequality constraints
  // For the contact point force basis, the constraints are always > 0,
  // so the stay constant as in ResizeQP.


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
      input.w_com * (rs.Jdot_times_v_com() - input.comdd_d()).transpose() *
          rs.J_com());

  cost_pelvdd_->UpdateConstraint(
      input.w_pelv * rs.pelv().J().transpose() * rs.pelv().J(),
      input.w_pelv * (rs.pelv().Jdot_times_v() - input.pelvdd_d()).transpose() *
          rs.pelv().J());

  cost_torsodd_->UpdateConstraint(
      input.w_torso * rs.torso().J().transpose() * rs.torso().J(),
      input.w_torso * (rs.torso().Jdot_times_v() - input.torsodd_d()).transpose() *
          rs.torso().J());

  for (int i = 0; i < 2; i++) {
    cost_footdd_[i]->UpdateConstraint(
        input.w_foot * rs.foot(i).J().transpose() * rs.foot(i).J(),
        input.w_foot * (rs.foot(i).Jdot_times_v() - input.footdd_d(i)).transpose() *
        rs.foot(i).J()); 
  }

  // regularize vd to vd_d
  cost_vd_reg_->UpdateConstraint(input.w_vd * MatrixXd::Identity(num_vd_, num_vd_),
                        input.w_vd * (-input.vd_d()));

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
  output->vd() = vd.value();
  output->comdd() = rs.J_com() * output->vd() + rs.Jdot_times_v_com();
  output->pelvdd() = rs.pelv().J() * output->vd() + rs.pelv().Jdot_times_v();
  output->torsodd() = rs.torso().J() * output->vd() + rs.torso().Jdot_times_v();

  output->joint_torque() = torque_linear_ * VariableList2VectorXd({vd, basis}) + torque_constant_;

  for (int i = 0; i < 2; i++) {
    output->footdd(i) = (rs.foot(i).J() * output->vd() + rs.foot(i).Jdot_times_v());
    output->foot_wrench_in_world_frame(i).setZero();
    output->foot_wrench_in_sensor_frame(i).setZero();
  }

  // matrix from basis to wrench
  rowIdx = colIdx = 0;
  for (int i = 0; i < num_supports_; i++) {
    const SupportElement &support = input.support(i);
    const Vector3d ref_point = rs.robot().transformPoints(rs.cache(), Vector3d::Zero(), support.body().get_body_index(), 0);

    for (size_t j = 0; j < support.contact_points().size(); j++) {
      const Vector3d contact_point = rs.robot().transformPoints(rs.cache(), support.contact_points()[j], support.body().get_body_index(), 0);
      // Force part: just sum up all the point forces, so theese are I
      point_force_to_wrench_.block<3, 3>(rowIdx + 3, colIdx).setIdentity();
      // Torque part:
      point_force_to_wrench_.block<3, 3>(rowIdx, colIdx) = vectorToSkewSymmetric(contact_point - ref_point);
      colIdx += 3;
    }
    rowIdx += 6;
  }
  DRAKE_ASSERT(rowIdx == 6 * num_supports_);
  DRAKE_ASSERT(colIdx == 3 * num_point_forces_);
  contact_wrenches_ = point_force_to_wrench_ * basis_to_force_matrix_ * basis.value();

  // Compute wrench for left and right foot in the world frame.
  for (int i = 0; i < num_supports_; i++) {
    int contact_body_idx = input.support(i).body().get_body_index();
    if (contact_body_idx == rs.foot_sensor(Side::LEFT).body().get_body_index()) {
      output->foot_wrench_in_world_frame(Side::LEFT) = contact_wrenches_.segment<6>(6 * i);
    }
    else if (contact_body_idx == rs.foot_sensor(Side::RIGHT).body().get_body_index()) {
      output->foot_wrench_in_world_frame(Side::RIGHT) = contact_wrenches_.segment<6>(6 * i);
    }
  }

  // Convert world frame wrench to sensor frame
  for (int i = 0; i < 2; i++) {
    Isometry3d T(Isometry3d::Identity());
    T.translation() =
        rs.foot(i).pose().translation() - rs.foot_sensor(i).pose().translation();

    output->foot_wrench_in_sensor_frame(i) =
        transformSpatialForce(T, output->foot_wrench_in_world_frame(i));

    output->foot_wrench_in_sensor_frame(i).head<3>() =
        rs.foot_sensor(i).pose().linear().transpose() *
        output->foot_wrench_in_sensor_frame(i).head<3>();
    output->foot_wrench_in_sensor_frame(i).tail<3>() =
        rs.foot_sensor(i).pose().linear().transpose() *
        output->foot_wrench_in_sensor_frame(i).tail<3>();
  }

  // Sanity check, net external wrench should = centroidal_matrix * vd + centroidal_matrix_dot * v
  Vector6d Ld = rs.centroidal_momentum_matrix() * output->vd() + rs.centroidal_momentum_matrix_dot_times_v();
  Vector6d net_wrench = rs.robot().getMass() * rs.robot().a_grav;
  for (int i = 0; i < num_supports_; i++) {
    int contact_body_idx = input.support(i).body().get_body_index();
    net_wrench += contact_wrenches_.segment<6>(6 * i);
    const Vector3d ref_point = rs.robot().transformPoints(rs.cache(), Vector3d::Zero(), contact_body_idx, 0);
    net_wrench.segment<3>(0) += (ref_point - rs.com()).cross(contact_wrenches_.segment<3>(6 * i + 3));
  }
  DRAKE_ASSERT((net_wrench - Ld).isZero(EPSILON));

  if (!output->is_valid()) {
    std::cerr << "output is invalid\n";
    return -1;
  }

  return 0;
} 











double QPOutput::ComputeQPCost(const HumanoidStatus& rs, const QPInput& input) const {
  VectorXd c, tot;
  c = 0.5 * vd_.transpose() * input.w_com * rs.J_com().transpose() *
          rs.J_com() * vd_ +
      input.w_com * (rs.Jdot_times_v_com() - input.comdd_d()).transpose() *
          rs.J_com() * vd_;
  tot = c;
  std::cout << "com cost: " << c << std::endl;

  c = 0.5 * vd_.transpose() * input.w_pelv * rs.pelv().J().transpose() *
          rs.pelv().J() * vd_ +
      input.w_pelv * (rs.pelv().Jdot_times_v() - input.pelvdd_d()).transpose() *
          rs.pelv().J() * vd_;
  tot += c;
  std::cout << "pelv cost: " << c << std::endl;

  c = 0.5 * vd_.transpose() * input.w_torso * rs.torso().J().transpose() *
          rs.torso().J() * vd_ +
      input.w_torso * (rs.torso().Jdot_times_v() - input.torsodd_d()).transpose() *
          rs.torso().J() * vd_;
  tot += c;
  std::cout << "torso cost: " << c << std::endl;

  c = 0.5 * vd_.transpose() * input.w_vd * vd_ +
      input.w_vd * (-input.vd_d()).transpose() * vd_;
  tot += c;
  std::cout << "vd_ cost: " << c << std::endl;

  for (int i = 0; i < 2; i++) {
    c = 0.5 * foot_wrench_in_world_frame_[i].transpose() *
            input.w_wrench_reg * foot_wrench_in_world_frame_[i] +
        input.w_wrench_reg * (-input.wrench_d(i)).transpose() *
            foot_wrench_in_world_frame_[i];
    tot += c;
    std::cout << "wrench cost " << i << ": " << c << std::endl;
  }

  std::cout << "total cost: " << tot << std::endl;
  return tot[0];
}

std::ostream& operator<< (std::ostream &out, const QPInput& input) {
  out << "===============================================\n";
  out << "comdd_d: " << input.comdd_d().transpose() << std::endl;
  out << "pelvdd_d: " << input.pelvdd_d().transpose() << std::endl;
  out << "torsodd_d: " << input.torsodd_d().transpose() << std::endl;
  out << "footdd_d[0]: " << input.footdd_d(0).transpose() << std::endl;
  out << "footdd_d[1]: " << input.footdd_d(1).transpose() << std::endl;
  out << "vd_d: " << input.vd_d().transpose() << std::endl;
  out << "wrench_d[0]: " << input.wrench_d(0).transpose() << std::endl;
  out << "wrench_d[1]: " << input.wrench_d(1).transpose() << std::endl;

  out << "w_com: " << input.w_com << std::endl;
  out << "w_pelv: " << input.w_pelv << std::endl;
  out << "w_torso: " << input.w_torso << std::endl;
  out << "w_foot: " << input.w_foot << std::endl;
  out << "w_vd: " << input.w_vd << std::endl;
  out << "w_wrench_reg: " << input.w_wrench_reg << std::endl;
  return out;
}

std::ostream& operator<< (std::ostream &out, const QPOutput& output) {
  out << "===============================================\n";
  out << "accelerations:\n";
  for (int i = 0; i < output.vd().size(); i++)
    out << output.coord_name(i) << ": " << output.vd()[i] << std::endl;

  out << "com acc: ";
  out << output.comdd().transpose() << std::endl;

  out << "pelv acc: ";
  out << output.pelvdd().transpose() << std::endl;

  out << "torso acc: ";
  out << output.torsodd().transpose() << std::endl;

  out << "left foot acc: " << output.footdd(Side::LEFT).transpose()
            << std::endl;
  out << "right foot acc: " << output.footdd(Side::RIGHT).transpose()
            << std::endl;

  out << "===============================================\n";
  out << "left foot wrench_w: "
            << output.foot_wrench_in_world_frame(Side::LEFT).transpose()
            << std::endl;
  out << "right foot wrench_w: "
            << output.foot_wrench_in_world_frame(Side::RIGHT).transpose()
            << std::endl;
  out << "left foot wrench in sensor frame: "
            << output.foot_wrench_in_sensor_frame(Side::LEFT).transpose()
            << std::endl;
  out << "right foot wrench in sensor frame: "
            << output.foot_wrench_in_sensor_frame(Side::RIGHT).transpose()
            << std::endl;

  out << "===============================================\n";
  out << "torque:\n";
  for (int i = 0; i < output.joint_torque().size(); i++)
    out << output.coord_name(i+6) << ": " << output.joint_torque()[i]
              << std::endl;
  out << "===============================================\n";
  return out;
}


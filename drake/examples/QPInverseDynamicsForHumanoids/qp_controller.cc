#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"

#include "drake/math/cross_product.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

const double QPController::kUpperBoundForContactBasis = 1000;

static void FindCostAndEqConstraintRows(const Eigen::VectorXd& weights, std::list<int>* cost_idx, std::list<int>* eq_idx) {
  cost_idx->clear();
  eq_idx->clear();
  for (int i = 0; i < weights.size(); i++) {
    if (weights[i] > 0)
      cost_idx->push_back(i);
    else if (weights[i] < 0)
      eq_idx->push_back(i);
  }
}

void QPController::ResizeQP(
    const RigidBodyTree& robot,
    const QPInput& input) {
  const std::list<ContactInformation>& all_contacts = input.contact_info();
  const std::map<std::string, DesiredBodyMotion>& all_body_motions = input.desired_body_motions();
  const DesiredJointMotions& all_joint_motions = input.desired_joint_motions();
  const DesiredCentroidalMomentumChange& cen_mom_change = input.desired_centroidal_momentum_change();
  // Figure out dimensions.
  int num_contact_body = all_contacts.size();
  int num_vd = robot.get_num_velocities();
  int num_basis = 0;
  int num_point_force = 0;
  for (const ContactInformation& contact : all_contacts) {
    num_point_force += contact.contact_points().size();
    num_basis += contact.num_basis();
  }
  int num_torque = robot.actuators.size();
  int num_variable = num_vd + num_basis;

  // Figure out size of the constrained dimensions of body motions.
  int num_body_motion_as_cost = 0;
  int num_body_motion_as_eq = 0;
  std::vector<std::list<int>> body_motion_row_idx_as_cost(all_body_motions.size());
  std::vector<std::list<int>> body_motion_row_idx_as_eq(all_body_motions.size());
  int ctr = 0;
  for (auto it = all_body_motions.begin(); it != all_body_motions.end(); it++) {
    const DesiredBodyMotion& body_motion = it->second;
    FindCostAndEqConstraintRows(body_motion.weights(), &body_motion_row_idx_as_cost[ctr], &body_motion_row_idx_as_eq[ctr]);

    if (body_motion_row_idx_as_cost[ctr].size() > 0)
      num_body_motion_as_cost++;
    if (body_motion_row_idx_as_eq[ctr].size() > 0)
      num_body_motion_as_eq++;
    ctr++;
  }

  // Figure out size of the constrained dimensions of desired joint motions.
  std::list<int> cost_row_idx;
  std::list<int> eq_row_idx;
  FindCostAndEqConstraintRows(all_joint_motions.weights(), &cost_row_idx, &eq_row_idx);
  int num_joint_motion_as_cost = (cost_row_idx.size() > 0) ? 1 : 0;
  int num_joint_motion_as_eq = (eq_row_idx.size() > 0) ? 1 : 0;

  // Figure out size of the constrained dimensions of centroidal momentum change.
  FindCostAndEqConstraintRows(cen_mom_change.weights(), &cost_row_idx, &eq_row_idx);
  int num_cen_mom_chg_as_cost = (cost_row_idx.size() > 0) ? 1 : 0;
  int num_cen_mom_chg_as_eq = (eq_row_idx.size() > 0) ? 1 : 0;

  // Structure of the QP remains the same, no need to make a new MathematicalProgram.
  if (num_contact_body == num_contact_body_ && num_vd == num_vd_ &&
      num_basis == num_basis_ && num_point_force == num_point_force_ &&
      num_torque == num_torque_ && num_variable == num_variable_ &&
      num_body_motion_as_cost == num_body_motion_as_cost_ &&
      num_body_motion_as_eq == num_body_motion_as_eq_ &&
      num_joint_motion_as_cost == num_joint_motion_as_cost_ &&
      num_joint_motion_as_eq == num_joint_motion_as_eq_ &&
      num_joint_motion_as_eq == num_joint_motion_as_eq_ &&
      num_joint_motion_as_cost == num_joint_motion_as_cost_) {
    return;
  }

  num_contact_body_ = num_contact_body;
  num_vd_ = num_vd;
  num_basis_ = num_basis;
  num_point_force_ = num_point_force;
  num_torque_ = num_torque;
  num_variable_ = num_variable;
  num_body_motion_as_cost_ = num_body_motion_as_cost;
  num_body_motion_as_eq_ = num_body_motion_as_eq;
  num_joint_motion_as_cost_ = num_joint_motion_as_cost;
  num_joint_motion_as_eq_ = num_joint_motion_as_eq;
  num_cen_mom_chg_as_cost_ = num_cen_mom_chg_as_cost;
  num_cen_mom_chg_as_eq_ = num_cen_mom_chg_as_eq;

  std::cout << "num_contact_body_ " << num_contact_body_ << std::endl;
  std::cout << "num_vd_ " << num_vd_ << std::endl;
  std::cout << "num_basis_ " << num_basis_ << std::endl;
  std::cout << "num_point_force_ " << num_point_force_ << std::endl;
  std::cout << "num_torque_ " << num_torque_ << std::endl;
  std::cout << "num_variable_ " << num_variable_ << std::endl;
  std::cout << "num_body_motion_as_cost_ " << num_body_motion_as_cost_ << std::endl;
  std::cout << "num_body_motion_as_eq_ " << num_body_motion_as_eq_ << std::endl;
  std::cout << "num_joint_motion_as_cost_ " << num_joint_motion_as_cost_ << std::endl;
  std::cout << "num_joint_motion_as_eq_ " << num_joint_motion_as_eq_ << std::endl;
  std::cout << "num_cen_mom_chg_as_cost_ " << num_cen_mom_chg_as_cost_ << std::endl;
  std::cout << "num_cen_mom_chg_as_eq_ " << num_cen_mom_chg_as_eq_ << std::endl;

  // The order of insertion is important, the rest of the program assumes this
  // layout.
  prog_ = solvers::MathematicalProgram();
  solvers::DecisionVariableView vd =
      prog_.AddContinuousVariables(num_vd_, "vd");
  solvers::DecisionVariableView basis =
      prog_.AddContinuousVariables(num_basis_, "basis");

  // Allocate space for contact force jacobian and basis matrix.
  stacked_contact_jacobians_.resize(3 * num_point_force_, num_vd_);
  basis_to_force_matrix_.resize(3 * num_point_force_, num_basis_);
  stacked_contact_jacobians_dot_times_v_.resize(3 * num_point_force_);
  torque_linear_.resize(num_torque_, num_variable_);
  dynamics_linear_.resize(6, num_variable_);

  // Allocate equality constraints.
  // Dyanmics
  eq_dynamics_ =
      prog_.AddLinearEqualityConstraint(Eigen::MatrixXd::Zero(6, num_variable_),
                                        Eigen::Vector6d::Zero(), {vd, basis});
  eq_dynamics_->set_description("dynamics eq");

  // Contact constraints, 3 rows per contact point
  // TODO(siyuan.feng@tri.global): switch this to cost eventually.
  eq_contacts_.resize(num_contact_body_);
  ctr = 0;
  for (const ContactInformation& contact : all_contacts) {
    eq_contacts_[ctr++] = prog_.AddLinearEqualityConstraint(
        Eigen::MatrixXd::Zero(3 * contact.contact_points().size(),
                              num_vd_),
        Eigen::VectorXd::Zero(3 * contact.contact_points().size()),
        {vd});
  }

  // Allocate inequality constraints.
  // Contact force scalar (Beta), which is constant and does not depend on the
  // robot configuration.
  ineq_contact_wrench_ = prog_.AddLinearConstraint(
      Eigen::MatrixXd::Identity(num_basis_, num_basis_),
      Eigen::VectorXd::Zero(num_basis_),
      Eigen::VectorXd::Constant(num_basis_, kUpperBoundForContactBasis),
      {basis});
  ineq_contact_wrench_->set_description("contact force basis ineq");
  // Torque limit
  ineq_torque_limit_ = prog_.AddLinearConstraint(
      Eigen::MatrixXd::Zero(num_torque_, num_variable_),
      Eigen::VectorXd::Zero(num_torque_), Eigen::VectorXd::Zero(num_torque_),
      {vd, basis});
  ineq_torque_limit_->set_description("torque limit ineq");

  // Allocate cost terms.
  tmp_vd_vec_.resize(num_vd_);
  tmp_vd_mat_.resize(num_vd_, num_vd_);

  // Setup cost / eq constraints for centroidal momentum change.
  if (num_cen_mom_chg_as_cost_) {
    cost_cen_mom_chg_ = prog_.AddQuadraticCost(tmp_vd_mat_, tmp_vd_vec_, {vd});
    cost_cen_mom_chg_->set_description("centroidal momentum change cost");
  } else {
    cost_cen_mom_chg_ = nullptr;
  }
  if (num_cen_mom_chg_as_eq_) {
    // Dimension doesn't matter, will be reset when updating the constaint.
    eq_cen_mom_chg_ = prog_.AddLinearEqualityConstraint(tmp_vd_mat_, tmp_vd_vec_, {vd});
    eq_cen_mom_chg_->set_description("centroidal momentum change eq");
  } else {
    eq_cen_mom_chg_ = nullptr;
  }

  // Setup cost / eq constraints for body motion.
  body_Jdv_.resize(all_body_motions.size());
  body_J_.resize(all_body_motions.size());
  cost_body_motion_.resize(num_body_motion_as_cost_);
  eq_body_motion_.resize(num_body_motion_as_eq_);
  int cost_ctr = 0, eq_ctr = 0;
  for (const auto& row_idx_as_cost : body_motion_row_idx_as_cost) {
    if (row_idx_as_cost.size() > 0) {
      cost_body_motion_[cost_ctr++] = prog_.AddQuadraticCost(tmp_vd_mat_, tmp_vd_vec_, {vd});
    }
  }
  for (const auto& row_idx_as_eq : body_motion_row_idx_as_eq) {
    if (row_idx_as_eq.size() > 0) {
      // Dimension doesn't matter, will be reset when updating the constaint.
      eq_body_motion_[eq_ctr++] = prog_.AddLinearEqualityConstraint(tmp_vd_mat_, tmp_vd_vec_, {vd});
    }
  }
  DRAKE_ASSERT(cost_ctr == num_body_motion_as_cost_);
  DRAKE_ASSERT(eq_ctr == num_body_motion_as_eq_);

  // Setup cost / eq constraints for joint motion.
  if (num_joint_motion_as_cost_ > 0) {
    cost_joint_motion_ = prog_.AddQuadraticCost(tmp_vd_mat_, tmp_vd_vec_, {vd});
    cost_joint_motion_->set_description("vd cost");
  } else {
    cost_joint_motion_ = nullptr;
  }
  if (num_joint_motion_as_eq_ > 0) {
    // Dimension doesn't matter, will be reset when updating the constaint.
    eq_joint_motion_ = prog_.AddLinearEqualityConstraint(tmp_vd_mat_, tmp_vd_vec_, {vd});
    eq_joint_motion_->set_description("vd eq");
  } else {
    eq_joint_motion_ = nullptr;
  }

  // Regularize basis.
  cost_basis_reg_ =
      prog_.AddQuadraticCost(Eigen::MatrixXd::Identity(num_basis_, num_basis_),
                             Eigen::VectorXd::Zero(num_basis_), {basis});
  cost_basis_reg_->set_description("basis reg cost");
  basis_reg_mat_ = Eigen::MatrixXd::Identity(num_basis_, num_basis_);
  basis_reg_vec_ = Eigen::VectorXd::Zero(num_basis_);
}

int QPController::Control(const HumanoidStatus& rs, const QPInput& input,
                          QPOutput* output) {
  if (!input.is_valid(rs.robot().get_num_velocities())) {
    std::cerr << "input is invalid\n";
    return -1;
  }

  // Resize and zero temporary matrices.
  ResizeQP(rs.robot(), input);
  SetTempMatricesToZero();

  ////////////////////////////////////////////////////////////////////
  // The equations of motion look like:
  // M(q) * vd + h(q,v) = S * tau + J^T * lambda
  // M(q) is the inertia matrix, h(q,v) is the gravitational and centrifugal
  // force, vd is acceleration, S is the selection matrix (top 6 rows are
  // zeros due to the floating base), tau is joint torque, J^T is the transpose
  // of all contact Jacobian, and lambda is the contact wrench in the world
  // frame.
  // In this implementation, lambda is replaced by a set of point forces
  // applied at different contact points per each contact link.
  // The equations of motion is updated to:
  // M(q) * vd + h(q,v) = S * tau + J^T * basis * Beta
  //
  // For inverse dynamics, we are usually given desired motions, and
  // we want to solve for tau to achieve those motions.
  // Desired motions can be directly specified as desired_vd, or as
  // acceleration_d in Cartesian
  // space, which is linear w.r.t. vd as well: acceleration_d = J * vd + Jd * v.
  //
  // Note that since S.topRows(6) is zero,
  // tau = M_l * vd + h_l - (J^T * basis)_l * Beta
  // where _l means the lower num_torque_ rows of those matrices.
  // So we just need to solve for vd and Beta, and tau can be computed as
  // above. We can formulate inverse dynamics a QP problem.
  //
  // For the QP problem:
  // the unknown is _X = [vd, Beta]
  // equality constraints:
  //  M_u * vd + h_u = (J^T * basis)_u * Beta (equations of motion)
  //  J * vd + Jd * v = 0, (contact constraints)
  // inEquality: joint torque limit, limits on Beta, etc.
  // cost func:
  //  min (Jcom*vd + Jcomd*v - desired_comdd)^2
  //    + (vd - desired_vd)^2
  //    + all_kinds_of_body_acceleration_cost_terms (pelvis, torso, feet, etc)
  //
  // I made the dynamics and stationary contact equality constraints.
  // Alternatively, they can be setup as high weight cost terms. This is
  // sometimes preferred as it introduce slacks for better stability.
  //
  // For more details on the QP setup, the interested readers could refer to
  // [1]  An efficiently solvable quadratic program for stabilizing dynamic
  // locomotion, Scott Kuindersma, Frank Permenter, and Russ Tedrake
  // http://groups.csail.mit.edu/robotics-center/public_papers/Kuindersma13.pdf
  const solvers::DecisionVariableView vd = prog_.GetVariable("vd");
  const solvers::DecisionVariableView basis = prog_.GetVariable("basis");

  int basis_start = basis.index();
  int vd_start = vd.index();

  // Stack the contact Jacobians and basis matrices for each contact link.
  int rowIdx = 0;
  int colIdx = 0;
  for (const ContactInformation& contact : input.contact_info()) {
    int force_dim = 3 * contact.contact_points().size();
    int basis_dim = contact.num_basis();
    basis_to_force_matrix_.block(rowIdx, colIdx, force_dim, basis_dim) =
        contact.ComputeBasisMatrix(rs.robot(), rs.cache());
    stacked_contact_jacobians_.block(rowIdx, 0, force_dim, num_vd_) =
        contact.ComputeJacobianAtContactPoints(rs.robot(), rs.cache());
    stacked_contact_jacobians_dot_times_v_.segment(rowIdx, force_dim) =
        contact.ComputeJacobianDotTimesVAtContactPoints(rs.robot(), rs.cache());
    rowIdx += force_dim;
    colIdx += basis_dim;
  }
  JB_ = stacked_contact_jacobians_.transpose() * basis_to_force_matrix_;
  DRAKE_ASSERT(rowIdx == num_point_force_ * 3);
  DRAKE_ASSERT(colIdx == num_basis_);

  // tau = M_l * vd + h_l - (J^T * basis)_l * Beta
  // tau = torque_linear_ * X + torque_constant_
  torque_linear_.block(0, vd_start, num_torque_, num_vd_) =
      rs.M().bottomRows(num_torque_);
  torque_linear_.block(0, basis_start, num_torque_, num_basis_) =
      -JB_.bottomRows(num_torque_);
  torque_constant_ = rs.bias_term().tail(num_torque_);

  ////////////////////////////////////////////////////////////////////
  // Equality constraints:
  // Equations of motion part, 6 rows
  dynamics_linear_.block(0, vd_start, 6, num_vd_) = rs.M().topRows(6);
  dynamics_linear_.block(0, basis_start, 6, num_basis_) = -JB_.topRows(6);
  dynamics_constant_ = -rs.bias_term().head<6>();
  eq_dynamics_->UpdateConstraint(dynamics_linear_, dynamics_constant_);

  // Contact constraints, 3 rows per contact point
  rowIdx = 0;
  int ctr = 0;
  for (const ContactInformation& contact : input.contact_info()) {
    int force_dim = 3 * contact.contact_points().size();
    eq_contacts_[ctr]->UpdateConstraint(
        stacked_contact_jacobians_.block(rowIdx, 0, force_dim, num_vd_),
        -stacked_contact_jacobians_dot_times_v_.segment(rowIdx, force_dim));
    eq_contacts_[ctr]->set_description(contact.name() + " contact eq");
    rowIdx += force_dim;
    ctr++;
  }
  DRAKE_ASSERT(rowIdx == num_point_force_ * 3);

  ////////////////////////////////////////////////////////////////////
  // Inequality constraints:
  // For the contact point force basis, the constraints are always > 0,
  // so the stay constant as in ResizeQP.

  // Torque limits: min <= tau <= max, num_torque_ rows
  // min <= M_l * vd + h_l - (J^T * basis)_l * Beta <= max
  // min - h_l <= M_l * vd - (J^T * basis)_l * Beta <= max - h_l
  // tau = rs.robot->B.bottomRows(num_torque_) * u,
  // u = rs.robot->B.bottomRows(num_torque_).transpose() * tau
  // since B should be orthonormal.
  // Tau is joint space indexed, and u is actuator space indexed.
  // constraints are specified with u index.
  inequality_linear_ =
      rs.robot().B.bottomRows(num_torque_).transpose() * torque_linear_;
  inequality_upper_bound_ = inequality_lower_bound_ =
      -rs.robot().B.bottomRows(num_torque_).transpose() * torque_constant_;
  for (size_t i = 0; i < rs.robot().actuators.size(); i++) {
    inequality_lower_bound_[i] += rs.robot().actuators[i].effort_limit_min_;
    inequality_upper_bound_[i] += rs.robot().actuators[i].effort_limit_max_;
  }
  ineq_torque_limit_->UpdateConstraint(
      inequality_linear_, inequality_lower_bound_, inequality_upper_bound_);

  ////////////////////////////////////////////////////////////////////
  // Cost function:
  // CoM term (task space acceleration costs)
  // w * (J * vd + Jdv - desired_comdd)^T * (J * vd + Jdv - desired_comdd)
  std::list<int> row_idx_as_cost;
  std::list<int> row_idx_as_eq;
  FindCostAndEqConstraintRows(input.desired_centroidal_momentum_change().weights(), &row_idx_as_cost, &row_idx_as_eq);
  Eigen::Vector6d linear_term = rs.centroidal_momentum_matrix_dot_times_v() - input.desired_centroidal_momentum_change().momentum_dot();
  AddAsCosts(rs.centroidal_momentum_matrix(), linear_term, input.desired_centroidal_momentum_change().weights(), row_idx_as_cost, cost_cen_mom_chg_);
  AddAsConstraints(rs.centroidal_momentum_matrix(), -linear_term, row_idx_as_eq, eq_cen_mom_chg_);

  // Body motion
  int contact_ctr = 0, cost_ctr = 0, eq_ctr = 0;
  for (auto it = input.desired_body_motions().begin(); it != input.desired_body_motions().end(); it++) {
    const DesiredBodyMotion& body_motion_d = it->second;
    body_J_[contact_ctr] = GetTaskSpaceJacobian(
        rs.robot(), rs.cache(), *body_motion_d.body(), Eigen::Vector3d::Zero());
    body_Jdv_[contact_ctr] = GetTaskSpaceJacobianDotTimesV(
        rs.robot(), rs.cache(), *body_motion_d.body(), Eigen::Vector3d::Zero());
    linear_term = body_Jdv_[contact_ctr] - body_motion_d.accelerations();

    // Find the rows that correspond to cost and equality constraints.
    FindCostAndEqConstraintRows(body_motion_d.weights(), &row_idx_as_cost, &row_idx_as_eq);
    if (!row_idx_as_cost.empty()) {
      AddAsCosts(body_J_[contact_ctr], linear_term, body_motion_d.weights(), row_idx_as_cost, cost_body_motion_[cost_ctr]);
      cost_body_motion_[cost_ctr]->set_description(body_motion_d.name() + " cost");
      cost_ctr++;
    }
    if (!row_idx_as_eq.empty()) {
      AddAsConstraints(body_J_[contact_ctr], -linear_term, row_idx_as_eq, eq_body_motion_[eq_ctr]);
      eq_body_motion_[eq_ctr]->set_description(body_motion_d.name() + " eq");
      eq_ctr++;
    }
    contact_ctr++;
  }

  // Joint motion
  FindCostAndEqConstraintRows(input.desired_joint_motions().weights(), &row_idx_as_cost, &row_idx_as_eq);
  // Process eq constraints.
  if (row_idx_as_eq.size() > 0) {
    int row_ctr = 0;
    for (int d : row_idx_as_eq) {
      tmp_vd_mat_.row(row_ctr).setZero();
      tmp_vd_mat_(row_ctr, d) = 1;
      tmp_vd_vec_[row_ctr] = input.desired_joint_motions().accelerations()[d];
      row_ctr++;
    }
    eq_joint_motion_->UpdateConstraint(tmp_vd_mat_.topRows(row_ctr), tmp_vd_vec_.head(row_ctr));
  }
  // Procecss cost terms.
  if (row_idx_as_cost.size() > 0) {
    tmp_vd_mat_.setZero();
    tmp_vd_vec_.setZero();

    for (int d : row_idx_as_cost) {
      double weight = input.desired_joint_motions().weights()[d];
      tmp_vd_mat_(d, d) = weight;
      tmp_vd_vec_[d] = -weight * input.desired_joint_motions().accelerations()[d];
    }
    cost_joint_motion_->UpdateQuadraticAndLinearTerms(tmp_vd_mat_, tmp_vd_vec_);
  }

  // Regularize basis to zero.
  cost_basis_reg_->UpdateQuadraticAndLinearTerms(
      input.w_basis_reg() * basis_reg_mat_,
      basis_reg_vec_);

  ////////////////////////////////////////////////////////////////////
  // Call solver.
  if (!solver_.available()) {
    std::cerr << "Solver not available.\n";
    return -1;
  }
  solvers::SolutionResult result = solver_.Solve(prog_);
  if (result != solvers::SolutionResult::kSolutionFound) {
    std::cerr << "solution not found\n";
    return -1;
  }
  solution_ = prog_.GetSolutionVectorValues();

  ////////////////////////////////////////////////////////////////////
  // Examples of inspecting each cost / eq, ineq term
  auto costs = prog_.quadratic_costs();
  auto eqs = prog_.linear_equality_constraints();
  auto ineqs = prog_.linear_constraints();

  output->mutable_costs().resize(costs.size());
  ctr = 0;
  Eigen::VectorXd tmp_vec;
  for (auto& cost_b : costs) {
    std::shared_ptr<solvers::Constraint> cost = cost_b.constraint();
    cost->Eval(cost_b.VariableListToVectorXd(), tmp_vec);
    output->mutable_cost(ctr).first = cost->get_description();
    output->mutable_cost(ctr).second = tmp_vec(0);
    ctr++;
  }

  for (auto& eq_b : eqs) {
    std::shared_ptr<solvers::LinearEqualityConstraint> eq = eq_b.constraint();
    DRAKE_ASSERT((eq->A() * eq_b.VariableListToVectorXd() - eq->lower_bound()).isZero(EPSILON));
  }

  for (auto& ineq_b : ineqs) {
    std::shared_ptr<solvers::LinearConstraint> ineq = ineq_b.constraint();
    tmp_vec = ineq->A() * ineq_b.VariableListToVectorXd();
    for (int i = 0; i < tmp_vec.size(); i++) {
      DRAKE_ASSERT(tmp_vec[i] >= ineq->lower_bound()[i] - EPSILON &&
                   tmp_vec[i] <= ineq->upper_bound()[i] + EPSILON);
    }
  }

  ////////////////////////////////////////////////////////////////////
  // Parse results.
  // Compute resulting contact wrenches.
  int basis_index = 0;
  int point_force_index = 0;
  point_forces_ = basis_to_force_matrix_ * basis.value();

  output->mutable_resolved_contacts().clear();
  for (const ContactInformation& contact : input.contact_info()) {
    ResolvedContact resolved_contact(contact.body());

    // Copy basis.
    resolved_contact.mutable_basis() =
        basis.value().segment(basis_index, contact.num_basis());
    basis_index += contact.num_basis();

    // Compute contact points and reference point in the world frame.
    contact.ComputeContactPointsAndWrenchReferencePoint(
        rs.robot(), rs.cache(), Eigen::Vector3d::Zero(),
        &resolved_contact.mutable_contact_points(),
        &resolved_contact.mutable_reference_point());

    // Convert point forces to an equivalent wrench wrt to the reference point
    // in the world frame.
    resolved_contact.mutable_equivalent_wrench() =
        contact.ComputeWrenchMatrix(resolved_contact.contact_points(),
                                         resolved_contact.reference_point()) *
        point_forces_.segment(point_force_index,
                              3 * contact.num_contact_points());

    // Copy point forces.
    resolved_contact.mutable_point_forces().resize(
        contact.num_contact_points());
    for (Eigen::Vector3d& point_force :
         resolved_contact.mutable_point_forces()) {
      point_force = point_forces_.segment<3>(point_force_index);
      point_force_index += 3;
    }

    // Add to output.
    output->mutable_resolved_contacts().push_back(resolved_contact);
  }

  // Set output accelerations.
  output->mutable_vd() = vd.value();
  output->mutable_comdd() = rs.J_com() * output->vd() + rs.Jdot_times_v_com();

  output->mutable_body_accelerations().clear();
  ctr = 0;
  for (auto it = input.desired_body_motions().begin(); it != input.desired_body_motions().end(); it++) {
    const DesiredBodyMotion& body_motion_d = it->second;
    BodyAcceleration body_acceleration(body_motion_d.body());
    // Compute accelerations.
    body_acceleration.mutable_acceleration() =
        body_J_[ctr] * output->vd() + body_Jdv_[ctr];

    // Add to output.
    output->mutable_body_accelerations().push_back(body_acceleration);
    ctr++;
  }

  // Set output joint torques.
  output->mutable_joint_torque() = torque_linear_ * solution_ + torque_constant_;

  ////////////////////////////////////////////////////////////////////
  // Sanity check:
  // Net external wrench = centroidal_matrix * vd + centroidal_matrix_dot * v
  Eigen::Vector6d Ld = rs.centroidal_momentum_matrix() * output->vd() +
                       rs.centroidal_momentum_matrix_dot_times_v();
  Eigen::Vector6d net_wrench = rs.robot().getMass() * rs.robot().a_grav;
  for (const ResolvedContact& resolved_contact : output->resolved_contacts()) {
    const Eigen::Vector6d& contact_wrench =
        resolved_contact.equivalent_wrench();
    const Eigen::Vector3d& ref_point = resolved_contact.reference_point();
    net_wrench += contact_wrench;
    net_wrench.segment<3>(0) +=
        (ref_point - rs.com()).cross(contact_wrench.segment<3>(3));
  }
  DRAKE_ASSERT((net_wrench - Ld).isZero(EPSILON));

  if (!output->is_valid(rs.robot().get_num_velocities(),
                        rs.robot().actuators.size())) {
    std::cerr << "output is invalid\n";
    return -1;
  }

  return 0;
}






std::ostream& operator<<(std::ostream& out, const QPInput& input) {
  out << "===============================================\n";
  out << "QPInput:\n";
  out << input.desired_centroidal_momentum_change();

  for (auto it = input.desired_body_motions().begin(); it != input.desired_body_motions().end(); it++)
    out << it->second;

  out << input.desired_joint_motions();

  out << "weight_basis_reg: " << input.w_basis_reg() << std::endl;

  for (const ContactInformation& contact : input.contact_info())
    out << contact;

  return out;
}

std::ostream& operator<<(std::ostream& out, const QPOutput& output) {
  out << "===============================================\n";
  out << "QPOutput:\n";
  out << "accelerations:\n";
  for (int i = 0; i < output.vd().size(); i++) {
    out << output.coord_name(i) << ": " << output.vd()[i] << std::endl;
  }

  out << "com acc: ";
  out << output.comdd().transpose() << std::endl;

  for (const BodyAcceleration& body_motion : output.body_accelerations()) {
    out << body_motion.name()
        << " acc: " << body_motion.acceleration().transpose() << std::endl;
  }

  out << "===============================================\n";
  for (const ResolvedContact& contact_result : output.resolved_contacts()) {
    out << contact_result.name()
        << " wrench: " << contact_result.equivalent_wrench().transpose()
        << std::endl;
    out << "point forces:\n";
    for (size_t j = 0; j < contact_result.point_forces().size(); j++) {
      out << contact_result.point_force(j).transpose() << " at "
          << contact_result.contact_point(j).transpose() << std::endl;
    }
  }

  out << "===============================================\n";
  out << "torque:\n";
  for (int i = 0; i < output.joint_torque().size(); i++) {
    out << output.coord_name(i + 6) << ": " << output.joint_torque()[i]
        << std::endl;
  }
  out << "===============================================\n";
  out << "costs:\n";
  for (const std::pair<std::string, double>& cost : output.costs()) {
    out << cost.first << ": " << cost.second << std::endl;
  }

  return out;
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

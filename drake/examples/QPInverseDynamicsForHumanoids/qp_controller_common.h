#pragma once

#include <iostream>
#include <list>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/kinematics_cache.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * Enum class for constraint types.
 * Hard: will be enforced by equality constraints.
 * Soft: will be enforced by cost functions.
 */
enum class ConstraintType { Hard = -1, Skip = 0, Soft = 1 };

std::ostream& operator<<(std::ostream& out, const ConstraintType& type);

/**
 * Base class for specifying various desired objectives.
 * The objectives can be ignored, set as equality constraints or optimized as
 * cost terms depending on the specified types.
 * For cost terms, a positive weight needs to be specified.
 */
class ConstrainedValues {
 private:
  std::vector<ConstraintType> constraint_types_;
  VectorX<double> weights_;
  VectorX<double> values_;

 public:
  ConstrainedValues() {}

  explicit ConstrainedValues(int dim)
      : constraint_types_(dim, ConstraintType::Skip),
        weights_(VectorX<double>::Zero(dim)),
        values_(VectorX<double>::Zero(dim)) {}

  void resize(int dim);

  void SetAllConstraintTypesBasedOnWeights();

  /**
   * Get all the indices that has the specified constraint type.
   * @param type Matching constraint type
   * @return indices
   */
  std::list<int> GetConstraintTypeIndices(ConstraintType type) const;

  /**
   * Set given indices' constraint types to the given type.
   * @param indices List of indices
   * @param type Desired type
   */
  void SetConstraintType(const std::list<int>& indices, ConstraintType type);

  /**
   * Set all constraint types to the given type.
   * @param type Desired type
   */
  void SetAllConstraintType(ConstraintType type);

  bool is_valid() const { return is_valid(size()); }

  bool is_valid(int dim) const;

  bool operator==(const ConstrainedValues& other) const;

  bool operator!=(const ConstrainedValues& other) const {
    return !(this->operator==(other));
  }

  // Getters
  inline int size() const { return values_.size(); }
  inline const VectorX<double>& weights() const { return weights_; }
  inline const VectorX<double>& values() const { return values_; }
  inline const std::vector<ConstraintType>& constraint_types() const {
    return constraint_types_;
  }

  inline double value(int i) const { return values_[i]; }
  inline double weight(int i) const { return weights_[i]; }
  inline ConstraintType constraint_type(int i) const {
    return constraint_types_.at(i);
  }

  // Setters
  inline VectorX<double>& mutable_weights() { return weights_; }
  inline VectorX<double>& mutable_values() { return values_; }
  inline std::vector<ConstraintType>& mutable_constraint_types() {
    return constraint_types_;
  }

  inline double& mutable_value(int i) { return values_[i]; }
  inline double& mutable_weight(int i) { return weights_[i]; }
  inline ConstraintType& mutable_constraint_type(int i) {
    return constraint_types_.at(i);
  }
};

/**
 * This class describes contact related information for each body in contact
 * with the world.
 * Each contact body has a set of point contacts. For each contact point,
 * only point contact force can be applied, and the friction cone is
 * approximated by a set of basis vectors.
 *
 * The stationary contact (small foot acceleration) condition can be described
 * as:
 * J * vd + J_dot_times_vd = Kd * (0 - v_contact_pt).
 * Only the linear velocities and accelerations are considered here.
 * Kd >= 0 is a stabilizing velocity gain to damp out contact velocity
 * This condition can be enforced either as an equality constraint or as a
 * cost term.
 */
class ContactInformation {
 public:
  static const int kDefaultNumBasisPerContactPoint = 4;

  /*
   * @param body Reference to a RigidBody, which must be valid through the
   * lifespan of this obejct.
   * @param num_basis_per_contact_point number of basis per contact point
   */
  ContactInformation(const RigidBody<double>& body,
                     int num_basis = kDefaultNumBasisPerContactPoint);

  /**
   * Computes a matrix (Basis) that converts a vector of scalars (Beta) to
   * the stacked point contact forces (F).
   * All the point forces are in the world frame, and are applied at
   * the contact points in the world frame.
   * Basis is 3 * number of contact points by
   * \param num_basis_per_contact_point * number of contact points.
   * @param robot model
   * @param cache that stores the kinematics information, needs to be
   * initialized first.
   * @return Basis matrix
   */
  MatrixX<double> ComputeBasisMatrix(
      const RigidBodyTree<double>& robot,
      const KinematicsCache<double>& cache) const;

  /**
   * Computes the contact points and reference point location in the world
   * frame.
   * @param robot Robot model
   * @param cache Stores the kinematics information, needs to be initialized
   * first.
   * @param offset Offset for the reference point expressed in body frame.
   * @param contact_points Output of the function. Holds the contact point
   * locations.
   * @param reference_point Output of the function. Holds the reference point
   * location.
   */
  void ComputeContactPointsAndWrenchReferencePoint(
      const RigidBodyTree<double>& robot, const KinematicsCache<double>& cache,
      const Vector3<double>& offset, drake::Matrix3X<double>* contact_points,
      Vector3<double>* reference_point) const;

  /**
   * Computes a matrix that converts a vector of stacked point forces
   * to an equivalent wrench in a frame that has the same orientation as the
   * world frame, but located at \param reference_point. The stacked point
   * forces are assumed to have the same order of \param contact_points.
   * @param contact_points where the point forces are applied at. These are in
   * the world frame.
   * @param referece_point the reference point for the equivalent wrench.
   * @return The matrix that converts point forces to an equivalent wrench.
   */
  MatrixX<double> ComputeWrenchMatrix(
      const Matrix3X<double>& contact_points,
      const Vector3<double>& reference_point) const;

  /**
   * Computes the stacked task space Jacobian matrix (only the position part)
   * for all the contact points.
   * @param robot Robot model
   * @param cache Stores the kinematics information, needs to be initialized
   * first.
   * @return The stacked Jacobian matrix
   */
  MatrixX<double> ComputeJacobianAtContactPoints(
      const RigidBodyTree<double>& robot,
      const KinematicsCache<double>& cache) const;

  /**
   * Computes the stacked task space Jacobian dot times v vector (only the
   * position part) for all the contact points.
   * @param robot Robot model
   * @param cache Stores the kinematics information, needs to be initialized
   * first.
   * @return The stacked Jacobian dot times v vector
   */
  VectorX<double> ComputeJacobianDotTimesVAtContactPoints(
      const RigidBodyTree<double>& robot,
      const KinematicsCache<double>& cache) const;

  /**
   * Computes the stacked velocities for all the contact points.
   * @param robot Robot model
   * @param cache Stores the kinematics information, needs to be initialized
   * first.
   * @return Stacked velocities.
   */
  VectorX<double> ComputeLinearVelocityAtContactPoints(
      const RigidBodyTree<double>& robot,
      const KinematicsCache<double>& cache) const;

  bool is_valid() const;

  bool operator==(const ContactInformation& other) const;

  inline bool operator!=(const ContactInformation& other) const {
    return !(this->operator==(other));
  }

  inline const std::string& body_name() const { return body_->get_name(); }
  inline int num_contact_points() const { return contact_points_.cols(); }
  inline int num_basis() const {
    return num_contact_points() * num_basis_per_contact_point_;
  }

  // Getters
  inline double mu() const { return mu_; }
  inline double weight() const { return weight_; }
  inline double Kd() const { return Kd_; }
  inline ConstraintType acceleration_constraint_type() const {
    return acceleration_constraint_type_;
  }
  inline const Matrix3X<double>& contact_points() const {
    return contact_points_;
  }

  inline const Vector3<double>& normal() const { return normal_; }
  inline const RigidBody<double>& body() const { return *body_; }
  inline int num_basis_per_contact_point() const {
    return num_basis_per_contact_point_;
  }

  // Setters
  inline Matrix3X<double>& mutable_contact_points() { return contact_points_; }
  inline double& mutable_mu() { return mu_; }
  inline double& mutable_weight() { return weight_; }
  inline double& mutable_Kd() { return Kd_; }
  inline ConstraintType& mutable_acceleration_constraint_type() {
    return acceleration_constraint_type_;
  }
  inline Vector3<double>& mutable_normal() { return normal_; }
  inline int& mutable_num_basis_per_contact_point() {
    return num_basis_per_contact_point_;
  }
  inline void set_body(const RigidBody<double>& body) { body_ = &body; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  const RigidBody<double>* body_;
  // Offsets of the contact point specified in the body frame.
  Matrix3X<double> contact_points_;

  // TODO(siyuan.feng): Normal is currently assumed to be the same for all
  // the contact points.
  // Contact normal specified in the body frame.
  Vector3<double> normal_;

  int num_basis_per_contact_point_;

  // Friction coeff
  double mu_;

  // Velocity gain for stabilizing contact
  double Kd_;

  // Weight for the cost function term, only useful when
  // acceleration_constraint_type_ == ConstraintType::Soft.
  double weight_;

  // Determines if all contact points' acceleration are enforced as equality
  // constraints or summed in a cost term.
  // This cannot be ConstraintType::Skip.
  ConstraintType acceleration_constraint_type_;
};

std::ostream& operator<<(std::ostream& out, const ContactInformation& contact);

/**
 * A wrapper class specifying desired body motion (acceleration) for a rigid
 * body and their corresponding weights for the QP.
 * The acceleration is expressed in a frame that has the same orientation as
 * the world, and located at the origin of the body.
 *
 * The first three elements for weights and accelerations are angular.
 *
 * The desired acceleration can be skipped, enforced as equality constraints
 * or optimized as a cost term depending on the constraint type.
 *
 * TODO: (siyuan.feng) Expand this to be expressed in other frame.
 * TODO: (siyuan.feng) Expand this to have policies (controllers).
 */
class DesiredBodyMotion : public ConstrainedValues {
 public:
  /**
   * @param body Reference to a RigidBody, which must be valid through the
   * lifespan of this obejct.
   */
  explicit DesiredBodyMotion(const RigidBody<double>& body)
      : ConstrainedValues(6), body_(&body), control_during_contact_(false) {}

  inline bool is_valid() const {
    return this->ConstrainedValues::is_valid(kTwistSize);
  }

  std::string get_row_name(int i) const;

  bool operator==(const DesiredBodyMotion& other) const;

  inline bool operator!=(const DesiredBodyMotion& other) const {
    return !(this->operator==(other));
  }

  // Getters
  inline const RigidBody<double>& body() const { return *body_; }
  inline const std::string& body_name() const { return body_->get_name(); }
  inline bool control_during_contact() const { return control_during_contact_; }

  // Setters
  inline bool& mutable_control_during_contact() {
    return control_during_contact_;
  }
  inline void set_body(const RigidBody<double>& body) { body_ = &body; }

 private:
  const RigidBody<double>* body_;

  // TODO(siyuan.feng) Actually implement this in the qp controller.
  bool control_during_contact_;
};

std::ostream& operator<<(std::ostream& out, const DesiredBodyMotion& input);

/**
 * A wrapper class specifying desired DoF (degree of freedom)
 * motions (acceleration) and their corresponding weights for the QP.
 *
 * The desired acceleration can be skipped, enforced as equality constraints
 * or optimized as a cost term depending on the constraint type.
 *
 * TODO: (siyuan.feng) Expand this to have policies (controllers).
 */
class DesiredDoFMotions : public ConstrainedValues {
 public:
  DesiredDoFMotions() {}
  explicit DesiredDoFMotions(const std::vector<std::string>& names)
      : ConstrainedValues(static_cast<int>(names.size())), dof_names_(names) {}

  inline bool is_valid() const {
    return this->DesiredDoFMotions::is_valid(size());
  }

  bool is_valid(int dim) const;

  bool operator==(const DesiredDoFMotions& other) const;

  inline bool operator!=(const DesiredDoFMotions& other) const {
    return !(this->operator==(other));
  }

  // Getters
  inline const std::vector<std::string>& dof_names() const {
    return dof_names_;
  }
  inline const std::string& dof_name(int i) const { return dof_names_.at(i); }

 private:
  std::vector<std::string> dof_names_;
};

std::ostream& operator<<(std::ostream& out, const DesiredDoFMotions& input);

/**
 * A wrapper class specifying desired centroidal momentum change and their
 * corresponding weights for the QP.
 * The change in momentum are expressed in the world frame.
 * The first three terms are angular.
 * Linear momentum change = com acceleration * mass.
 *
 * The desired centroidal momentum change can be skipped, enforced as
 * equality constraints or optimized as a cost term depending on the
 * constraint type.
 *
 * TODO: (siyuan.feng) Expand this to have policies (controllers).
 */
class DesiredCentroidalMomentumDot : public ConstrainedValues {
 public:
  DesiredCentroidalMomentumDot() : ConstrainedValues(kTwistSize) {}

  inline bool is_valid() const {
    return this->ConstrainedValues::is_valid(kTwistSize);
  }

  std::string get_row_name(int i) const;
};

std::ostream& operator<<(std::ostream& out,
                         const DesiredCentroidalMomentumDot& input);

/**
 * Input to the QP inverse dynamics controller
 */
class QPInput {
 public:
  explicit QPInput(const RigidBodyTree<double>& r);

  inline bool is_valid() const { return is_valid(desired_dof_motions_.size()); }

  /**
   * Checks validity of this QPInput.
   * @param num_vd Dimension of acceleration in the generalized coordinates.
   * @return true if this is valid.
   */
  bool is_valid(int num_vd) const;

  bool operator==(const QPInput& other) const;

  inline bool operator!=(const QPInput& other) const {
    return !(this->operator==(other));
  }

  // Getters
  inline double w_basis_reg() const { return w_basis_reg_; }
  inline const std::string& dof_name(size_t idx) const {
    return desired_dof_motions_.dof_name(idx);
  }
  inline const std::unordered_map<std::string, ContactInformation>&
  contact_information() const {
    return contact_info_;
  }
  inline const std::unordered_map<std::string, DesiredBodyMotion>&
  desired_body_motions() const {
    return desired_body_motions_;
  }
  inline const DesiredDoFMotions& desired_dof_motions() const {
    return desired_dof_motions_;
  }
  inline const DesiredCentroidalMomentumDot& desired_centroidal_momentum_dot()
      const {
    return desired_centroidal_momentum_dot_;
  }

  // Setters
  inline double& mutable_w_basis_reg() { return w_basis_reg_; }
  inline std::unordered_map<std::string, ContactInformation>&
  mutable_contact_information() {
    return contact_info_;
  }
  inline std::unordered_map<std::string, DesiredBodyMotion>&
  mutable_desired_body_motions() {
    return desired_body_motions_;
  }
  inline DesiredDoFMotions& mutable_desired_dof_motions() {
    return desired_dof_motions_;
  }
  inline DesiredCentroidalMomentumDot&
  mutable_desired_centroidal_momentum_dot() {
    return desired_centroidal_momentum_dot_;
  }

 private:
  // Contact information
  std::unordered_map<std::string, ContactInformation> contact_info_;

  // Desired task space accelerations for specific bodies
  std::unordered_map<std::string, DesiredBodyMotion> desired_body_motions_;

  // Desired joint accelerations
  DesiredDoFMotions desired_dof_motions_;

  // Desired centroidal momentum change (change of overall linear and angular
  // momentum)
  DesiredCentroidalMomentumDot desired_centroidal_momentum_dot_;

  // Weight for regularizing basis vectors
  double w_basis_reg_;
};

std::ostream& operator<<(std::ostream& out, const QPInput& input);

/**
 * This class holds the contact force / wrench related information, and works
 * closely with ContactInformation.
 */
class ResolvedContact {
 public:
  /**
   * @param body Reference to a RigidBody, which must be valid through the
   * lifespan of this obejct.
   */
  explicit ResolvedContact(const RigidBody<double>& body) : body_(&body) {}

  bool is_valid() const;

  bool operator==(const ResolvedContact& other) const;

  // Getters
  inline const RigidBody<double>& body() const { return *body_; }
  inline const std::string& body_name() const { return body_->get_name(); }
  inline const VectorX<double>& basis() const { return basis_; }
  inline const Matrix3X<double>& point_forces() const { return point_forces_; }
  inline const Matrix3X<double>& contact_points() const {
    return contact_points_;
  }
  inline const Vector6<double>& equivalent_wrench() const {
    return equivalent_wrench_;
  }
  inline const Vector3<double>& reference_point() const {
    return reference_point_;
  }
  inline const Vector6<double>& body_acceleration() const {
    return body_acceleration_;
  }
  inline int num_contact_points() const { return contact_points_.cols(); }
  inline int num_basis_per_contact_point() const {
    return num_basis_per_contact_point_;
  }

  // Setters
  inline void set_body(const RigidBody<double>& body) { body_ = &body; }
  inline VectorX<double>& mutable_basis() { return basis_; }
  inline Matrix3X<double>& mutable_point_forces() { return point_forces_; }
  inline Matrix3X<double>& mutable_contact_points() { return contact_points_; }
  inline Vector6<double>& mutable_equivalent_wrench() {
    return equivalent_wrench_;
  }
  inline Vector3<double>& mutable_reference_point() { return reference_point_; }
  inline Vector6<double>& mutable_body_acceleration() {
    return body_acceleration_;
  }
  inline int& mutable_num_basis_per_contact_point() {
    return num_basis_per_contact_point_;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  const RigidBody<double>* body_;

  int num_basis_per_contact_point_;

  // Stacked scalars for all the basis vectors.
  VectorX<double> basis_;

  // Point contact forces in the world frame.
  Matrix3X<double> point_forces_;

  // Contact points in the world frame.
  Matrix3X<double> contact_points_;

  // The equivalent wrench of all the point forces, w.r.t a frame that has
  // the same orientation as the world frame, but located at reference_point.
  Vector6<double> equivalent_wrench_;

  // Body acceleration w.r.t a frame that has the same orientation as the
  // world frame, but located at body's origin.
  Vector6<double> body_acceleration_;

  // Reference point in the world frame for the equivalent wrench.
  Vector3<double> reference_point_;
};

std::ostream& operator<<(std::ostream& out, const ResolvedContact& contact);

/**
 * This class holds task space acceleration for a rigid body. The first three
 * are angular acceleration.
 */
class BodyAcceleration {
 public:
  /**
   * @param body Reference to a RigidBody, which must be valid through the
   * lifespan of this obejct.
   */
  explicit BodyAcceleration(const RigidBody<double>& body)
      : body_(&body), accelerations_(Vector6<double>::Zero()) {}

  inline bool is_valid() const { return accelerations_.allFinite(); }

  bool operator==(const BodyAcceleration& other) const;
  // Getters
  inline const RigidBody<double>& body() const { return *body_; }
  inline const std::string& body_name() const { return body_->get_name(); }
  inline const Vector6<double>& accelerations() const { return accelerations_; }

  // Setter
  inline Vector6<double>& mutable_accelerations() { return accelerations_; }
  inline void set_body(const RigidBody<double>& body) { body_ = &body; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  const RigidBody<double>* body_;
  Vector6<double> accelerations_;
};

std::ostream& operator<<(std::ostream& out, const BodyAcceleration& acc);

/**
 * Output of the QP inverse dynamics controller
 */
class QPOutput {
 public:
  explicit QPOutput(const RigidBodyTree<double>& r);

  bool is_valid(int num_vd) const;

  // Getters
  inline const std::vector<std::string>& dof_names() const {
    return dof_names_;
  }
  inline const std::string& dof_name(int idx) const {
    return dof_names_.at(idx);
  }
  inline const Vector3<double>& comdd() const { return comdd_; }
  inline const Vector6<double>& centroidal_momentum_dot() const {
    return centroidal_momentum_dot_;
  }
  inline const VectorX<double>& vd() const { return vd_; }
  inline const std::unordered_map<std::string, BodyAcceleration>&
  body_accelerations() const {
    return body_accelerations_;
  }
  inline const VectorX<double>& dof_torques() const { return dof_torques_; }
  inline const std::unordered_map<std::string, ResolvedContact>&
  resolved_contacts() const {
    return resolved_contacts_;
  }
  inline const std::vector<std::pair<std::string, double>>& costs() const {
    return costs_;
  }
  inline const std::pair<std::string, double>& costs(size_t i) const {
    return costs_.at(i);
  }

  // Setters
  inline Vector3<double>& mutable_comdd() { return comdd_; }
  inline Vector6<double>& mutable_centroidal_momentum_dot() {
    return centroidal_momentum_dot_;
  }
  inline VectorX<double>& mutable_vd() { return vd_; }
  inline std::unordered_map<std::string, BodyAcceleration>&
  mutable_body_accelerations() {
    return body_accelerations_;
  }
  inline std::unordered_map<std::string, ResolvedContact>&
  mutable_resolved_contacts() {
    return resolved_contacts_;
  }
  inline VectorX<double>& mutable_dof_torques() { return dof_torques_; }

  inline std::vector<std::pair<std::string, double>>& mutable_costs() {
    return costs_;
  }
  inline std::pair<std::string, double>& mutable_cost(size_t i) {
    return costs_.at(i);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // Names for each generalized coordinate.
  std::vector<std::string> dof_names_;

  // Computed accelerations for the center of mass in the world frame
  Vector3<double> comdd_;
  // Computed centroidal momentum dot in the world frame: [angular; linear]
  // The linear part equals comdd_ * mass.
  Vector6<double> centroidal_momentum_dot_;
  // Computed generalized coordinate accelerations
  VectorX<double> vd_;
  // Computed joint torque, in dof's order, not robot.actuator's order.
  VectorX<double> dof_torques_;

  // Computed contact related information such as point contact forces
  std::unordered_map<std::string, ResolvedContact> resolved_contacts_;

  // Tracked body motion
  std::unordered_map<std::string, BodyAcceleration> body_accelerations_;

  // Pair of the name of cost term and cost value (only the quadratic and linear
  // term, no constant term).
  std::vector<std::pair<std::string, double>> costs_;
};

std::ostream& operator<<(std::ostream& out, const QPOutput& output);

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/param_parsers/id_controller_config.pb.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/param_parsers/rigid_body_tree_alias_groups.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller_common.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {
namespace param_parsers {

/**
 * Struct for storing parameters for generating a desired acceleration
 * objective \f$ \dot{v}_d \f$ for the QP inverse dynamics controller. A way
 * to generate \f$ \dot{v}_d \f$ is through a simple PD law around a nominal
 * setpoint \f$ (q^*, v^*, \dot{v}^*) \f$ as:
 * \f[
 * \dot{v}_d = kp (q^* - q) + kd (v^* - v) + \dot{v}^*.
 * \f]
 * This setpoint is typically generated by some motion planner or simple
 * heuristics. \f$ \dot{v}_d \f$ is then used in the quadratic cost function
 * in the QP inverse dynamics controller. The cost function typically looks
 * like:
 * \f[
 * min_{\dot{v}} (\dot{v}_d - \dot{v})^T W (\dot{v}_d - \dot{v}),
 * \f]
 * where \f$ W \f$ is a diagonal weighting matrix. \f$ W \f$'s diagonal terms
 * are from `weight`. Depending on the application, \f$ \dot{v}_d \f$ can also
 * be ignored or treated as an equality constraint in the optimization.
 * If `weight(i)` is positive, \f$ \dot{v}_d(i) \f$ is used as a cost term.
 * If `weight(i)` is negative, \f$ \dot{v}_d(i) \f$ is used as an equality
 * constraint. If weight(i) is zero, \f$ \dot{v}_d(i) \f$ is ignored.
 *
 * This struct holds name, kp, kd, and weight.
 */
struct DesiredMotionParam {
  /// Name can be arbitrary.
  std::string name;
  VectorX<double> kp;
  VectorX<double> kd;
  VectorX<double> weight;

  DesiredMotionParam() {}

  explicit DesiredMotionParam(int dim) {
    kp = VectorX<double>::Zero(dim);
    kd = VectorX<double>::Zero(dim);
    weight = VectorX<double>::Zero(dim);
  }

  /// Only checks numerical values being equal. Does not check for
  /// name equality.
  bool operator==(const DesiredMotionParam& other) const {
    if (!kp.isApprox(other.kp)) return false;
    if (!kd.isApprox(other.kd)) return false;
    if (!weight.isApprox(other.weight)) return false;
    return true;
  }

  bool operator!=(const DesiredMotionParam& other) const {
    return !(*this == other);
  }
};

std::ostream& operator<<(std::ostream& out, const DesiredMotionParam& param);

/**
 * A struct for holding contact-related parameters such as local offsets of the
 * contact points, friction coefficient, etc. These can be used to make a
 * ContactInformation which is used by the QP inverse dynamics controller.
 * Please see ContactInformation for more details.
 */
struct ContactParam {
  /// Name can be arbitrary.
  std::string name;
  /// Specified in the body frame.
  Matrix3X<double> contact_points;
  /// Specified in the body frame.
  Vector3<double> normal;
  int num_basis_per_contact_point;
  /// Friction coefficient.
  double mu;
  /// The QP controller tries to damp contact motions by adding a desired
  /// acceleration objective / constraint as \f$ \dot{v}_d = -kd v \f$.
  /// `weight` is used to specify how \f$ \dot{v}_d \f$ is used in the
  /// optimization. If `weight` is positive, \f$ \dot{v}_d \f$ is used as a
  /// cost term. If `weight` is negative, \f$ \dot{v}_d \f$ is used as an
  /// equality constraint. `weight` shouldn't be zero.
  double kd;
  double weight;

  /// Only checks numerical values being equal. Does not check for
  /// name equality.
  bool operator==(const ContactParam& other) const {
    if (num_basis_per_contact_point != other.num_basis_per_contact_point)
      return false;
    if (mu != other.mu) return false;
    if (kd != other.kd) return false;
    if (weight != other.weight) return false;
    if (!normal.isApprox(other.normal)) return false;
    if (!contact_points.isApprox(other.contact_points)) return false;
    return true;
  }

  bool operator!=(const ContactParam& other) const { return !(*this == other); }
};

std::ostream& operator<<(std::ostream& out, const ContactParam& param);

/**
 * A class for parsing and storing parameters that are used to generate QpInput
 * for the inverse dynamics controller. Utility methods for generating
 * ContactInformation, DesiredBodyMotion, DesiredDoFMotion and
 * DesiredCentroidalMomentumDot from the stored parameters are also provided.
 * DesiredBodyMotion, DesiredDoFMotion and DesiredCentroidalMomentumDot consist
 * of two groups of numbers: gains / weights and desired acceleration. This
 * class only works with the gains and weights. The second group needs to be
 * computed by some feedback policy, which outside the scope of this class.
 *
 * This class loads its configuration from text-format protocol buffers.
 * Uninitialized fields will default to 0 for numerical values, and empty string
 * for string values. It is highly recommended that all fields are specified
 * when writing a config file.
 */
class ParamSet {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ParamSet)

  ParamSet() {}

  /**
   * Loads parameters from a config file for the inverse dynamics controller.
   * The format of the config file is defined in id_controller_config.proto.
   *
   * For the `ContactConfig` and `AccelerationConfig` (defined in
   * id_controller_config.proto), the `name` field should correspond to either
   * a body group or joint group name in the associated
   * RigidBodyTreeAliasGroups. Every member in that group will have the same
   * parameters. A `default` parameter can also be specified for `body_motion`,
   * `dof_motion` and `contact` for InverseDynamicsControllerConfig. It will be
   * returned when parameters are not explicitly specified. It is recommended
   * to supply the default parameters.
   *
   * For AccelerationConfig, the number of recurrence for `kp`, `kd` and
   * `weight` can either be 1 or exactly matches the dimension of the
   * associated parameter. The first case is a simpler way for specifying
   * everything with the same number. For example, when specifying parameters
   * for spatial accelerations, `kp`, `kd` and `weight` need to be 6
   * dimensional. When used for accelerations in the generalized coordinates,
   * `kp`, `kd` and `weight` need to match the dimension of that joint group.
   * E.g. `kp`, `kd` and `weight` need to be 8 dimensional for a joint group
   * that consists of a floating base joint and 2 single dof joints.
   *
   * @param config_path Path to the config file.
   * @param alias_group Specifies the relationship between body / joint groups
   * and the RigidBodyTree it is constructed from.
   *
   * @throws std::runtime_error if the config file can not be parsed correctly.
   */
  void LoadFromFile(const std::string& config_path,
                    const RigidBodyTreeAliasGroups<double>& alias_group);

  /**
   * Returns a map from body names to ContactInformation, where the body names
   * belong to the body group specified by @p group_name in @p alias_group.
   * For each body without a corresponding ContactParam, a ContactInformation
   * will be constructed using the default ContactParam. If @p alias_group
   * does not contain @p group_name, an empty map will be returned.
   *
   * @param group_name Name of the body group of interest.
   * @param alias_group Specifies the relationship between body / joint groups
   * and the RigidBodyTree it is constructed from.
   * @return A map from body names to ContactInformation
   */
  std::unordered_map<std::string, ContactInformation> MakeContactInformation(
      const std::string& group_name,
      const RigidBodyTreeAliasGroups<double>& alias_group) const;

  /**
   * Returns a map from body names to DesiredBodyMotions, where the body names
   * belongs to the body group specified by @p group_name in @p alias_group.
   * This method only sets the weights and constraint_types fields of
   * DesiredBodyMotion; it does not set the values, which must be set
   * separately. For each body name, if it has no corresponding
   * DesiredMotionParam, a DesiredBodyMotion will be constructed using the
   * default DesiredMotionParam. If @p alias_group does not contain
   * @p group_name, an empty map will be returned.
   *
   * @param group_name Name of the body group of interest.
   * @param alias_group Specifies the relationship between body / joint groups
   * and the RigidBodyTree it is constructed from.
   * @return A map from body names to DesiredBodyMotions.
   */
  std::unordered_map<std::string, DesiredBodyMotion> MakeDesiredBodyMotion(
      const std::string& group_name,
      const RigidBodyTreeAliasGroups<double>& alias_group) const;

  /**
   * Returns a single ContactInformation for @p body. If @p body has no
   * corresponding ContactParam, a ContactInformation constructed with the
   * default ContactParam will be returned.
   */
  ContactInformation MakeContactInformation(
      const RigidBody<double>& body) const;

  /**
   * Returns a single DesiredBodyMotion for @p body. This method only sets the
   * weights and constraint_types fields of DesiredBodyMotion; it does not set
   * the values, which must be set separately. If @p body has no corresponding
   * DesiredMotionParam, a DesiredBodyMotion constructed with the default
   * DesiredMotionParam will be returned.
   */
  DesiredBodyMotion MakeDesiredBodyMotion(const RigidBody<double>& body) const;

  /**
   * Finds the kp and kd gains for all the bodies in the body group specified
   * by @p group_name. For each body in body group @p group_name, if it has no
   * corresponding DesiredMotionParam, its kp and kd will be set to the values
   * in the default DesiredMotionParam. The ith entry in @p kp and @p kd
   * correspond to the ith entry in body group @p group_name.
   * @p kp and @p kd will be resized to match the size of the body group
   * specified by @p group_name. If @p group_name does not exist in
   * @p alias_group, @p kp and @p kd will be cleared.
   *
   * @param group_name Name of the body group of interest.
   * @param alias_group RigidBodyTreeAliasGroups specifying the relationship
   * between body / joint groups and the RigidBodyTree it is constructed from.
   * @param[out] kp container for kp's output.
   * @param[out] kd container for kd's output.
   */
  void LookupDesiredBodyMotionGains(
      const std::string& group_name,
      const RigidBodyTreeAliasGroups<double>& alias_group,
      std::vector<Vector6<double>>* kp, std::vector<Vector6<double>>* kd) const;

  /**
   * Finds the kp and kd gains for @p body. If it has no corresponding
   * DesiredMotionParam, the kp and kd will be set to the values in the default
   * DesiredMotionParam.
   *
   * @param body Referent to the RigidBody of interest.
   * @param[out] kp output.
   * @param[out] kd output.
   */
  void LookupDesiredBodyMotionGains(const RigidBody<double>& body,
                                    Vector6<double>* kp,
                                    Vector6<double>* kd) const;

  /**
   * Obtains the kp and kd gains for all DoF.
   * @p kp and @p kd will be resized to match the number of DoFs of the
   * RigidBodyTree in the RigidBodyTreeAliasGroups passed to LoadFromFile().
   *
   * @param[out] kp output.
   * @param[out] kd output.
   */
  void LookupDesiredDofMotionGains(VectorX<double>* kp,
                                   VectorX<double>* kd) const;

  /**
   * Obtains the kp and kd gains for the centroidal momentum tracker.
   *
   * @param[out] kp output.
   * @param[out] kd output.
   */
  void LookupDesiredCentroidalMomentumDotGains(Vector6<double>* kp,
                                               Vector6<double>* kd) const;

  /**
   * Returns DesiredDofMotions configured by DesiredMotionParam for all DoF.
   * This method only sets the weights and constraint_types fields in
   * DesiredDofMotions. The values field needs to be set separately.
   * For each DoF with no DesiredMotionParam, the default DesiredMotionParam
   * will be used.
   */
  DesiredDofMotions MakeDesiredDofMotions() const;

  /**
   * Returns DesiredCentroidalMomentumDot configured by the DesiredMotionParam
   * for centroidal momentum. This method only sets the weights and
   * constraint_types fields in DesiredCentroidalMomentumDot. The values field
   * needs to be set separately.
   */
  DesiredCentroidalMomentumDot MakeDesiredCentroidalMomentumDot() const;

  /**
   * Returns a QpInput for the given contacts and tracked bodies using the
   * parameters hold by this instance. Note that this function only sets the
   * `weights` and `constraint_types` fields for DesiredBodyMotion and
   * DesiredDofMotions, the desire accelerations need to be set separately by
   * some control policy.
   * @param contact_body_groups, Names of body groups that are in contact.
   * For each body of each group, a ContactInformation will be populated in the
   * returned QpInput.
   * @param tracked_body_groups, Names of body groups that are being tracked.
   * For each body of each group, a DesiredBodyMotion will be populated in the
   * returned QpInput.
   */
  QpInput MakeQpInput(
      const std::vector<std::string>& contact_body_groups,
      const std::vector<std::string>& tracked_body_groups,
      const RigidBodyTreeAliasGroups<double>& alias_group) const;

  QpInput MakeQpInput(
      const std::vector<const RigidBody<double>*>& contact_bodies,
      const std::vector<const RigidBody<double>*>& tracked_bodies,
      const RigidBodyTreeAliasGroups<double>& alias_group) const;

  /**
   * Returns the weight for regularizing the basis vectors of contact forces.
   */
  double get_basis_regularization_weight() const {
    return basis_regularization_weight_;
  }

  /**
   * Returns the name of this ParamSet.
   */
  const std::string& get_name() const { return name_; }

  /**
   * Sets the name of this ParamSet.
   * @param name New name for this ParamSet.
   */
  void set_name(const std::string& name) { name_ = name; }

 private:
  // Name of this ParamSet.
  std::string name_;

  double basis_regularization_weight_{0};

  // Maps from body_name (or "default") to ContactParam.
  std::unordered_map<std::string, ContactParam> contact_params_;
  // Maps from body_name (or "default") to DesiredMotionParam.
  std::unordered_map<std::string, DesiredMotionParam> body_motion_params_;

  // One DesiredMotionParam of size 1 per DoF.
  std::vector<DesiredMotionParam> dof_motion_params_;

  DesiredMotionParam centroidal_momentum_dot_params_;

  // Generates a ContactInformation for @p body based on @p param.
  ContactInformation MakeContactInformationFromParam(
      const RigidBody<double>& body, const ContactParam& param) const;

  // Generates a DesiredBodyMotion for @p body based on @p param.
  DesiredBodyMotion MakeDesiredBodyMotionFromParam(
      const RigidBody<double>& body, const DesiredMotionParam& param) const;
};

}  // namespace param_parsers
}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

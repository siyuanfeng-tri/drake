#include "drake/multibody/kinematics_results.h"

#include "drake/common/eigen_types.h"
#include "drake/multibody/kinematics_cache.h"
#include "drake/util/drakeGeometryUtil.h"

namespace drake {
namespace systems {

template<typename T>
KinematicsResults<T>::KinematicsResults(const RigidBodyTree<T>* tree) :
    tree_(tree), kinematics_cache_(tree_->CreateKinematicsCache()) {
}

template <typename T>
void KinematicsResults<T>::Update(const Eigen::Ref<const VectorX<T>>& q,
                                  const Eigen::Ref<const VectorX<T>>& v) {
  this->kinematics_cache_.initialize(q, v);
  //this->tree_->doKinematics(this->kinematics_cache_, false);
  this->tree_->doKinematics(this->kinematics_cache_, true);

  mass_matrix_ = tree_->massMatrix(kinematics_cache_);
  eigen_aligned_std_unordered_map<RigidBody<T> const*, TwistVector<T>> f_ext;
  dynamics_bias_ = tree_->dynamicsBiasTerm(kinematics_cache_, f_ext);

  centroidal_momentum_matrix_ = tree_->centroidalMomentumMatrix(kinematics_cache_);
  centroidal_momentum_matrix_dot_times_v_ =
     tree_->centroidalMomentumMatrixDotTimesV(kinematics_cache_);

//  centroidal_momentum_ = centroidal_momentum_matrix_ * v;
  center_of_mass_ = tree_->centerOfMass(kinematics_cache_);
  center_of_mass_dot_ = centroidal_momentum_matrix_ * v / tree_->getMass();
}

template <typename T>
int KinematicsResults<T>::get_num_bodies() const {
  return tree_->get_num_bodies();
}

template<typename T>
int KinematicsResults<T>::get_num_positions() const {
  return kinematics_cache_.get_num_positions();
}

template<typename T>
int KinematicsResults<T>::get_num_velocities() const {
  return kinematics_cache_.get_num_velocities();
}

template <typename T>
Isometry3<T> KinematicsResults<T>::get_pose_in_world_frame(
    int index, const Isometry3<T>& local_offset) const {
  return tree_->relativeTransform(kinematics_cache_,
      tree_->world().get_body_index(), index) * local_offset;
}

template <typename T>
TwistVector<T> KinematicsResults<T>::get_twist_in_world_frame(
    int index, const Isometry3<T>& local_offset) const {
  int world_index = tree_->world().get_body_index();
  // Since there is no relaitve motion between body and the fixed frame.
  // The twist of the offseted frame is just the twist of the body.
  return tree_->relativeTwist(
      kinematics_cache_, world_index, index, world_index);
}

template <typename T>
TwistVector<T> KinematicsResults<T>::get_twist_in_world_aligned_body_frame(
    int index, const Isometry3<T>& local_offset) const {
  TwistVector<T> twist_in_world = get_twist_in_world_frame(index, local_offset);
  Isometry3<T> body_to_world = get_pose_in_world_frame(index, local_offset);
  Isometry3<T> world_to_world_aligned_body(Isometry3<T>::Identity());
  world_to_world_aligned_body.translation() = -body_to_world.translation();
  return transformSpatialMotion(world_to_world_aligned_body, twist_in_world);
}

template <typename T>
MatrixX<T> KinematicsResults<T>::get_jacobian_for_world_aligned_body_frame(
    int index, const Isometry3<T>& local_offset) const {
  int world_index = tree_->world().get_body_index();

  Vector3<T> p = tree_->transformPoints(
      kinematics_cache_, local_offset.translation(), index, world_index);

  std::vector<int> v_or_q_indices;
  MatrixX<T> J_body = tree_->geometricJacobian(
      kinematics_cache_, world_index, index, world_index,
      true, &v_or_q_indices);

  int col = 0;
  MatrixX<T> J = MatrixX<T>::Zero(kTwistSize, tree_->get_num_velocities());
  for (int idx : v_or_q_indices) {
    // Angular velocity stays the same.
    J.col(idx) = J_body.col(col);
    // Linear velocity needs an additional cross product term.
    J.col(idx).template tail<kSpaceDimension>() +=
        J_body.col(col).template head<kSpaceDimension>().cross(p);
    col++;
  }

  return J;
}

template <typename T>
TwistVector<T> KinematicsResults<T>::get_jacobian_dot_time_v_for_world_aligned_body_frame(
    int index, const Isometry3<T>& local_offset) const {
  int world_index = tree_->world().get_body_index();

  Vector3<T> p = tree_->transformPoints(
      kinematics_cache_, local_offset.translation(), index, world_index);
  TwistVector<T> twist = tree_->relativeTwist(
      kinematics_cache_, world_index, index, world_index);
  TwistVector<T> J_body_dot_times_v = tree_->geometricJacobianDotTimesV(
      kinematics_cache_, world_index, index, world_index);

  // linear vel of p
  Vector3<T> pdot = twist.template head<3>().cross(p)
                  + twist.template tail<3>();

  // each column of Jt = [Jg_omega; Jg_v + Jg_omega.cross(p)]
  // for Jtdot * v, the angular part stays the same,
  // for the linear part:
  //  = [\dot{Jg_v} + \dot{Jg_omega}.cross(p) + Jg_omega.cross(pdot)] * v
  //  = [liner part of JgdotV + angular of JgdotV.cross(p) + omega.cross(pdot)]
  TwistVector<T> Jdv = J_body_dot_times_v;
  Jdv.template tail<3>() += twist.template head<3>().cross(pdot)
                         + J_body_dot_times_v.template head<3>().cross(p);

  return Jdv;
}

template <typename T>
Eigen::VectorBlock<const VectorX<T>> KinematicsResults<T>::get_joint_position(
    const RigidBody<T>& body) const {
  return kinematics_cache_.getQ().segment(body.get_position_start_index(),
                                          body.getJoint().get_num_positions());
}

template <typename T>
Eigen::VectorBlock<const VectorX<T>> KinematicsResults<T>::get_joint_velocity(
    const RigidBody<T>& body) const {
  return kinematics_cache_.getV().segment(body.get_velocity_start_index(),
                                          body.getJoint().get_num_velocities());
}

// Explicitly instantiates on the most common scalar types.
template class KinematicsResults<double>;

}  // namespace systems
}  // namespace drake


#pragma once

#include <Eigen/Geometry>

#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace systems {

// Forward declaration for KinematicsResults.
template <typename T>
class RigidBodyPlant;

/// A class containing the kinematics results from a RigidBodyPlant system.
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class KinematicsResults {
 public:
  /// Constructs a KinematicsResults object associated with @param tree.
  /// An alias to @param tree is maintained so that the tree's lifetime must
  /// exceed this object's lifetime.
  explicit KinematicsResults(const RigidBodyTree<T>* tree);

  /// Updates the KinematicsResults object given the configuration vector
  /// @param q and velocity vector @param v.
  void Update(const Eigen::Ref<const VectorX<T>>& q,
              const Eigen::Ref<const VectorX<T>>& v);

  /// Returns the number of bodies in the kinematics results.
  int get_num_bodies() const;

  /// Returns the number of generalized positions.
  int get_num_positions() const;

  /// Returns the number of generalized velocities.
  int get_num_velocities() const;

  /// Returns the quaternion representation of the three dimensional orientation
  /// of body @p body_index in the world's frame.
  Quaternion<T> get_body_orientation(int body_index) const {
    return Quaternion<T>(get_pose_in_world_frame(body_index).linear());
  }

  /// Returns the three dimensional position of body @p body_index in world's
  /// frame.
  Vector3<T> get_body_position(int body_index) const {
    return get_pose_in_world_frame(body_index).translation();
  }

  /// Returns the pose of body @p body with respect to the world.
  Isometry3<T> get_pose_in_world_frame(
      const RigidBody<T>& body, const Isometry3<T>& local_offset = Isometry3<T>::Identity()) const {
    return get_pose_in_world_frame(body.get_body_index(), local_offset);
  }

  /// Returns the twist of @p body with respect to the world, expressed in world
  /// frame.
  TwistVector<T> get_twist_in_world_frame(
      const RigidBody<T>& body, const Isometry3<T>& local_offset = Isometry3<T>::Identity()) const {
    return get_twist_in_world_frame(body.get_body_index(), local_offset);
  }

  /// Returns the twist of @p body with respect to the world, expressed in world
  /// frame.
  TwistVector<T> get_twist_in_world_aligned_body_frame(
      const RigidBody<T>& body, const Isometry3<T>& local_offset = Isometry3<T>::Identity()) const {
    return get_twist_in_world_aligned_body_frame(body.get_body_index(), local_offset);
  }

  /// Returns the pose of body @p body with respect to the world.
  Isometry3<T> get_pose_in_world_frame(
      const RigidBodyFrame<T>& frame, const Isometry3<T>& local_offset = Isometry3<T>::Identity()) const {
    return get_pose_in_world_frame(frame.get_frame_index(), local_offset);
  }

  /// Returns the twist of @p body with respect to the world, expressed in world
  /// frame.
  TwistVector<T> get_twist_in_world_frame(
      const RigidBodyFrame<T>& frame, const Isometry3<T>& local_offset = Isometry3<T>::Identity()) const {
    return get_twist_in_world_frame(frame.get_frame_index(), local_offset);
  }

  /// Returns the twist of @p body with respect to the world, expressed in world
  /// frame.
  TwistVector<T> get_twist_in_world_aligned_body_frame(
      const RigidBodyFrame<T>& frame, const Isometry3<T>& local_offset = Isometry3<T>::Identity()) const {
    return get_twist_in_world_aligned_body_frame(frame.get_frame_index(), local_offset);
  }

  /// Returns the joint position vector associated with the joint between
  /// @p body and @p body's parent.
  /// TODO(tkoolen) should pass in joint instead of body, but that's currently
  /// not convenient.
  Eigen::VectorBlock<const VectorX<T>> get_joint_position(
      const RigidBody<T>& body) const;

  /// Returns the joint velocity vector associated with the joint between
  /// @p body and @p body's parent.
  /// TODO(tkoolen) should pass in joint instead of body, but that's currently
  /// not convenient.
  Eigen::VectorBlock<const VectorX<T>> get_joint_velocity(
      const RigidBody<T>& body) const;

  const RigidBodyTree<T>& get_tree() const { return *tree_; }

  const KinematicsCache<T>& get_kinematics_cache() const { return kinematics_cache_; }

  const MatrixX<T>& get_mass_matrix() const { return mass_matrix_; }

  const VectorX<T>& get_dynamics_bias() const { return dynamics_bias_; }

  const MatrixX<T>& get_centroidal_momentum_matrix() const { return centroidal_momentum_matrix_; }

  const VectorX<T>& get_centroidal_momentum_matrix_dot_times_v() const { return centroidal_momentum_matrix_dot_times_v_; }

  const Vector3<T>& get_center_of_mass() const { return center_of_mass_; }

  const Vector3<T>& get_center_of_mass_dot() const { return center_of_mass_dot_; }

  const VectorX<T>& get_positions() const { return kinematics_cache_.getQ(); }

  const VectorX<T>& get_velocities() const { return kinematics_cache_.getV(); }

  /**
   * This function computes the Jacobian of the body frame with respect to the
   * world frame expressed in a frame that is aligned with the world frame but
   * located at the origin of the body frame.
   * get_twist_in_world_aligned_body_frame(body) =
   * get_jacobian_for_world_aligned_body_frame(body) * v
   * @param body, Reference to the body
   * @param local_offset from the body frame origin to the point of interest
   * in body frame
   * @return Jacobian
   */
  MatrixX<T> get_jacobian_for_world_aligned_body_frame(
      const RigidBody<T>& body,
      const Isometry3<T>& local_offset = Isometry3<T>::Identity()) const {
    return get_jacobian_for_world_aligned_body_frame(
        body.get_body_index(), local_offset);
  }

  MatrixX<T> get_jacobian_for_world_aligned_body_frame(
      const RigidBodyFrame<T>& frame,
      const Isometry3<T>& local_offset = Isometry3<T>::Identity()) const {
    return get_jacobian_for_world_aligned_body_frame(
        frame.get_frame_index(), local_offset);
  }

  /**
   * This function computes the Jacobian dot times the generalized velocities
   * term of the body frame with respect to the world frame expressed in a
   * frame that is aligned with the world frame but located at the origin of
   * the body frame.
   * get_twist_in_world_aligned_body_frame(body)_dot =
   * get_jacobian_for_world_aligned_body_frame(body) * vd +
   * get_jacobian_dot_time_v_for_world_aligned_body_frame(body)
   * @param body reference to the body
   * @param local_offset from the body frame origin to the point of interest
   * in body frame
   * @return Jacobian_dot * v
   */
  TwistVector<T> get_jacobian_dot_time_v_for_world_aligned_body_frame(
      const RigidBody<T>& body,
      const Isometry3<T>& local_offset = Isometry3<T>::Identity()) const {
    return get_jacobian_dot_time_v_for_world_aligned_body_frame(
        body.get_body_index(), local_offset);
  }

  TwistVector<T> get_jacobian_dot_time_v_for_world_aligned_body_frame(
      const RigidBodyFrame<T>& frame,
      const Isometry3<T>& local_offset = Isometry3<T>::Identity()) const {
    return get_jacobian_dot_time_v_for_world_aligned_body_frame(
        frame.get_frame_index(), local_offset);
  }

 private:
  // RigidBodyPlant is the only class allowed to update KinematicsResults
  // through UpdateFromContext().
  // TODO(amcastro-tri): when KinematicsResults can reference entries in the
  // cache this friendship and the method UpdateFromContext() won't be needed.
  // friend class RigidBodyPlant<T>;

  // Updates KinematicsResults from a context provided by RigidBodyPlant.
  // Only RigidBodyPlant has access to this method since it is a friend.
  // TODO(amcastro-tri): when KinematicsResults can reference entries in the
  // cache this method won't be needed.
  // void UpdateFromContext(const Context<T>& context);

  Isometry3<T> get_pose_in_world_frame(
      int index, const Isometry3<T>& local_offset = Isometry3<T>::Identity()) const;

  TwistVector<T> get_twist_in_world_frame(
      int index, const Isometry3<T>& local_offset = Isometry3<T>::Identity()) const;

  TwistVector<T> get_twist_in_world_aligned_body_frame(
      int index, const Isometry3<T>& local_offset = Isometry3<T>::Identity()) const;

  MatrixX<T> get_jacobian_for_world_aligned_body_frame(
      int index, const Isometry3<T>& local_offset = Isometry3<T>::Identity()) const;

  TwistVector<T> get_jacobian_dot_time_v_for_world_aligned_body_frame(
      int index, const Isometry3<T>& local_offset = Isometry3<T>::Identity()) const;

  const RigidBodyTree<T>* tree_;
  KinematicsCache<T> kinematics_cache_;

  MatrixX<T> mass_matrix_;
  VectorX<T> dynamics_bias_;

  MatrixX<T> centroidal_momentum_matrix_;
  VectorX<T> centroidal_momentum_matrix_dot_times_v_;

  Vector3<T> center_of_mass_;
  Vector3<T> center_of_mass_dot_;
};

}  // namespace systems
}  // namespace drake

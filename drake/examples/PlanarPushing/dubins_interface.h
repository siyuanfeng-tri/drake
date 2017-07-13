#pragma once

#include "drake/examples/PlanarPushing/dubins.h"

#include <iostream>
#include <vector>

#include "drake/common/eigen_types.h"
// This class is an implementation of the paper:
// Pushing revisited: Differential flatness, trajectory planning and
// stabilization. Jiaji Zhou and Matthew T. Mason. Submitted to ISRR 2017.
// The PlanPath method will return a time-optimal pushing trajectory from
// an initial pose to an end pose.
// The general idea is to plan the path of a body fixed specific point
// (akin to center of rear axle) using Dubins curve and then recover the SE(2)
// trajectory of the rigid body.
// See
// github.com/robinzhoucmu/Pushing/blob/master/differential_flat/PushActionDubins.m
// for a similar implementation in Matlab..

class DubinsPushPlanner {
 public:
  // Construct the class and set parameters for reduction.
  DubinsPushPlanner(Eigen::Vector2d contact_point,
                    Eigen::Vector2d contact_normal, double mu, double ls_a,
                    double ls_b);
  ~DubinsPushPlanner();

  // Planer: given initial object pose to final object pose, return the
  // object cartesian pose and the pusher cartesian pose trajectories.
  void PlanPath(const Eigen::Vector3d cart_pose_start,
                const Eigen::Vector3d cart_pose_goal, int num_way_points,
                Eigen::Matrix<double, Eigen::Dynamic, 3>* object_poses,
                Eigen::Matrix<double, Eigen::Dynamic, 3>* pusher_poses);

 private:
  // Compute the dubins frame (at the differential flat output point akin to
  // center of rear axle) and minimum turning radius.
  void DubinsReduction();

  // Compute the center of rear axle: the mid point between the two CORs
  // corresponding to left and right edge of the friction cone.
  void ComputeCenterOfRearAxleAndTurningRadius();

  void ComputeDubinsFrame();

  void ComputePusherFrame();

  void ComputeFrictionCone();

  // Convert flat space state and velocity to cartesian pose wrt Dubins frame.
  void FlatSpaceToCartesianSpace(
      const Eigen::Matrix<double, Eigen::Dynamic, 2> flat_z,
      const Eigen::Matrix<double, Eigen::Dynamic, 2> flat_vz,
      Eigen::Matrix<double, Eigen::Dynamic, 3>* cartesian_poses);

  // This is for converting a cartesian pose in Dubins frame to augmented flat
  // state. Assuming instantaneous velocity will follow the positive y axis.
  // The third dimension in flat_augmented state denotes the heading of the head
  // point.
  void CartesianSpaceToFlatSpace(const Eigen::Vector3d cartesian_pose,
                                 Eigen::Vector3d* flat_augmented_z);

  // Given a starting pose, end pose in flat space and delta path length,
  // return the planned path in flat space.
  void GetDubinsPathFlatOutput(
      const Eigen::Vector3d flat_agumented_z_start,
      const Eigen::Vector3d flat_augmented_z_goal, const int num_way_points,
      Eigen::Matrix<double, Eigen::Dynamic, 2>* flat_traj_z);

  // Given a path (length N) in flat space, map it to cartesian space
  // (length N-1) in Dubins frame.
  void GetCartesianPathGivenFlatOutputPath(
      const Eigen::Matrix<double, Eigen::Dynamic, 2> flat_traj_z,
      Eigen::Matrix<double, Eigen::Dynamic, 3>* cartesian_traj);

  // Helper functions.
  Eigen::Vector3d GetDubinsPoseVectorGivenObjectPoseVector(
      const Eigen::Vector3d pose_vector_object);

  Eigen::Vector3d GetObjectPoseVectorGivenDubinsPoseVector(
      const Eigen::Vector3d pose_vector_dubins);

  Eigen::Vector3d ConvertSE3ToPoseVector(const Eigen::Isometry2d tf);

  Eigen::Isometry2d ConvertPoseVectorToSE3(const Eigen::Vector3d pose_vector);

  // First 2 elements (related to Fx, Fy) of the diagonal ellipsoid limit
  // surface.
  double ls_a_;
  // Third element (related to torque) the diagonal ellipsoid limit surface.
  double ls_b_;
  // Coefficient of friction.
  double mu_;
  // Minimum turning radius such that the contact remains sticking.
  double r_turn_;
  // Unit length ls_a_ / ls_b_;
  double unit_length_;

  // Pusher contact point in body frame.
  Eigen::Vector2d contact_point_;
  // Pusher contact normal in body frame.
  Eigen::Vector2d contact_normal_;
  // Left edge of the friction cone in body frame.
  Eigen::Vector2d friction_cone_left_;
  // Right edge of the friction cone in body frame.
  Eigen::Vector2d friction_cone_right_;

  // Homogeneous transform (SE(2)) of the dubins frame w.r.t the object frame.
  Eigen::Isometry2d tf_dubins_frame_;

  // Homogeneous transform of the pusher frame w.r.t. the object frame.
  Eigen::Isometry2d tf_pusher_frame_;

  // Center of rear axle w.r.t the dubins frame.
  Eigen::Vector2d center_rear_axle_;
};

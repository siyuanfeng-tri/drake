#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/manipulation/util/trajectory_utils.h"

#include "drake/lcmt_cartesian_trajectory.hpp"

namespace drake {
namespace manipulation {

class CartesianKeyframeTranslator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CartesianKeyframeTranslator)

  static void InitializeMessage(lcmt_cartesian_keyframe* msg);

  static void EncodeMessage(
      const std::string& name,
      double time,
      const Isometry3<double>& pose,
      const Eigen::Ref<const Vector6<double>>& velocity,
      const Eigen::Ref<const Vector6<double>>& acceleration,
      lcmt_cartesian_keyframe* msg);

};

class CartesianTrajectoryTranslator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CartesianTrajectoryTranslator)

  static void InitializeMessage(lcmt_cartesian_trajectory* msg);

  static void EncodeMessage(
      const std::string& name,
      const std::vector<double>& times,
      const std::vector<Isometry3<double>>& poses,
      const std::vector<Vector6<double>>& velocities,
      const std::vector<Vector6<double>>& accelerations,
      lcmt_cartesian_trajectory* msg);

  static void EncodeMessage(
      const std::string& name,
      const std::vector<double>& times,
      const PiecewiseCartesianTrajectory<double>& traj,
      lcmt_cartesian_trajectory* msg);

  // TODO: deal with expressed in frame and constraint mode.
  static void DecodeMessage(
      const lcmt_cartesian_trajectory& msg,
      std::string* name,
      PiecewiseCartesianTrajectory<double>* traj);
};

}  // namespace manipulation
}  // namespace drake

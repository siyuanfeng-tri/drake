#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/manipulation/util/trajectory_utils.h"

#include "drake/lcmt_dof_trajectory.hpp"

namespace drake {
namespace manipulation {

class DofKeyframeTranslator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DofKeyframeTranslator)

  static void InitializeMessage(int size, lcmt_dof_keyframe* msg);

  // TODO: constraint_mode
  static void EncodeMessage(double time,
      const Eigen::Ref<const VectorX<double>>& position,
      const Eigen::Ref<const VectorX<double>>& velocity,
      const Eigen::Ref<const VectorX<double>>& acceleration,
      lcmt_dof_keyframe* msg);

  // TODO: constraint_mode
  static void DecodeMessage(const lcmt_dof_keyframe& msg,
      double* time,
      Eigen::Ref<VectorX<double>> position,
      Eigen::Ref<VectorX<double>> velocity,
      Eigen::Ref<VectorX<double>> acceleration);
};

class DofTrajectoryTranslator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DofTrajectoryTranslator)

  static void InitializeMessage(lcmt_dof_trajectory* msg);

  static void EncodeMessage(
      const std::vector<double>& times,
      const std::vector<VectorX<double>>& positions,
      const std::vector<VectorX<double>>& velocities,
      const std::vector<VectorX<double>>& accelerations,
      lcmt_dof_trajectory* msg);

  static void DecodeMessage(
      const lcmt_dof_trajectory& msg,
      PiecewiseCubicTrajectory<double>* traj);

 private:
  const DofKeyframeTranslator translator_;
};

}  // namespace manipulation
}  // namespace drake

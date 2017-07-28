#pragma once

#include "drake/common/eigen_types.h"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace examples {
namespace jjz {

extern const Matrix3<double> R_ET;
extern const Isometry3<double> X_ET;
extern const Isometry3<double> X_WG;
extern const std::string kEEName;

class IiwaState {
 public:
  static constexpr double kUninitTime = -1.0;

  IiwaState(const RigidBodyTree<double>& iiwa);
  bool UpdateState(const lcmt_iiwa_status& msg);
  const KinematicsCache<double>& get_cache() const { return cache_; }
  const VectorX<double>& get_q() const { return q_; }
  const VectorX<double>& get_v() const { return v_; }
  const VectorX<double>& get_ext_trq() const { return ext_trq_; }
  const VectorX<double>& get_trq() const { return trq_; }
  const Vector6<double>& get_ext_wrench() const { return ext_wrench_; }
  double get_time() const { return time_; }
  double get_dt() const { return delta_time_; }

 private:
  const RigidBodyTree<double>& iiwa_;
  const RigidBody<double>& end_effector_;
  KinematicsCache<double> cache_;
  double time_{kUninitTime};
  double delta_time_{0};

  VectorX<double> q_;
  VectorX<double> v_;
  VectorX<double> trq_;
  VectorX<double> ext_trq_;
  MatrixX<double> J_;

  // J^T * F = ext_trq_
  Vector6<double> ext_wrench_;

  bool init_{false};
};

}  // namespace jjz
}  // namespace examples
}  // namespace drake

#pragma once

#include "drake/common/eigen_types.h"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_jjz_controller.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/manipulation/util/trajectory_utils.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace jjz {

extern const Matrix3<double> R_ET;
extern const Isometry3<double> X_ET;
extern const Isometry3<double> X_WG;
extern const std::string kEEName;

inline double get_system_time() {
  struct timespec the_tp;
  clock_gettime(CLOCK_REALTIME, &the_tp);
  return ((double)(the_tp.tv_sec)) + 1.0e-9 * the_tp.tv_nsec;
}

class IiwaState {
 public:
  static constexpr double kUninitTime = -1.0;

  IiwaState(const RigidBodyTree<double>* iiwa,
            const RigidBodyFrame<double>* frame_T);
  bool UpdateState(const lcmt_iiwa_status& msg);
  const KinematicsCache<double>& get_cache() const { return cache_; }
  const VectorX<double>& get_q() const { return q_; }
  const VectorX<double>& get_v() const { return v_; }
  const VectorX<double>& get_ext_trq() const { return ext_trq_; }
  const VectorX<double>& get_trq() const { return trq_; }
  const Vector6<double>& get_ext_wrench() const { return ext_wrench_; }
  const Isometry3<double>& get_X_WT() const { return X_WT_; }
  const Vector6<double>& get_V_WT() const { return V_WT_; }
  double get_time() const { return time_; }
  double get_dt() const { return delta_time_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  const RigidBodyTree<double>* iiwa_;
  const RigidBodyFrame<double>* frame_T_;
  KinematicsCache<double> cache_;
  double time_{kUninitTime};
  double delta_time_{0};

  VectorX<double> q_{};
  VectorX<double> v_{};
  VectorX<double> trq_{};
  VectorX<double> ext_trq_{};
  MatrixX<double> J_{};

  Isometry3<double> X_WT_{Isometry3<double>::Identity()};
  Vector6<double> V_WT_{Vector6<double>::Zero()};

  // J^T * F = ext_trq_
  Vector6<double> ext_wrench_{Vector6<double>::Zero()};

  bool init_{false};
};

class WsgState {
 public:
  static constexpr double kUninitTime = -1.0;

  void UpdateState(const lcmt_schunk_wsg_status& msg) {
    time_ = static_cast<double>(msg.utime) / 1e6;
    position_ = msg.actual_position_mm / 1000.;
    velocity_ = msg.actual_speed_mm_per_s / 1000.;
    force_ = msg.actual_force;
  }

  double get_time() const { return time_; }
  double get_position() const { return position_; }
  double get_velocity() const { return velocity_; }
  double get_force() const { return force_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  double time_{kUninitTime};
  double position_{0};
  double velocity_{0};
  double force_{0};
};

void FillDebugMessage(const IiwaState& state, lcmt_jjz_controller* msg);

template <typename T>
const T& clamp(const T& val, const T& lo, const T& hi) {
  DRAKE_DEMAND(lo < hi);

  if (val <= lo)
    return lo;
  else if (val >= hi)
    return hi;
  else
    return val;
}

Eigen::Matrix<double, 7, 1> pose_to_vec(const Isometry3<double>& pose);

// Solves the IK, s.t. FK(ret, frame_T) = X_WT.
VectorX<double> PointIk(const Isometry3<double>& X_WT,
                        const RigidBodyFrame<double>& frame_T,
                        const VectorX<double>& q_ini,
                        RigidBodyTree<double>* robot);

// When at the solution, @p frame_C's origin in world would be at
// @p camera_in_world, and @p frame_C's z axis would be pointing at
// @p target_in_world.
VectorX<double> GazeIk(const Vector3<double>& target_in_world,
                       const Vector3<double>& camera_in_world,
                       const RigidBodyFrame<double>& frame_C,
                       const VectorX<double>& q_ini,
                       RigidBodyTree<double>* robot);

// When at the solution, @p frame_C's z axis would be pointing at
// @p target_in_world, and of opposite direction of
// @p target_to_camera_in_world, and the distance between @p frame_C's origin
// and @p target_in_world is larger than min_dist.
VectorX<double> GazeIk2(const Vector3<double>& target_in_world,
                        const Vector3<double>& target_to_camera_in_world,
                        double min_dist, const RigidBodyFrame<double>& frame_C,
                        const VectorX<double>& q_ini,
                        RigidBodyTree<double>* robot);

std::vector<VectorX<double>> ComputeCalibrationConfigurations(
    const RigidBodyTree<double>& robot, const RigidBodyFrame<double>& frame_C,
    const VectorX<double>& q0, const Vector3<double>& p_WG, double min_dist,
    double width, double height, int num_width_pt, int num_height_pt);

// Returns a trajectory of T in W frame.
manipulation::PiecewiseCartesianTrajectory<double>
PlanPlanarPushingTrajMultiAction(const Vector3<double>& x_GQ,
                                 const Isometry3<double>& X_WG, double duration,
                                 std::string load_file_name = "");

PiecewisePolynomial<double> RetimeTrajCubic(
    const std::vector<MatrixX<double>>& q, const MatrixX<double>& v0,
    const MatrixX<double>& v1, const MatrixX<double>& v_lower,
    const MatrixX<double>& v_upper, const MatrixX<double>& vd_lower,
    const MatrixX<double>& vd_upper);

}  // namespace jjz
}  // namespace drake

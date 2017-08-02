#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_tree.h"

#include "drake/manipulation/planner/jacobian_ik.h"
#include "drake/manipulation/util/trajectory_utils.h"
#include "drake/examples/kuka_iiwa_arm/jjz_common.h"

namespace drake {
namespace examples {
namespace jjz {

class FSMState {
 public:
  FSMState(const std::string& name) : name_(name) {}
  virtual ~FSMState() {
    std::cout << "[" << get_name() << "] exiting.\n";
  }

  const std::string& get_name() const { return name_; }
  double get_start_time() const { return start_time_; }
  double get_in_state_time(const IiwaState& state) const {
    return state.get_time() - start_time_;
  }
  bool is_init() const { return init_; }

  void Initialize(const IiwaState& state) {
    if (is_init())
      return;

    start_time_ = state.get_time();
    init_ = true;

    DoInitialize(state);

    std::cout << "[" << get_name() << "] initialized at " << state.get_time() << "\n";
  }

  virtual void Update(const IiwaState& state) {}
  virtual void Control(const IiwaState& state, Eigen::Ref<VectorX<double>> q_d) const {}
  virtual bool IsDone(const IiwaState& state) const {
    return false;
  }

 protected:
  virtual void DoInitialize(const IiwaState& state) {}

 private:
  std::string name_;
  double start_time_{0};
  bool init_{false};
};

class MoveJoint : public FSMState {
 public:
  MoveJoint(const std::string& name, const VectorX<double>& q_d, double duration);

  void DoInitialize(const IiwaState& state) override;
  void Control(const IiwaState& state, Eigen::Ref<VectorX<double>> q_d) const override;
  bool IsDone(const IiwaState& state) const override;

 private:
  double duration_;
  const VectorX<double> q_end_;
  PiecewisePolynomial<double> traj_;
};

class MoveTool : public FSMState {
 public:
  MoveTool(const std::string& name, const RigidBodyTree<double>* robot,
      const VectorX<double>& q0, const VectorX<double>& q_norm);

  void DoInitialize(const IiwaState& state) override;
  void Update(const IiwaState& state) override;
  void Control(const IiwaState& state, Eigen::Ref<VectorX<double>> q_d) const override;

  virtual Isometry3<double> ComputeDesiredToolInWorld(const IiwaState& state) = 0;

 private:
  const RigidBodyTree<double>& robot_;
  const RigidBodyFrame<double> frame_T_;
  KinematicsCache<double> cache_;
  manipulation::planner::JacobianIk jaco_planner_;

  Vector6<double> gain_T_;
  VectorX<double> q_norm_;
  double last_time_;
};

class MoveToolFollowTraj : public MoveTool {
 public:
  MoveToolFollowTraj(const std::string& name, const RigidBodyTree<double>* robot,
      const VectorX<double>& q0, const VectorX<double>& q_norm,
      const manipulation::PiecewiseCartesianTrajectory<double>& traj);

  bool IsDone(const IiwaState& state) const override;
  Isometry3<double> ComputeDesiredToolInWorld(const IiwaState& state) override;

  void DoInitialize(const IiwaState& state) override;

  bool is_integrating() const { return is_integrating_; }
  const Isometry3<double>& get_pose_int() const { return pose_err_I_; }

  void reset_pose_integrator() { pose_err_I_.setIdentity(); }

  void set_integrating(bool flag) { is_integrating_ = flag; }
  void set_f_W_d(const Vector3<double>& f) { f_W_d_ = f; }
  void set_f_W_dead_zone(const Vector3<double>& zone) { f_W_dead_zone_ = zone; }
  void set_ki_force(const Vector3<double>& gain) { ki_.tail<3>() = gain; }
  void set_position_int_max_range(const Vector3<double>& range) { pos_I_range_ = range; }

  const Vector3<double>& get_f_W_d() const { return f_W_d_; }
  const Vector3<double>& get_f_W_dead_zone() const { return f_W_dead_zone_; }
  const Vector3<double>& get_position_int_range() const { return pos_I_range_; }

 private:
  manipulation::PiecewiseCartesianTrajectory<double> ee_traj_;

  // pose integrator
  Isometry3<double> pose_err_I_{Isometry3<double>::Identity()};
  bool is_integrating_{false};

  // Params.
  Vector3<double> f_W_d_{Vector3<double>::Zero()};
  Vector3<double> f_W_dead_zone_{Vector3<double>::Constant(INFINITY)};
  Vector3<double> pos_I_range_{Vector3<double>::Zero()};
  Vector6<double> ki_{Vector6<double>::Zero()};
};

}  // namespace jjz
}  // namespace examples
}  // namespace drake

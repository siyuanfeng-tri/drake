#pragma once

#include "drake/common/eigen_types.h"
#include "drake/examples/kuka_iiwa_arm/jjz_common.h"
#include "drake/multibody/rigid_body_tree.h"

#include "drake/lcmt_jjz_controller.hpp"
#include "drake/manipulation/planner/jacobian_ik.h"
#include "drake/manipulation/util/trajectory_utils.h"

namespace drake {
namespace examples {
namespace jjz {

class FSMState {
 public:
  FSMState(const std::string& name) : name_(name) {}
  virtual ~FSMState() { std::cout << "[" << get_name() << "] exiting.\n"; }

  const std::string& get_name() const { return name_; }
  double get_start_time() const { return start_time_; }
  double get_in_state_time(const IiwaState& state) const {
    return state.get_time() - start_time_;
  }
  bool is_init() const { return init_; }

  void Initialize(const IiwaState& state) {
    start_time_ = state.get_time();
    init_ = true;

    DoInitialize(state);

    std::cout << "[" << get_name() << "] initialized at " << state.get_time()
              << "\n";
  }

  void set_name(const std::string& name) { name_ = name; }

  virtual void Update(const IiwaState& state, lcmt_jjz_controller* msg) {}
  virtual void Control(const IiwaState& state, Eigen::Ref<VectorX<double>> q_d,
      Eigen::Ref<VectorX<double>> trq_d, lcmt_jjz_controller* msg) const {}
  virtual bool IsDone(const IiwaState& state) const { return false; }

 protected:
  virtual void DoInitialize(const IiwaState& state) {}

 private:
  std::string name_;
  double start_time_{0};
  bool init_{false};
};

class MoveJoint : public FSMState {
 public:
  MoveJoint(const std::string& name, const VectorX<double>& q0,
            const VectorX<double>& q1, double duration);

  void Control(const IiwaState& state, Eigen::Ref<VectorX<double>> q_d,
      Eigen::Ref<VectorX<double>> trq_d, lcmt_jjz_controller* msg) const override;
  bool IsDone(const IiwaState& state) const override;

 private:
  PiecewisePolynomial<double> traj_;
};

class MoveTool : public FSMState {
 public:
  MoveTool(const std::string& name, const RigidBodyTree<double>* robot,
           const RigidBodyFrame<double>* frame_T, const VectorX<double>& q0);

  void DoInitialize(const IiwaState& state) override;
  void Update(const IiwaState& state, lcmt_jjz_controller* msg) override;
  void Control(const IiwaState& state, Eigen::Ref<VectorX<double>> q_d,
      Eigen::Ref<VectorX<double>> trq_d, lcmt_jjz_controller* msg) const override;

  virtual Isometry3<double> ComputeDesiredToolInWorld(
      const IiwaState& state) const = 0;

  Isometry3<double> get_X_WT_ik() const {
    return robot_.CalcFramePoseInWorldFrame(cache_, frame_T_);
  }

  void set_nominal_q(const VectorX<double>& q_norm) { q_norm_ = q_norm; }
  void set_tool_gain(const Vector6<double>& gain) { gain_T_ = gain; }

  const RigidBodyTree<double>& get_robot() const { return robot_; }
  const RigidBodyFrame<double>& get_tool_frame() const { return frame_T_; }

 private:
  const RigidBodyTree<double>& robot_;
  const RigidBodyFrame<double> frame_T_;
  KinematicsCache<double> cache_;
  manipulation::planner::JacobianIk jaco_planner_;

  Vector6<double> gain_T_{Vector6<double>::Constant(1)};
  VectorX<double> q_norm_;
  double last_time_;
};

class MoveToolStraightUntilTouch : public MoveTool {
 public:
  MoveToolStraightUntilTouch(const std::string& name,
      const RigidBodyTree<double>* robot, const RigidBodyFrame<double>* frame_T,
      const VectorX<double>& q0, const Vector3<double>& dir, double vel);

  Isometry3<double> ComputeDesiredToolInWorld(
      const IiwaState& state) const override;

  bool IsDone(const IiwaState& state) const override;
  double get_f_ext_thresh() const { return f_ext_thresh_; }
  void set_f_ext_thresh(double thresh) { f_ext_thresh_ = thresh; }

 private:
  Isometry3<double> X_WT0_;
  Vector3<double> dir_;
  double vel_;

  double f_ext_thresh_{10};
};

class HoldPositionAndApplyForce : public FSMState {
 public:
  HoldPositionAndApplyForce(const std::string& name,
      const RigidBodyTree<double>* robot, const RigidBodyFrame<double>* frame_T);

  void Update(const IiwaState& state, lcmt_jjz_controller* msg) override;
  void Control(const IiwaState& state, Eigen::Ref<VectorX<double>> q_d,
      Eigen::Ref<VectorX<double>> trq_d, lcmt_jjz_controller* msg) const override;

  const Vector6<double>& get_desired_ext_wrench() const { return ext_wrench_d_; }
  void set_desired_ext_wrench(const Vector6<double>& w) { ext_wrench_d_ = w; }

 private:
  void DoInitialize(const IiwaState& state) override {
    q0_ = state.get_q();
  }

  VectorX<double> q0_;

  Vector6<double> ext_wrench_d_{Vector6<double>::Zero()};
  const RigidBodyTree<double>& robot_;
  const RigidBodyFrame<double> frame_T_;
  KinematicsCache<double> cache_;
};

class MoveToolFollowTraj : public MoveTool {
 public:
  MoveToolFollowTraj(
      const std::string& name, const RigidBodyTree<double>* robot,
      const RigidBodyFrame<double>* frame_T, const VectorX<double>& q0,
      const manipulation::PiecewiseCartesianTrajectory<double>& traj);

  void set_X_WT_traj(
      const manipulation::PiecewiseCartesianTrajectory<double>& traj) {
    X_WT_traj_ = traj;
  }

  bool IsDone(const IiwaState& state) const override;
  Isometry3<double> ComputeDesiredToolInWorld(
      const IiwaState& state) const override;
  void Control(const IiwaState& state, Eigen::Ref<VectorX<double>> q_d,
      Eigen::Ref<VectorX<double>> trq_d, lcmt_jjz_controller* msg) const override;

  //const Vector6<double>& get_desired_ext_wrench() const { return ext_wrench_d_; };
  //void set_desired_ext_wrench(const Vector6<double>& ext_wrench) { ext_wrench_d_ = ext_wrench; }
  void set_mu(double mu) { mu_ = mu; }
  void set_yaw_mu(double mu) { yaw_mu_ = mu; }
  void set_fz(double fz) { fz_ = fz; }

 private:
  manipulation::PiecewiseCartesianTrajectory<double> X_WT_traj_;
  // Vector6<double> ext_wrench_d_{Vector6<double>::Zero()};
  double vel_thres_{1e-3};
  double mu_{0};
  double yaw_mu_{0};
  double fz_{10};
};

}  // namespace jjz
}  // namespace examples
}  // namespace drake

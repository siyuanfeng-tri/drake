#pragma once

#include "drake/common/eigen_types.h"
#include "drake/examples/kuka_iiwa_arm/jjz_common.h"
#include "drake/multibody/rigid_body_tree.h"

#include "drake/lcmt_jjz_controller.hpp"
#include "drake/manipulation/planner/jacobian_ik.h"
#include "drake/manipulation/util/trajectory_utils.h"

namespace drake {
namespace jjz {

struct PrimitiveOutput {
  enum Status { UNINIT = 0, EXECUTING = 1, DONE = 2, ERR = 3 };
  Status status{UNINIT};
  VectorX<double> q_cmd;
  VectorX<double> trq_cmd;
  // This is only set with all cartesian mode primitives. MoveJ has this set to
  // I.
  Isometry3<double> X_WT_cmd{Isometry3<double>::Identity()};
};

class MotionPrimitive {
 public:
  MotionPrimitive(const std::string& name) : name_(name) {}
  virtual ~MotionPrimitive() {
    std::cout << "[" << get_name() << "] exiting.\n";
  }

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

  void Control(const IiwaState& state, PrimitiveOutput* output,
               lcmt_jjz_controller* msg) const {
    // Set default values.
    const int dim = get_dimension();
    output->status = PrimitiveOutput::EXECUTING;
    output->q_cmd.resize(dim);
    output->trq_cmd.resize(dim);
    output->q_cmd.setZero();
    output->trq_cmd.setZero();

    output->X_WT_cmd.setIdentity();

    DoControl(state, output, msg);
  }

  void set_name(const std::string& name) { name_ = name; }

  virtual void Update(const IiwaState& state, lcmt_jjz_controller* msg) {}

  virtual int get_dimension() const = 0;

 protected:
  virtual void DoInitialize(const IiwaState& state) {}
  virtual void DoControl(const IiwaState& state, PrimitiveOutput* output,
                         lcmt_jjz_controller* msg) const {}

 private:
  std::string name_;
  double start_time_{0};
  bool init_{false};
};

class MoveJoint : public MotionPrimitive {
 public:
  MoveJoint(const std::string& name, const VectorX<double>& q0,
            const VectorX<double>& q1, double duration);

  void DoControl(const IiwaState& state, PrimitiveOutput* output,
                 lcmt_jjz_controller* msg) const override;

  int get_dimension() const override { return traj_.rows() * traj_.cols(); }

 private:
  PiecewisePolynomial<double> traj_;
};

class MoveTool : public MotionPrimitive {
 public:
  MoveTool(const std::string& name, const RigidBodyTree<double>* robot,
           const RigidBodyFrame<double>* frame_T, const VectorX<double>& q0);

  void Update(const IiwaState& state, lcmt_jjz_controller* msg) override;

  virtual Isometry3<double> ComputeDesiredToolInWorld(
      const IiwaState& state) const = 0;

  Isometry3<double> get_X_WT_ik() const {
    return robot_.CalcFramePoseInWorldFrame(cache_, frame_T_);
  }

  void set_nominal_q(const VectorX<double>& q_norm) { q_norm_ = q_norm; }
  void set_tool_gain(const Vector6<double>& gain) { gain_T_ = gain; }

  const RigidBodyTree<double>& get_robot() const { return robot_; }
  const RigidBodyFrame<double>& get_tool_frame() const { return frame_T_; }

  int get_dimension() const final { return robot_.get_num_positions(); }

 protected:
  void DoInitialize(const IiwaState& state) override;
  void DoControl(const IiwaState& state, PrimitiveOutput* output,
                 lcmt_jjz_controller* msg) const override;

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
                             const RigidBodyTree<double>* robot,
                             const RigidBodyFrame<double>* frame_T,
                             const VectorX<double>& q0,
                             const Vector3<double>& dir, double vel);

  Isometry3<double> ComputeDesiredToolInWorld(
      const IiwaState& state) const override;

  double get_f_ext_thresh() const { return f_ext_thresh_; }
  void set_f_ext_thresh(double thresh) { f_ext_thresh_ = thresh; }

 private:
  void DoControl(const IiwaState& state, PrimitiveOutput* output,
                 lcmt_jjz_controller* msg) const override;

  Isometry3<double> X_WT0_;
  Vector3<double> dir_;
  double vel_;

  double f_ext_thresh_{10};
};

class HoldPositionAndApplyForce : public MotionPrimitive {
 public:
  HoldPositionAndApplyForce(const std::string& name,
                            const RigidBodyTree<double>* robot,
                            const RigidBodyFrame<double>* frame_T);

  void Update(const IiwaState& state, lcmt_jjz_controller* msg) override;

  const Vector6<double>& get_desired_ext_wrench() const {
    return ext_wrench_d_;
  }
  void set_desired_ext_wrench(const Vector6<double>& w) { ext_wrench_d_ = w; }

  int get_dimension() const final { return robot_.get_num_positions(); }

 private:
  void DoInitialize(const IiwaState& state) override { q0_ = state.get_q(); }
  void DoControl(const IiwaState& state, PrimitiveOutput* output,
                 lcmt_jjz_controller* msg) const override;

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

  Isometry3<double> ComputeDesiredToolInWorld(
      const IiwaState& state) const override;

  void set_mu(double mu) { mu_ = mu; }
  void set_yaw_mu(double mu) { yaw_mu_ = mu; }
  void set_fz(double fz) { fz_ = fz; }

 private:
  void DoControl(const IiwaState& state, PrimitiveOutput* output,
                 lcmt_jjz_controller* msg) const override;

  manipulation::PiecewiseCartesianTrajectory<double> X_WT_traj_;

  double vel_thres_{1e-3};
  double mu_{0};
  double yaw_mu_{0};
  double fz_{10};
};

}  // namespace jjz
}  // namespace drake

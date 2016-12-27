#pragma once

#include "drake/examples/QPInverseDynamicsForHumanoids/generic_plan.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

struct footstep_t {
  bool is_right_foot;
  Isometry3<double> pose;
};

class HumanoidWalkingPlan : public GenericHumanoidPlan {
 public:
  HumanoidWalkingPlan(const RigidBodyTree<double>& robot);

  void HandleWalkingPlan(const HumanoidStatus&rs);

  bool DoStateTransition(const HumanoidStatus& rs) override;

 private:
  enum WalkingState {
    WEIGHT_TRANSFER,
    SWING
  };

  WalkingState cur_state_;
  std::list<footstep_t> footstep_plan_;

  void SwitchContact(double plan_time);
  void GenerateTrajs(double time, const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const ContactState& planned_contacts);

  Eigen::Vector2d Footstep2DesiredZMP(Side side, const Eigen::Isometry3d &step) const;
  PiecewisePolynomial<double> PlanZMPTraj(const std::vector<Eigen::Vector2d> &zmp_d, int num_of_zmp_knots, const Eigen::Vector2d &zmp_d0, const Eigen::Vector2d &zmpd_d0, double time_before_first_weight_shift) const;
  void GeneratePelvisTraj(const KinematicsCache<double>& cache,
    double pelvis_height_above_sole, double liftoff_time,
    double next_liftoff_time,
    const Eigen::Isometry3d& nxt_stance_foot_pose,
    const Eigen::Isometry3d& nxt_swing_foot_pose);

  void GenerateTorsoTraj(const KinematicsCache<double>& cache,
    double next_liftoff_time,
    const Eigen::Isometry3d& nxt_swing_foot_pose);

  void GenerateSwingTraj(const RigidBody<double>* swing_foot,
      const Isometry3<double>& foot0,
      const Isometry3<double>& foot1,
      double mid_z_offset,
      double pre_swing_dur, double swing_up_dur,
      double swing_transfer_dur, double swing_down_dur);

  int step_count_ = 0;

  double p_ss_duration_ = 1;
  double p_ds_duration_ = 0.5;
  double p_pelvis_height_ = 0.88;
  double p_swing_foot_touchdown_z_vel_ = -0.05;
  double p_swing_foot_touchdown_z_offset_ = -0.00;
  double p_step_height_ = 0.05;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

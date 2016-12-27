#pragma once

#include <set>

#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial_base.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/systems/controllers/zmp_planner.h"

#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

typedef std::map<const RigidBody<double>*, ContactInformation> ContactState;

class PiecewiseContactInformation : public PiecewiseFunction {
 public:
  PiecewiseContactInformation() {}

  PiecewiseContactInformation(const std::vector<double>& time, const std::vector<ContactState>& contact_info)
    : PiecewiseFunction(time), contact_info_segments_(contact_info) {
    DRAKE_DEMAND(!contact_info.empty());
    DRAKE_DEMAND(time.size() == contact_info.size() + 1);
  }

  Eigen::Index rows() const { return 1; }
  Eigen::Index cols() const { return 1; }

  const ContactState& get_contacts(double time) const {
    int seg_idx = getSegmentIndex(time);
    DRAKE_DEMAND(seg_idx >= 0 && seg_idx < static_cast<int>(contact_info_segments_.size()));

    return contact_info_segments_.at(seg_idx);
  }

 private:
  std::vector<ContactState> contact_info_segments_;
};

class GenericHumanoidPlan {
 protected:
  // TODO;
  double interp_t0_ {0};
  double planned_contact_switch_time_ {0};
  // hack
  mutable bool init_qp_input_ {true};

  // robot for doing kinematics
  const RigidBodyTree<double>* robot_;
  const RigidBody<double>* left_foot_;
  const RigidBody<double>* right_foot_;
  const RigidBody<double>* pelvis_;
  const RigidBody<double>* torso_;

  VectorX<double> q_;
  VectorX<double> v_;

  // estimated robot state
  VectorX<double> est_q_;
  VectorX<double> est_v_;

  /////////////
  // These are set by HandlePlan in derived classes.
  std::set<const RigidBody<double>*> tracked_bodies_;
  PiecewiseContactInformation contacts_traj_;

  // FB controllers
  systems::ZMPPlanner zmp_planner_;
  VectorTrajectoryTracker<double> dof_tracker_;
  std::map<const RigidBody<double>*, CartesianTrajectoryTracker<double>> body_trackers_;

  // Params
  std::map<const RigidBody<double>*, DesiredBodyMotion> all_trackable_bodies_;
  std::map<const RigidBody<double>*, ContactInformation> all_contactable_bodies_;
  ContactInformation MakeDefaultFootContactInformation(const RigidBody<double>* body) const;

  Vector6<double> Kp_pelvis_;
  Vector6<double> Kd_pelvis_;

  Vector6<double> Kp_foot_;
  Vector6<double> Kd_foot_;

  VectorX<double> Kp_joints_;
  VectorX<double> Kd_joints_;

  double com_height_ = 1;
  // Also need to have weights

  virtual void InitializeQPInput(const HumanoidStatus& rs, QPInput* qp_input) const;

 public:
  GenericHumanoidPlan(const RigidBodyTree<double>& robot);

  // Make QPInput
  virtual void UpdateQPInput(const HumanoidStatus& rs, QPInput* qp_input) const;

  // Trigger event changes.
  virtual bool DoStateTransition(const HumanoidStatus& rs)=0;

  // Handle plan

  // Contact state related.
  ContactState double_support() const {
    ContactState cs;
    cs.emplace(left_foot_, all_contactable_bodies_.at(left_foot_));
    cs.emplace(right_foot_, all_contactable_bodies_.at(right_foot_));
    return cs;
  }

  ContactState single_support_left() const {
    ContactState cs;
    cs.emplace(left_foot_, all_contactable_bodies_.at(left_foot_));
    return cs;
  }

  ContactState single_support_right() const {
    ContactState cs;
    cs.emplace(right_foot_, all_contactable_bodies_.at(right_foot_));
    return cs;
  }

  bool is_foot_contact(const ContactState& cs, Side side) const {
    if (side == Side::LEFT && cs.find(left_foot_) != cs.end()) {
      return true;
    } else if (side == Side::RIGHT && cs.find(right_foot_) != cs.end()) {
      return true;
    } else {
      return false;
    }
  }

  bool is_both_feet_contact(const ContactState& cs) const {
    return is_foot_contact(cs, Side::LEFT) && is_foot_contact(cs, Side::RIGHT);
  }
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

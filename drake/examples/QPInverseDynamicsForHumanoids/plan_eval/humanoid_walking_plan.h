#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/humanoid_plan.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

template <typename T>
struct Footstep {
  Side side;
  Isometry3<T> pose;
};

template <typename T> class FootstepSequence {
 private:
  std::list<Footstep<T>> footsteps_;
  int steps_taken_{0};

 public:
  int size() const { return static_cast<int>(footsteps_.size()); }
  int get_steps_taken() const { return steps_taken_; }
  bool empty() const { return footsteps_.empty(); }
  const Footstep<T>& get_current_footstep() const { return footsteps_.front(); }
  const std::list<Footstep<T>>& get_footsteps() const { return footsteps_; }

  const RigidBody<T>* get_current_stance_foot(
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) const {
    if (footsteps_.empty()) {
      return nullptr;
    }
    if (footsteps_.front().side == Side::LEFT) {
      return alias_groups.get_body("right_foot");
    }
    return alias_groups.get_body("left_foot");
  }

  const RigidBody<T>* get_current_swing_foot(
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) const {
    if (footsteps_.empty()) {
      return nullptr;
    }
    if (footsteps_.front().side == Side::LEFT) {
      return alias_groups.get_body("left_foot");
    }
    return alias_groups.get_body("right_foot");
  }

  void take_a_step() {
    footsteps_.pop_front();
    steps_taken_++;
  }

  void reset() {
    footsteps_.clear();
    steps_taken_ = 0;
  }

  bool is_first_step() const {
    return steps_taken_ == 0;
  }

  bool is_last_step() const {
    return footsteps_.size() == 1;
  }

  void push_back(const Footstep<T> footstep) { footsteps_.push_back(footstep); }
};

template <typename T>
class HumanoidWalkingPlan : public HumanoidPlan<T> {
 public:
  HumanoidWalkingPlan() {}

 private:
  struct WalkingEvent {
    T time{0};
    std::function<void(HumanoidWalkingPlan<T>*, const HumanoidStatus&,
                       const param_parsers::ParamSet&,
                       const param_parsers::RigidBodyTreeAliasGroups<T>&)>
        action_func;

    bool is_triggered(const HumanoidStatus& robot_status) const {
      return (robot_status.time() >= time);
    }
  };

  enum WalkingState { HOLD = 0, WEIGHT_SHIFT, SWING };

  HumanoidPlan<T>* CloneHumanoidPlanDerived() const;

  void InitializeHumanoidPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups);

  void HandlePlanMessageGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      const void* message_bytes, int message_length);

  void ExecutePlanGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups);

  void UpdatePlanOnTouchdown(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups);

  void UpdatePlanOnLiftoff(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups);

  PiecewisePolynomial<T> GenerateDesiredZmpTrajectory(
      const ContactState& current_contact_state,
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      T* liftoff_time) const;

  std::pair<const RigidBody<T>*, PiecewiseCartesianTrajectory<T>>
  GenerateDesiredSwingFootTrajectory(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      const Isometry3<T>& touchdown_pose) const;

  std::pair<const RigidBody<T>*, PiecewiseCartesianTrajectory<T>>
  GenerateDesiredSwingPelvisTrajectory(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      const Isometry3<T>& stance_foot_pose,
      const Isometry3<T>& touchdown_pose) const;

  std::pair<const RigidBody<T>*, PiecewiseCartesianTrajectory<T>>
  GenerateDesiredSwingTorsoTrajectory(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      const Isometry3<T>& stance_foot_pose,
      const Isometry3<T>& touchdown_pose) const;

  Vector2<T> ComputeMidFootXyFromFootPose(
      const Isometry3<T>& foot_pose,
      const param_parsers::ParamSet& paramset) const;

  T ComputePelvisYaw(const Isometry3<T>& stance_foot_pose,
                     const Isometry3<T>& touchdown_foot_pose) const;
  T ComputePelvisHeight(const Isometry3<T>& stance_foot_pose,
                        const Isometry3<T>& touchdown_foot_pose) const;

  // foot step and crap
  FootstepSequence<T> footsteps_;
  WalkingEvent next_event_;
  WalkingState state_;

  // TODO these need to go to params.
  const T pelvis_height_{0.9};
  const T ss_duration_{1};
  const T ds_duration_{0.5};
  const T extra_time_{1};
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

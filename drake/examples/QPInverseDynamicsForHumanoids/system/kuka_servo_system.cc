#include "drake/examples/QPInverseDynamicsForHumanoids/system/kuka_servo_system.h"

#include <vector>

#include "drake/common/drake_path.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller_common.h"

#include "drake/common/trajectories/piecewise_quaternion.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

// An example plan that follows a desired trajetory.
struct KukaServoPlan {
  VectorSetpoint<double> joint_PDff;
};

KukaServoSystem::KukaServoSystem(
    const RigidBodyTree<double>& robot,
    const std::string& alias_groups_file_name,
    const std::string& param_file_name, double dt)
    : DiscreteTimePlanEvalSystem(robot, alias_groups_file_name, param_file_name,
                                 dt) {
  input_port_index_desired_state_ = DeclareInputPort(systems::kVectorValued, robot_.get_num_positions() + robot_.get_num_velocities()).get_index();
  last_v_d_ = VectorX<double>::Zero(robot_.get_num_velocities());
  set_name("kuka_plan_eval");
}

void KukaServoSystem::Initialize(systems::State<double>* state) {
  KukaServoPlan& plan = get_mutable_plan<KukaServoPlan>(state);
  DRAKE_DEMAND(robot_.get_num_velocities() == robot_.get_num_positions());
  plan.joint_PDff = VectorSetpoint<double>(robot_.get_num_velocities());
  paramset_.LookupDesiredDofMotionGains(&(plan.joint_PDff.mutable_Kp()),
                                        &(plan.joint_PDff.mutable_Kd()));

  QpInput& qp_input = get_mutable_qp_input(state);
  qp_input = paramset_.MakeQpInput({}, /* contacts */
                                   {}, /* tracked bodies */
                                   alias_groups_);
}

void KukaServoSystem::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  // Gets the plan from abstract state.
  KukaServoPlan& plan = get_mutable_plan<KukaServoPlan>(state);

  // Gets the robot state from input.
  const HumanoidStatus* robot_status = EvalInputValue<HumanoidStatus>(
      context, input_port_index_humanoid_status_);

  // Gets the desired position and velocity.
  const systems::BasicVector<double>* desireds = EvalVectorInput(context, input_port_index_desired_state_);

  QpInput& qp_input = get_mutable_qp_input(state);
  qp_input = paramset_.MakeQpInput({}, /* contacts */
                                   {}, /* tracked bodies */
                                   alias_groups_);

  for (int i = 0; i < robot_.get_num_positions(); i++) {
    plan.joint_PDff.mutable_desired_position()[i] = desireds->GetAtIndex(i);
    plan.joint_PDff.mutable_desired_velocity()[i] = desireds->GetAtIndex(i + robot_.get_num_positions());
  }

  plan.joint_PDff.mutable_desired_acceleration() = (plan.joint_PDff.mutable_desired_velocity() - last_v_d_) / control_dt_;
  last_v_d_ = plan.joint_PDff.mutable_desired_velocity();

  // Update desired accelerations.
  qp_input.mutable_desired_dof_motions().mutable_values() =
      plan.joint_PDff.ComputeTargetAcceleration(robot_status->position(),
                                                robot_status->velocity());
}

std::unique_ptr<systems::AbstractState>
KukaServoSystem::AllocateAbstractState() const {
  std::cout << "alloc abs state\n";
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals(2);
  abstract_vals[abstract_state_plan_index_] =
      std::unique_ptr<systems::AbstractValue>(
          new systems::Value<KukaServoPlan>(KukaServoPlan()));
  abstract_vals[abstract_state_qp_input_index_] =
      std::unique_ptr<systems::AbstractValue>(new systems::Value<QpInput>(
          paramset_.MakeQpInput({}, /* contacts */
                                {}, /* tracked bodies */
                                alias_groups_)));
  return std::make_unique<systems::AbstractState>(std::move(abstract_vals));
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

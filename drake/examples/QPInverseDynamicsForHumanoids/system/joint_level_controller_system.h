#pragma once

#include "drake/util/drakeUtil.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

using systems::Context;
using systems::SystemOutput;
using systems::SystemPortDescriptor;
using systems::LeafSystemOutput;
using systems::AbstractValue;
using systems::Value;

class JointLevelControllerSystem : public systems::LeafSystem<double> {
 public:
  JointLevelControllerSystem(const RigidBodyTree& robot) : robot_(robot) {
    in_port_idx_qp_output_ =
        DeclareAbstractInputPort(systems::kInheritedSampling).get_index();
    in_port_idx_humanoid_status_ =
        DeclareAbstractInputPort(systems::kInheritedSampling).get_index();

    int act_size = robot_.actuators.size();
    // Load gains from some config.
    k_q_p_ = VectorX<double>::Zero(act_size);
    k_q_i_ = VectorX<double>::Zero(act_size);
    k_qd_p_ = VectorX<double>::Zero(act_size);
    k_f_p_ = VectorX<double>::Zero(act_size);
    ff_qd_ = VectorX<double>::Zero(act_size);
    ff_qd_d_ = VectorX<double>::Zero(act_size);
    // Directly feed torque through without any other feedbacks.
    ff_f_d_ = VectorX<double>::Constant(act_size, 1.);
    ff_const_ = VectorX<double>::Zero(act_size);
  }

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override {}

  void DoPublish(const Context<double>& context) const override {
    // Inputs
    const QPOutput* qp_output =
        EvalInputValue<QPOutput>(context, in_port_idx_qp_output_);

    const HumanoidStatus* rs = EvalInputValue<HumanoidStatus>(
        context, in_port_idx_humanoid_status_);

    // Output
    bot_core::atlas_command_t msg;

    // Make message.
    msg.utime = static_cast<uint64_t>(rs->time() * 1e6);

    int act_size = robot_.actuators.size();
    msg.num_joints = act_size;
    msg.joint_names.resize(msg.num_joints);
    msg.position.resize(msg.num_joints);
    msg.velocity.resize(msg.num_joints);
    msg.effort.resize(msg.num_joints);

    // Set desired position, velocity and torque
    for (int i = 0; i < act_size; ++i) {
      msg.joint_names[i] = robot_.actuators[i].name_;

      msg.position[i] = 0;
      msg.velocity[i] = 0;
      double act_trq = robot_.B.col(i).transpose() * qp_output->dof_torques();
      msg.effort[i] = act_trq;

      std::cout << msg.joint_names[i] << ": " << msg.effort[i] << std::endl;
    }

    eigenVectorToStdVector(k_q_p_, msg.k_q_p);
    eigenVectorToStdVector(k_q_i_, msg.k_q_i);
    eigenVectorToStdVector(k_qd_p_, msg.k_qd_p);
    eigenVectorToStdVector(k_f_p_, msg.k_f_p);
    eigenVectorToStdVector(ff_qd_, msg.ff_qd);
    eigenVectorToStdVector(ff_qd_d_, msg.ff_qd_d);
    eigenVectorToStdVector(ff_f_d_, msg.ff_f_d);
    eigenVectorToStdVector(ff_const_, msg.ff_const);

    // This is only used for gazebo simulator. Should be deprecated.
    msg.k_effort.resize(msg.num_joints);
  }

  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const Context<double>& context) const override {
    std::unique_ptr<LeafSystemOutput<double>> output(
        new LeafSystemOutput<double>);
    output->add_port(std::unique_ptr<AbstractValue>(new Value<bot_core::atlas_command_t>(bot_core::atlas_command_t())));
    return std::move(output);
  }

  /**
   * @return the input port number that corresponds to: humanoid status.
   */
  inline const SystemPortDescriptor<double>& get_input_port_humanoid_status()
      const {
    return get_input_port(in_port_idx_humanoid_status_);
  }

  inline const SystemPortDescriptor<double>& get_input_port_qp_output()
      const {
    return get_input_port(in_port_idx_qp_output_);
  }

 private:
  const RigidBodyTree& robot_;

  int in_port_idx_qp_output_;
  int in_port_idx_humanoid_status_;

  // Joint level gains, these are in actuator order.
  VectorX<double> k_q_p_;
  VectorX<double> k_q_i_;
  VectorX<double> k_qd_p_;
  VectorX<double> k_f_p_;
  VectorX<double> ff_qd_;
  VectorX<double> ff_qd_d_;
  VectorX<double> ff_f_d_;
  VectorX<double> ff_const_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

#pragma once

#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"

#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/util/drakeUtil.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

using systems::Context;
using systems::SystemOutput;
using systems::SystemPortDescriptor;
using systems::LeafSystemOutput;
using systems::AbstractValue;
using systems::Value;

/**
 * A stub for a more complex interface to joint level control.
 * The idea is to separate all joint level control from the higher level
 * full state feedback controller, e.g. qp inverse dynamics controller.
 * This can also run at a higher rate than the full state feedback controller.
 *
 * Possible things to be implemented here:
 *  set points, gains, integrators, filters
 *
 * Input: HumanoidStatus
 * Input: QPOutput
 * Output: lcm message bot_core::atlas_command_t in channel "ROBOT_COMMAND"
 */
class JointLevelControllerSystem : public systems::LeafSystem<double> {
 public:
  JointLevelControllerSystem(const RigidBodyTree& robot,
                             drake::lcm::DrakeLcmInterface* lcm)
      : robot_(robot), lcm_(lcm) {
    in_port_idx_qp_output_ =
        DeclareAbstractInputPort(systems::kInheritedSampling).get_index();
    in_port_idx_humanoid_status_ =
        DeclareAbstractInputPort(systems::kInheritedSampling).get_index();

    int act_size = robot_.actuators.size();
    // TODO(siyuan.fent): Load gains from some config.
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

    const HumanoidStatus* rs =
        EvalInputValue<HumanoidStatus>(context, in_port_idx_humanoid_status_);

    // Make message.
    std::vector<uint8_t> raw_bytes;
    bot_core::atlas_command_t msg;
    msg.utime = static_cast<uint64_t>(rs->time() * 1e6);

    int act_size = robot_.actuators.size();
    msg.num_joints = act_size;
    msg.joint_names.resize(msg.num_joints);
    msg.position.resize(msg.num_joints);
    msg.velocity.resize(msg.num_joints);
    msg.effort.resize(msg.num_joints);

    // Compute actuator torques from dof torques.
    VectorX<double> act_torques =
        robot_.B.transpose() * qp_output->dof_torques();

    // Set desired position, velocity and torque for all actuators.
    for (int i = 0; i < act_size; ++i) {
      msg.joint_names[i] = robot_.actuators[i].name_;
      msg.position[i] = 0;
      msg.velocity[i] = 0;
      msg.effort[i] = act_torques[i];
    }

    eigenVectorToStdVector(k_q_p_, msg.k_q_p);
    eigenVectorToStdVector(k_q_i_, msg.k_q_i);
    eigenVectorToStdVector(k_qd_p_, msg.k_qd_p);
    eigenVectorToStdVector(k_f_p_, msg.k_f_p);
    eigenVectorToStdVector(ff_qd_, msg.ff_qd);
    eigenVectorToStdVector(ff_qd_d_, msg.ff_qd_d);
    eigenVectorToStdVector(ff_f_d_, msg.ff_f_d);
    eigenVectorToStdVector(ff_const_, msg.ff_const);

    // This is only used for the Virtural Robotics Challenge's gazebo simulator.
    // Should be deprecated by now.
    msg.k_effort.resize(msg.num_joints, 0);
    // TODO(siyuan.feng): I am not sure what this does exactly. Probably also
    // deprecated.
    msg.desired_controller_period_ms = 0;

    // Encode and send the lcm message.
    int msg_size = msg.getEncodedSize();
    raw_bytes.resize(msg_size);
    msg.encode(raw_bytes.data(), 0, msg_size);
    lcm_->Publish("ROBOT_COMMAND", raw_bytes.data(), msg_size);
  }

  /**
   * @return Port for the input: HumanoidStatus.
   */
  inline const SystemPortDescriptor<double>& get_input_port_humanoid_status()
      const {
    return get_input_port(in_port_idx_humanoid_status_);
  }

  /**
   * @return Port for the input: QPOutput.
   */
  inline const SystemPortDescriptor<double>& get_input_port_qp_output() const {
    return get_input_port(in_port_idx_qp_output_);
  }

 private:
  const RigidBodyTree& robot_;

  int in_port_idx_qp_output_;
  int in_port_idx_humanoid_status_;

  // LCM publishing interface
  drake::lcm::DrakeLcmInterface* const lcm_;

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

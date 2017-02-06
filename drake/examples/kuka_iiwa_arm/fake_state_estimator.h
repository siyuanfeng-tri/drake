#pragma once

#include <memory>
#include <string>

#include "bot_core/robot_state_t.hpp"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/util/lcmUtil.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

class FakeStateEstimator : public systems::LeafSystem<double> {
 public:
  FakeStateEstimator(const RigidBodyTree<double>& robot)
    : robot_(robot),
      floating_body_(robot.bodies[1]->getJoint().is_floating() ? robot.bodies[1].get() : nullptr) {
    input_port_index_state_ = DeclareInputPort(systems::kVectorValued, robot.get_num_positions() + robot.get_num_velocities()).get_index();
    output_port_index_msg_ = DeclareAbstractOutputPort().get_index();
  }

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override {
    const systems::BasicVector<double>* state = EvalVectorInput(context, input_port_index_state_);
    VectorX<double> q = state->get_value().head(robot_.get_num_positions());
    VectorX<double> v = state->get_value().tail(robot_.get_num_velocities());
    KinematicsCache<double> cache = robot_.doKinematics(q, v);

    bot_core::robot_state_t& msg = output->GetMutableData(output_port_index_msg_)->GetMutableValue<bot_core::robot_state_t>();
    msg.utime = static_cast<int64_t>(context.get_time() * 1e6);
    Isometry3<double> floating_body_to_world = Isometry3<double>::Identity();
    TwistVector<double> floating_body_velocity = Vector6<double>::Zero();
    if (floating_body_) {
      // Pose of floating body with respect to world.
      floating_body_to_world = robot_.CalcBodyPoseInWorldFrame(cache, *floating_body_);
      floating_body_velocity = robot_.CalcBodySpatialVelocityInWorldFrame(cache, *floating_body_);
    }
    EncodePose(floating_body_to_world, msg.pose);
    EncodeTwist(floating_body_velocity, msg.twist);

    // Joint names, positions, velocities, and efforts.
    // Note: the order of the actuators in the rigid body tree determines the
    // order of the joint_name, joint_position, joint_velocity, and
    // joint_effort fields.
    msg.joint_name.resize(robot_.get_num_actuators());
    msg.joint_position.resize(robot_.get_num_actuators());
    msg.joint_velocity.resize(robot_.get_num_actuators());
    msg.joint_effort.resize(robot_.get_num_actuators());
    msg.num_joints = static_cast<int16_t>(msg.joint_name.size());
    int i = 0;
    for (const auto& actuator : robot_.actuators) {
      const auto& body = *actuator.body_;

      // To match usage of robot_state_t throughout OpenHumanoids code, set
      // joint_names field to position coordinate names.
      int position_index = body.get_position_start_index();
      int velocity_index = body.get_position_start_index();
      msg.joint_name[i] = robot_.get_position_name(position_index);

      msg.joint_position[i] = static_cast<float>(q[position_index]);
      msg.joint_velocity[i] = static_cast<float>(v[velocity_index]);
      msg.joint_effort[i] = 0;
      i++;
    }
  }

  std::unique_ptr<systems::AbstractValue> AllocateOutputAbstract(
      const systems::OutputPortDescriptor<double>& descriptor) const override {
    return systems::AbstractValue::Make<bot_core::robot_state_t>(bot_core::robot_state_t());
  }

  inline const systems::InputPortDescriptor<double>& get_input_port_state() const {
    return get_input_port(input_port_index_state_);
  }

  inline const systems::OutputPortDescriptor<double>& get_output_port_msg() const {
    return get_output_port(output_port_index_msg_);
  }

 private:
  const RigidBodyTree<double>& robot_;
  const RigidBody<double>* const floating_body_{nullptr};
  int input_port_index_state_{0};
  int output_port_index_msg_{0};
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

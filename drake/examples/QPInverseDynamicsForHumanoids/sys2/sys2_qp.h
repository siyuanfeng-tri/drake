#pragma once

#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/translator_between_lcmt_drake_signal.h"
#include "drake/lcmt_drake_signal.hpp"

#include "../qp_controller.h"
#include "qp_io_utils.h"

namespace drake {
namespace systems {

class System2QP : public LeafSystem<double> {
 public:
  explicit System2QP(const std::string &urdf_name) 
    : urdf_name_(urdf_name), robot_(std::make_shared<RigidBodyTree>(urdf_name, DrakeJoint::ROLLPITCHYAW)) {

    // init some qp stuff
    RigidBodyTree robot(urdf_name, DrakeJoint::ROLLPITCHYAW);
    num_positions_ = robot.number_of_positions();
    num_velocities_ = robot.number_of_velocities();
    num_joints_ = robot.actuators.size();

    // make IO ports
    // state input
    DeclareInputPort(kVectorValued, number_of_robot_states(), kInheritedSampling);
    // qp input
    DeclareInputPort(kVectorValued, number_of_qp_inputs(), kInheritedSampling);

    // qp output
    DeclareOutputPort(kVectorValued, number_of_qp_outputs(), kInheritedSampling);
  }

  void EvalOutput(const ContextBase<double> &context, SystemOutput<double>* output) const override;

  inline int number_of_robot_states() const {
    return num_positions_ + num_velocities_ + num_joints_ + 1 + 12;
    //return get_humanoid_status_size(robot_status_);
  }
  inline int number_of_qp_inputs() const {
    return 3 + 6 * 4 + num_velocities_ + 12 + 6;
    //return get_qp_input_size(qp_input_);
  }
  inline int number_of_qp_outputs() const {
    return 3 + 6 * 4 + num_velocities_ + num_joints_ + 12 + 12;
    //return get_qp_output_size(qp_output_);
  }

 private:

  int num_positions_;
  int num_velocities_;
  int num_joints_;
  std::string urdf_name_;
  const std::shared_ptr<RigidBodyTree> robot_;
};


}
}

#pragma once

#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/basic_state_vector.h"

#include "../humanoid_status.h"
#include "../qp_controller.h"

namespace drake {
namespace systems {

class System2DummyValkyrieSim : public LeafSystem<double> {
 public:
  explicit System2DummyValkyrieSim(const std::string &urdf)
    : robot_(std::make_unique<RigidBodyTree>(urdf, DrakeJoint::ROLLPITCHYAW)) {

    // input: qp_output
    this->DeclareAbstractInputPort(kInheritedSampling);

    // output: huamnoids status
    this->DeclareAbstractOutputPort(kInheritedSampling);

    dt_ = 0.02;
  }

  std::unique_ptr<ContinuousState<double>> AllocateContinuousState() const override {
    int num_q = robot_->number_of_positions();
    int num_v = robot_->number_of_velocities();
    int num_x = num_q + num_v;
    return std::make_unique<ContinuousState<double>>(std::make_unique<BasicStateVector<double>>(num_x), num_q, num_v, 0);
  }

  void EvalOutput(const ContextBase<double> &context, SystemOutput<double>* output) const override {
    // get acceleration from qpouput
    const QPOutput &qpout = context.get_abstract_input(0)->GetValue<QPOutput>();
    const Eigen::VectorXd &vd = qpout.vd();

    // get state
    const StateVector<double> &xc = context.get_state().continuous_state->get_state();
    DRAKE_ASSERT(xc.size() == robot_->number_of_positions() + robot_->number_of_velocities());
    DRAKE_ASSERT(vd.size() == robot_->number_of_velocities());

    Eigen::VectorXd new_x = xc.CopyToVector();
    new_x.segment(0, robot_->number_of_positions()) += dt_ * new_x.segment(robot_->number_of_positions(), robot_->number_of_velocities());
    new_x.segment(robot_->number_of_positions(), robot_->number_of_velocities()) += dt_ * vd;

    HumanoidStatus rs(*robot_);
    rs.Update(context.get_time() + dt_,
        new_x.segment(0, robot_->number_of_positions()),
        new_x.segment(robot_->number_of_positions(), robot_->number_of_velocities()),
        qpout.joint_torque(),
        qpout.foot_wrench_in_sensor_frame(0),
        qpout.foot_wrench_in_sensor_frame(1));

    // set output
    output->GetMutableData(0)->SetValue(rs);
  }

  std::unique_ptr<SystemOutput<double>> AllocateOutput(const ContextBase<double>& context) const {
    std::unique_ptr<LeafSystemOutput<double>> output(new LeafSystemOutput<double>);
    HumanoidStatus rs(*robot_);
    output->add_port(std::unique_ptr<AbstractValue>(new Value<HumanoidStatus>(rs)));
    return std::unique_ptr<SystemOutput<double>>(output.release());
  }

  inline int size_of_state() const { return robot_->number_of_positions() + robot_->number_of_velocities(); }
  Eigen::VectorXd get_default_initial_state() const { 
    Eigen::VectorXd x0(size_of_state());
    HumanoidStatus rs(*robot_);
    x0.segment(0, robot_->number_of_positions()) = rs.GetNominalPosition();
    x0.segment(robot_->number_of_positions(), robot_->number_of_velocities()).setZero();
    return x0;
  }

 private:
  std::unique_ptr<RigidBodyTree> robot_;
  double dt_;

};

}
}

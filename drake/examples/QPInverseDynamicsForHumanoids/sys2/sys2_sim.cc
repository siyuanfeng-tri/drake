#include "drake/systems/analysis/simulator.h"
#include "sys2_dummy_val.h"
#include "sys2_qp.h"
#include "drake/common/drake_path.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/primitives/constant_value_source.h"
#include "drake/systems/framework/diagram_builder.h"

using namespace drake;
using namespace systems;

static std::string urdf = drake::GetDrakePath() + std::string("/examples/QPInverseDynamicsForHumanoids/valkyrie_sim_drake.urdf");
RigidBodyTree robot(urdf);

void const_acc_test() {
  System2DummyValkyrieSim val(robot);
  DiagramBuilder<double> builder;

  QPOutput out(robot);
  out.vd() = Eigen::VectorXd::Constant(robot.number_of_velocities(), 0.01);

  std::unique_ptr<System<double>> const_qp_out = std::make_unique<ConstantValueSource<double>>(std::unique_ptr<AbstractValue>(new Value<QPOutput>(out)));
  builder.Connect(const_qp_out->get_output_port(0), val.get_input_port(0));
  auto diagram = builder.Build();

  Simulator<double> simulator(*diagram);

  // set initial state
  // probably should pass in sub context?
  val.set_initial_state(simulator.get_mutable_context());

  simulator.request_initial_step_size_attempt(1e-2);
  simulator.Initialize();

  simulator.StepTo(0.1);
}

QPInput default_QP_input(const RigidBodyTree& r) {
  QPInput input(r);
  // Make input.
  // These represent the desired motions for the robot, and are typically
  // outputs of motion planner or hand-crafted behavior state machines.
  input.comdd_d().setZero();
  input.pelvdd_d().setZero();
  input.torsodd_d().setZero();
  input.footdd_d(Side::LEFT).setZero();
  input.footdd_d(Side::RIGHT).setZero();
  input.wrench_d(Side::LEFT).setZero();
  input.wrench_d(Side::RIGHT).setZero();
  input.vd_d().setZero();
  // [5] is Fz, 660N * 2 is about robot weight.
  input.wrench_d(Side::LEFT)[5] = 660;
  input.wrench_d(Side::RIGHT)[5] = 660;

  // Weights are set arbitrarily by the control designer, these typically
  // require tuning.
  input.w_com = 1e2;
  input.w_pelv = 1e1;
  input.w_torso = 1e1;
  input.w_foot = 1e5;
  input.w_vd = 1e3;
  input.w_wrench_reg = 1e-5;

  // make contact
  input.supports().push_back(
      SupportElement(*r.FindBody("leftFoot")));
  input.support(0).get_mutable_contact_points().push_back(
      Vector3d(0.2, 0.05, -0.09));
  input.support(0).get_mutable_contact_points().push_back(
      Vector3d(0.2, -0.05, -0.09));
  input.support(0).get_mutable_contact_points().push_back(
      Vector3d(-0.05, -0.05, -0.09));
  input.support(0).get_mutable_contact_points().push_back(
      Vector3d(-0.05, 0.05, -0.09));

  input.supports().push_back(
      SupportElement(*r.FindBody("rightFoot")));
  input.support(1).get_mutable_contact_points().push_back(
      Vector3d(0.2, 0.05, -0.09));
  input.support(1).get_mutable_contact_points().push_back(
      Vector3d(0.2, -0.05, -0.09));
  input.support(1).get_mutable_contact_points().push_back(
      Vector3d(-0.05, -0.05, -0.09));
  input.support(1).get_mutable_contact_points().push_back(
      Vector3d(-0.05, 0.05, -0.09));

  return input;
}

void close_loop_test() {
  System2DummyValkyrieSim val(robot);
  System2QP qp_con(robot);
  QPInput input = default_QP_input(robot);

  DiagramBuilder<double> builder;
  builder.Connect(qp_con.get_output_port(0), val.get_input_port(0));
  builder.Connect(val.get_output_port(0), qp_con.get_input_port(0));

  // qp input
  std::unique_ptr<System<double>> const_qp_input = std::make_unique<ConstantValueSource<double>>(std::unique_ptr<AbstractValue>(new Value<QPInput>(input)));
  builder.Connect(const_qp_input->get_output_port(0), qp_con.get_input_port(1));
  auto diagram = builder.Build();

  Simulator<double> simulator(*diagram);
  // probably should pass in sub context?
  val.set_initial_state(simulator.get_mutable_context());

  simulator.request_initial_step_size_attempt(1e-2);
  simulator.Initialize();
  simulator.StepTo(1.0);
}

int main() {

  close_loop_test();

  return 0;
}

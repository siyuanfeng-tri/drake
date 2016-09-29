#include "sys2_qp.h"
#include "sys2_dummy_val.h"

#include "drake/common/drake_path.h"

#include "../dummy_integrate.h"

std::string urdf = drake::GetDrakePath() + std::string("/examples/QPInverseDynamicsForHumanoids/valkyrie_sim_drake.urdf");


void test_sim() {
  drake::systems::System2DummyValkyrieSim sim(urdf);
  std::unique_ptr<drake::systems::ContextBase<double>> context = sim.CreateDefaultContext();
  std::unique_ptr<drake::systems::SystemOutput<double>> output = sim.AllocateOutput(*context);

  DRAKE_ASSERT(context->get_num_input_ports() == 1);
  DRAKE_ASSERT(output->get_num_ports() == 1);

  // init state
  context->set_time(0);
  context->get_mutable_state()->continuous_state->get_mutable_state()->SetFromVector(sim.get_default_initial_state());
}

int main() {

  RigidBodyTree robot(urdf, DrakeJoint::ROLLPITCHYAW);
  HumanoidStatus rs(robot);

  drake::systems::System2QP qp_con;
  qp_con.set_name("qp_controller");

  std::unique_ptr<drake::systems::ContextBase<double>> context = qp_con.CreateDefaultContext();
  std::unique_ptr<drake::systems::SystemOutput<double>> output = qp_con.AllocateOutput(*context);
  DRAKE_ASSERT(context->get_num_input_ports() == 2);
  DRAKE_ASSERT(output->get_num_ports() == 1);

  return 0;
}

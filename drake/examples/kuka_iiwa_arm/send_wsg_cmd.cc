#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_tree_builder.h"
#include "drake/examples/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/gravity_compensator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"

using namespace drake;
using namespace examples;
using namespace systems;

int main(int argc, char* argv[]) {
  DRAKE_DEMAND(argc == 2);

  drake::lcm::DrakeLcm lcmm;
  const std::string channel_name = "SCHUNK_WSG_COMMAND";

  // Instantiates the "device under test".
  auto dut = drake::systems::lcm::LcmPublisherSystem::Make<lcmt_schunk_wsg_command>(channel_name, &lcmm);
  std::unique_ptr<Context<double>> context = dut->AllocateContext();

  lcmt_schunk_wsg_command msg;
  msg.target_position_mm = atoi(argv[1]);

  context->FixInputPort(0, AbstractValue::Make<lcmt_schunk_wsg_command>(msg));

  lcmm.StartReceiveThread();

  dut->Publish(*context);

  sleep(1);

  return 0;
}

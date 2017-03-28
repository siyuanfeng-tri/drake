#include "drake/systems/lcm/lcm_driven_loop.h"

#include "bot_core/robot_state_t.hpp"
#include <unistd.h>

namespace drake {
namespace systems {

void test() {
  lcm::LcmDrivenLoop<double> dut;

  auto sys = systems::lcm::LcmSubscriberSystem::Make<bot_core::robot_state_t>(
      "EST_ROBOT_STATE", dut.get_mutable_lcm());

  dut.Initialize(*sys, nullptr, sys.get());

  dut.Run<bot_core::robot_state_t>();
}

}  // namespace systems
}  // namespace drake

int main() {
  drake::systems::test();

  return 0;
}

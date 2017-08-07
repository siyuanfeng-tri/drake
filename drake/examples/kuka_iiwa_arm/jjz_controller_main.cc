#include "drake/examples/kuka_iiwa_arm/jjz_controller.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"

int main() {
  const std::string kPath =
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf";
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      kPath, drake::multibody::joints::kFixed, nullptr, &tree);
  drake::examples::jjz::JjzController controller(tree);

  controller.Start();

  drake::examples::jjz::IiwaState state(&controller.get_robot(), &controller.get_tool_frame());
  drake::examples::jjz::PrimitiveOutput output;

  std::vector<drake::VectorX<double>> qs(2, controller.get_robot().getZeroConfiguration());
  qs[1](3) = -M_PI / 4;
  int idx = 1;

  while (true) {
    // Measured state.
    controller.GetState(&state);

    // Controller's output, contains status flag.
    controller.GetPrimitiveOutput(&output);

    // Check status, and swap new plan when the old one is done.
    if (output.status == drake::examples::jjz::PrimitiveOutput::DONE) {
      controller.MoveJ(qs[idx++], 2);
      idx = idx % 2;
    }

    // Sleep 100ms.
    usleep(1e5);
  }
}


#include "drake/examples/kuka_iiwa_arm/jjz_controller.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"

#include "drake/manipulation/util/trajectory_utils.h"

using namespace drake;

int main() {
  const std::string kPath =
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf";
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      kPath, drake::multibody::joints::kFixed, nullptr, &tree);

  RigidBodyFrame<double> tool_frame("tool", tree.FindBody(jjz::kEEName),
                                    jjz::X_ET);

  drake::jjz::JjzController controller(tree, tool_frame);

  controller.Start();

  drake::jjz::IiwaState state(&controller.get_robot(),
                              &controller.get_tool_frame());
  drake::jjz::PrimitiveOutput output;

  double last_time = -1;
  int script_idx = 0;
  while (true) {
    // Measured state.
    controller.GetIiwaState(&state);
    if (state.get_time() == last_time) continue;
    last_time = state.get_time();

    // Controller's output, contains status flag.
    controller.GetPrimitiveOutput(&output);

    // Check status, and swap new plan when the old one is done.
    if (output.status == drake::jjz::PrimitiveOutput::DONE) {
      switch (script_idx) {
        case 2: {
          VectorX<double> q = controller.get_robot().getZeroConfiguration();
          q[1] = 45. / 180. * M_PI;
          q[3] = -90. / 180. * M_PI;
          q[5] = 45. / 180. * M_PI;
          controller.MoveJ(q, 2);
          script_idx++;
          break;
        }
        case 0: {
          controller.OpenGripperAndSleep(1);
          script_idx++;
          break;
        }
        case 1: {
          controller.CloseGripperAndSleep(1);
          script_idx++;
          break;
        }

          /*
        case 1: {
          controller.MoveStraightUntilTouch(Vector3<double>(0, 0, -1), 0.03,
                                            10);
          script_idx++;
          break;
        }

          case 2: {
            Isometry3<double> X_WT0 = output.X_WT_cmd;
            Isometry3<double> X_WT1 =
                Eigen::Translation<double, 3>(Vector3<double>(0, -0.3, 0)) *
                X_WT0;

            // I picked timing from 0.5 ~ 3 second, so that the primitive holds
            // for 0.5s first to let all transient stuff stable down.
            manipulation::PiecewiseCartesianTrajectory<double> traj =
                manipulation::PiecewiseCartesianTrajectory<double>::
                    MakeCubicLinearWithEndLinearVelocity({0.5, 3}, {X_WT0,
          X_WT1},
                                                         Vector3<double>::Zero(),
                                                         Vector3<double>::Zero());

            controller.MoveToolFollowTraj(traj, 10, 1, 0);
            script_idx++;
            break;
          }
          */
      }
    }

    // Sleep 100ms.
    // usleep(1e5);
  }
}

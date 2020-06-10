#include <ctime>
#include <iomanip>
#include <sstream>
#include <experimental/filesystem>
#include <gflags/gflags.h>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/examples/manipulation_station/lcm_blocking_trigger.h"
#include "drake/examples/manipulation_station/pdc_common.h"
#include "drake/examples/manipulation_station/pdc_ik.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"

#include "drake/lcmt_force_torque.hpp"

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(duration, 1e33, "Simulation duration.");
DEFINE_bool(
    test, false,
    "Set to true when testing the trained policy, false to collect data.");

DEFINE_double(record_start, 3, "");
DEFINE_double(record_period, 0.1, "");

DEFINE_string(model_name, "", "model_name");
DEFINE_string(model_dir, "", "Directory for models");
DEFINE_string(output_dir, "/home/sfeng/tmp/pdc_img/", "output directory");

namespace drake {
namespace examples {
namespace manipulation_station {
namespace {

using math::RigidTransform;

std::string GetExperimentRootDir() {
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%d-%H-%M-%S");
  std::string output_dir = FLAGS_output_dir;
  if (output_dir.back() != '/') output_dir += "/";
  return output_dir + oss.str();
}

math::RigidTransform<double> ComputeX_WT_desired(
    const math::RigidTransform<double>& X_WObj) {
  Eigen::Vector3d y_W = X_WObj.matrix().col(0).head<3>();
  Eigen::Vector3d x_W = y_W.cross(Eigen::Vector3d::UnitZ());
  Eigen::Vector3d z_W = x_W.cross(y_W);
  Eigen::Matrix3d rot;
  rot.col(0) = x_W;
  rot.col(1) = y_W;
  rot.col(2) = z_W;
  math::RotationMatrix<double> R =
      math::RotationMatrix<double>::ProjectToRotationMatrix(rot);
  math::RigidTransform<double> result(R, X_WObj.translation());
  return result * math::RigidTransform<double>(Eigen::Vector3d(0, 0, 0.3));
}

math::RigidTransform<double> ComputeX_WT(
    const multibody::MultibodyPlant<double>& plant, const Eigen::VectorXd& q,
    systems::Context<double>* context) {
  const auto& tool_frame = plant.GetFrameByName("tool_frame");
  plant.SetPositions(context, q);
  return plant.CalcRelativeTransform(*context, plant.world_frame(), tool_frame);
}

Eigen::VectorXd ComputeControl(
    const math::RigidTransform<double>& X_WT_now,
    const math::RigidTransform<double>& X_WT_desired) {
  Vector6<double> diff_W = manipulation::planner::ComputePoseDiffInCommonFrame(
      X_WT_now, X_WT_desired);
  return diff_W;
}

Eigen::VectorXd WaitForRemoteControl(
    double time, const math::RigidTransform<double>& X_WT_now) {
  LcmBlockingTrigger<lcmt_force_torque> handle("REMOTE_V_CMD", "");
  lcmt_force_torque response{};
  lcmt_force_torque request{};
  request.timestamp = static_cast<int64_t>(time * 1e6);
  handle.PublishTriggerAndWaitForAck(request, &response);

  Vector6<double> diff_T = Vector6<double>::Zero();
  diff_T[0] = response.tx;
  diff_T[1] = response.ty;
  diff_T[2] = response.tz;
  diff_T[3] = response.fx;
  diff_T[4] = response.fy;
  diff_T[5] = response.fz;

  diff_T.head<3>() = X_WT_now.rotation() * diff_T.head<3>();
  diff_T.tail<3>() = X_WT_now.rotation() * diff_T.tail<3>();
  return diff_T;
}

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  DataRecordParams record_params;
  record_params.record_period = FLAGS_record_period;
  record_params.record_root_dir = GetExperimentRootDir();
  record_params.record_start_time = FLAGS_record_start;

  const bool viz_point_cloud = false;
  MegaDiagramStuff stuff =
      build_pdc_mega_diagram(FLAGS_model_name, FLAGS_model_dir, viz_point_cloud,
                             record_params, std::nullopt);
  systems::Diagram<double>* diagram = stuff.diagram.get();
  multibody::MultibodyPlant<double>* rb_plant = stuff.controller_plant.get();
  ManipulationStation<double>* station = stuff.station;
  manipulation::robot_bridge::RobotBridge* robot_comm = stuff.robot_comm;

  systems::Simulator<double> simulator(*diagram);
  auto& station_context = diagram->GetMutableSubsystemContext(
      *station, &simulator.get_mutable_context());
  auto& rb_context = diagram->GetMutableSubsystemContext(
      *robot_comm, &simulator.get_mutable_context());

  // Nominal WSG position is open.
  station->GetInputPort("wsg_position").FixValue(&station_context, 0.1);
  // Force limit at 40N.
  station->GetInputPort("wsg_force_limit").FixValue(&station_context, 40.0);

  // Save initial configuration and tool pose.
  double t0 = simulator.get_context().get_time();
  Eigen::VectorXd q0 = station->GetIiwaPosition(station_context);

  auto temp_context = rb_plant->CreateDefaultContext();
  math::RigidTransform<double> X_WT0 =
      ComputeX_WT(*rb_plant, q0, temp_context.get());

  // I am manually controlling the simulator stepping to mimic the anzu
  // workflow for programming behaviors, e.g.:
  //    robot.MoveQ(q0);
  //    robot.MoveTool(X1);
  //    ...

  // Initialize command sources.
  double settling_time = 1;
  double t1 = settling_time - FLAGS_record_period;
  const trajectories::PiecewisePolynomial<double> q_traj0 =
      trajectories::PiecewisePolynomial<double>::FirstOrderHold({t0, t1},
                                                                {q0, q0});
  robot_comm->GetInputPort("q_trajectory")
      .FixValue(&rb_context,
                Value<trajectories::PiecewisePolynomial<double>>(q_traj0));

  // The time for the tool traj is kind of screwy.. I set it this way to avoid
  // move tool and move j being both active (which is determined by checking
  // context time against traj time.)
  manipulation::PiecewiseCartesianTrajectory<double> tool_traj0 =
      manipulation::PiecewiseCartesianTrajectory<double>::
          MakeCubicLinearWithEndLinearVelocity(
              {t0 - 1, t0 - 0.9},
              {X_WT0.GetAsIsometry3(), X_WT0.GetAsIsometry3()},
              Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  robot_comm->GetInputPort("tool_trajectory")
      .FixValue(&rb_context,
                Value<manipulation::PiecewiseCartesianTrajectory<double>>(
                    tool_traj0));

  // Randomize object initial condition. robot_comm->Initialize() needs to
  // happen after this call.
  std::random_device rd;
  RandomGenerator generator{rd()};
  // Search DUY in manipulation_station.cc for more details.
  diagram->SetRandomContext(&simulator.get_mutable_context(), &generator);

  // Note q0 is set in manipulation station, but you can override it here
  // again.
  q0 = station->GetIiwaPosition(station_context);

  // Initialize controller. Important, this needs to happen after
  // diagram->SetRandomContext, otherwise it will reset robot_comm's internal
  // state to the default model values (which are totally useless), and the x0
  // values that we just initialized will be wipedout.
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(rb_plant->num_multibody_states());
  x0.head(rb_plant->num_positions()) = q0;
  robot_comm->Initialize(x0, &rb_context);

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  // Exec first move j.
  simulator.AdvanceTo(t1);

  // Get object pose
  const auto& plant = stuff.station->get_multibody_plant();
  const auto& obj = plant.GetBodyByName(FLAGS_model_name);
  const auto& plant_context =
      diagram->GetSubsystemContext(plant, simulator.get_context());
  auto X_WObj = plant.CalcRelativeTransform(plant_context, plant.world_frame(),
                                            obj.body_frame());

  drake::log()->info("{}", X_WObj.matrix());
  auto target = ComputeX_WT_desired(X_WObj);

  auto& v_cmd_context = diagram->GetMutableSubsystemContext(
      *stuff.tool_vel_cmd, &simulator.get_mutable_context());
  while (true) {
    t0 = simulator.get_context().get_time();
    t1 = t0 + FLAGS_record_period;

    q0 = station->GetIiwaPosition(station_context);
    X_WT0 = ComputeX_WT(*rb_plant, q0, temp_context.get());

    Eigen::VectorXd V_cmd;
    if (!FLAGS_test) {
      V_cmd = ComputeControl(X_WT0, target);
    } else {
      V_cmd = WaitForRemoteControl(t0, X_WT0);
    }
    stuff.tool_vel_cmd->get_mutable_source_value(&v_cmd_context)
        .set_value(V_cmd);
    rb_context.FixInputPort(
        robot_comm->GetInputPort("tool_velocity_active").get_index(),
        Eigen::VectorXd::Constant(1, 1));

    simulator.AdvanceTo(t1);

    if (!FLAGS_test && t1 > 1 && V_cmd.norm() < 1e-2) {
      break;
    }
  }

  return 0;
}

}  // namespace
}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::manipulation_station::do_main(argc, argv);
}

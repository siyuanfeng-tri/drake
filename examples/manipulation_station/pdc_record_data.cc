#include <ctime>
#include <iomanip>
#include <sstream>
#include <experimental/filesystem>
#include <gflags/gflags.h>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/examples/manipulation_station/pdc_common.h"
#include "drake/examples/manipulation_station/pdc_ik.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(duration, 1e33, "Simulation duration.");
DEFINE_bool(test, false, "Disable random initial conditions in test mode.");
DEFINE_bool(viz_point_cloud, false, "Visualize point cloud?");

DEFINE_double(min_z, 0.3, "");
DEFINE_double(max_z, 0.6, "");

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

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  DataRecordParams record_params;
  record_params.record_period = FLAGS_record_period;
  record_params.record_root_dir = GetExperimentRootDir();
  record_params.record_start_time = FLAGS_record_start;

  MegaDiagramStuff stuff = build_pdc_mega_diagram(
      FLAGS_model_name, FLAGS_model_dir, FLAGS_viz_point_cloud, record_params,
      std::nullopt);
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
  math::RigidTransform<double> X_WT0;
  const auto& tool_frame = rb_plant->GetFrameByName("tool_frame");
  {
    auto temp_context = rb_plant->CreateDefaultContext();
    rb_plant->SetPositions(temp_context.get(), q0);
    X_WT0 = rb_plant->CalcRelativeTransform(
        *temp_context, rb_plant->world_frame(), tool_frame);
  }

  // I am manually controlling the simulator stepping to mimic the anzu
  // workflow for programming behaviors, e.g.:
  //    robot.MoveQ(q0);
  //    robot.MoveTool(X1);
  //    ...

  // Initialize command sources.
  double settling_time = 1;
  double t1 = settling_time;
  const trajectories::PiecewisePolynomial<double> q_traj0 =
      trajectories::PiecewisePolynomial<double>::FirstOrderHold({t0, t1},
                                                                {q0, q0});
  robot_comm->GetInputPort("q_trajectory")
      .FixValue(&rb_context,
                Value<trajectories::PiecewisePolynomial<double>>(q_traj0));

  // The time for the tool traj is kind of screwy.. I set it this way to avoid
  // move tool and move j being both active (which is determined by checking
  // context time against traj time.)
  /*
  const manipulation::SingleSegmentCartesianTrajectory<double> tool_traj0(
      X_WT0.GetAsIsometry3(), X_WT0.GetAsIsometry3(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), 0, 0, t0 - 1, t0 - 0.9);
  robot_comm->GetInputPort("tool_trajectory")
      .FixValue(&rb_context,
                Value<manipulation::SingleSegmentCartesianTrajectory<double>>(
                    tool_traj0));
  */
  const manipulation::PiecewiseCartesianTrajectory<double> tool_traj0 =
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

  // Plan scan traj.
  const auto& plant = station->get_multibody_plant();
  const auto& plant_context =
      diagram->GetSubsystemContext(plant, simulator.get_context());
  RigidTransform<double> X_WM = CalcRelativeTransform(
      plant, plant.GetPositionsAndVelocities(plant_context),
      plant.world_frame(), plant.GetFrameByName(FLAGS_model_name));

  const auto& camera_frame = plant.GetFrameByName("wrist_camera_frame");

  const double max_deg_tol = 10;
  GazeParams def_params = {Eigen::Vector3d::Zero(), max_deg_tol, FLAGS_min_z,
                           FLAGS_max_z};
  std::vector<GazeParams> params(2, def_params);
  /*
  params[0].waypoint_W = Eigen::Vector3d(0.6, -0.6, 0.8);
  params[0].min_dist -= 0.2;
  params[0].max_dist -= 0.2;

  params[1].waypoint_W = Eigen::Vector3d(0.6, 0.6, 0.8);
  */

  params[0].waypoint_W = Eigen::Vector3d(0., 0.6, 0.4);
  // params[0].min_dist -= 0.2;
  // params[0].max_dist -= 0.2;

  params[1].waypoint_W = Eigen::Vector3d(0., -0.6, 0.4);

  std::vector<Eigen::VectorXd> q_solutions =
      SolveScan(plant, camera_frame, X_WM.translation(), 0.2, params);
  drake::log()->info("qs size {}", q_solutions.size());
  if (q_solutions.empty()) {
    drake::log()->warn("Planning failed, rm {} and exiting.",
                       record_params.record_root_dir);
    std::experimental::filesystem::remove_all(record_params.record_root_dir);
    return -1;
  } else {
    auto q_low = rb_plant->GetPositionLowerLimits();
    auto q_high = rb_plant->GetPositionUpperLimits();
    bool low = true;
    for (auto& q : q_solutions) {
      if (low) {
        q[q.size() - 1] = std::max(q_low[q.size() - 1], -1.5);
      } else {
        q[q.size() - 1] = std::min(q_high[q.size() - 1], 1.5);
      }
      low = !low;
    }
    /*
     */
  }

  t0 = simulator.get_context().get_time();
  q0 = station->GetIiwaPosition(station_context);
  // Move to start of scan.
  trajectories::PiecewisePolynomial<double> q_traj =
      trajectories::PiecewisePolynomial<double>::FirstOrderHold(
          {t0, record_params.record_start_time - 0.5}, {q0, q_solutions.at(0)});
  robot_comm->GetInputPort("q_trajectory")
      .FixValue(&rb_context,
                Value<trajectories::PiecewisePolynomial<double>>(q_traj));
  simulator.AdvanceTo(record_params.record_start_time);

  // Scan and record.
  t0 = simulator.get_context().get_time();
  std::vector<Eigen::MatrixXd> qs;
  std::vector<double> ts;
  for (size_t i = 0; i < q_solutions.size(); i++) {
    qs.push_back(q_solutions[i]);
    t1 = t0 + 0.5 * i;
    ts.push_back(t1);
  }
  q_traj = trajectories::PiecewisePolynomial<double>::FirstOrderHold(ts, qs);
  robot_comm->GetInputPort("q_trajectory")
      .FixValue(&rb_context,
                Value<trajectories::PiecewisePolynomial<double>>(q_traj));
  simulator.AdvanceTo(t1);

  return 0;
}

}  // namespace
}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::manipulation_station::do_main(argc, argv);
}

#include <ctime>
#include <iomanip>
#include <sstream>
#include <gflags/gflags.h>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/examples/manipulation_station/pdc_common.h"
#include "drake/examples/manipulation_station/pdc_ik.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(duration, 1e33, "Simulation duration.");

DEFINE_double(min_z, 0.3, "");
DEFINE_double(max_z, 0.6, "");
DEFINE_string(model_name, "", "model_name");
DEFINE_string(model_dir, "", "Directory for models");

namespace drake {
namespace examples {
namespace manipulation_station {
namespace {

using math::RigidTransform;

struct PeelWaypoints {
  int64_t utime{};
  std::vector<std::string> names;
  std::vector<RigidTransform<double>> X_WP;
};

PeelWaypoints DecodeTransforms(const lcmt_viewer_draw& msg) {
  PeelWaypoints results;
  results.utime = msg.timestamp;
  for (int i = 0; i < msg.num_links; i++) {
    // w, x, y, z
    Eigen::Quaternion<double> quat(
        msg.quaternion.at(i)[0], msg.quaternion.at(i)[1],
        msg.quaternion.at(i)[2], msg.quaternion.at(i)[3]);
    Eigen::Vector3d pos(msg.position.at(i)[0], msg.position.at(i)[1],
                        msg.position.at(i)[2]);
    math::RotationMatrix<double> R(quat);
    results.names.push_back(msg.link_name.at(i));
    results.X_WP.push_back(RigidTransform<double>(R, pos));
  }
  return results;
}

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  PeelCommandParams cmd_params = {"PEEL_WAYPOINTS"};
  MegaDiagramStuff stuff = build_pdc_mega_diagram(
      FLAGS_model_name, FLAGS_model_dir, false, std::nullopt, cmd_params);
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
  const auto& rb_tool = rb_plant->GetFrameByName("tool_frame");
  {
    auto temp_context = rb_plant->CreateDefaultContext();
    rb_plant->SetPositions(temp_context.get(), q0);
    X_WT0 = rb_plant->CalcRelativeTransform(*temp_context,
                                            rb_plant->world_frame(), rb_tool);
  }

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

  const double max_deg_tol = 5;
  GazeParams def_params = {Eigen::Vector3d(0, 0, 0.5), max_deg_tol, FLAGS_min_z,
                           FLAGS_max_z};
  std::vector<GazeParams> params(2, def_params);

  std::vector<Eigen::VectorXd> q_solutions =
      SolveScan(plant, camera_frame, X_WM.translation(), 0.2, params);
  drake::log()->info("qs size {}", q_solutions.size());
  if (q_solutions.empty()) {
    drake::log()->warn("Planning failed.");
    return -1;
  }

  t0 = simulator.get_context().get_time();
  q0 = station->GetIiwaPosition(station_context);
  // Move to start of scan.
  trajectories::PiecewisePolynomial<double> q_traj =
      trajectories::PiecewisePolynomial<double>::FirstOrderHold(
          {t0, t0 + 1}, {q0, q_solutions.at(0)});
  robot_comm->GetInputPort("q_trajectory")
      .FixValue(&rb_context,
                Value<trajectories::PiecewisePolynomial<double>>(q_traj));
  t1 = t0 + 2;
  simulator.AdvanceTo(t1);

  // Pull commands from peel cmd subscriber.
  const systems::lcm::LcmSubscriberSystem& peel_cmd_sub =
      *stuff.peel_command_sub;
  const auto& peel_cmd_context =
      diagram->GetSubsystemContext(peel_cmd_sub, simulator.get_context());

  std::vector<PeelWaypoints> peel_cmds;
  while (true) {
    const auto& msg =
        peel_cmd_sub.get_output_port().Eval<lcmt_viewer_draw>(peel_cmd_context);
    PeelWaypoints mega_peel_cmd = DecodeTransforms(msg);
    if (mega_peel_cmd.X_WP.empty()) {
      t1 = simulator.get_context().get_time() + 1;
      drake::log()->info("Waiting for peel command, stepping to {}", t1);
      simulator.AdvanceTo(t1);
    } else {
      drake::log()->info("Got peel command {}", mega_peel_cmd.utime);
      PeelWaypoints strip;
      for (size_t i = 0; i < mega_peel_cmd.names.size(); i++) {
        const std::string name = mega_peel_cmd.names[i];
        drake::log()->info("{}", name);
        std::size_t pos = name.find("wp");
        if (name.substr(pos) == "wp_0") {
          if (i != 0) {
            peel_cmds.push_back(strip);
            drake::log()->info("Adding new strip");
          }
          strip = PeelWaypoints{};
        }
        strip.utime = mega_peel_cmd.utime;
        strip.names.push_back(name);
        strip.X_WP.push_back(mega_peel_cmd.X_WP[i]);

        // Last waypoint
        if (i == mega_peel_cmd.names.size() - 1) {
          peel_cmds.push_back(strip);
          drake::log()->info("Adding new strip");
        }
      }
      break;
    }
  }

  for (const PeelWaypoints& peel_cmd : peel_cmds) {
    std::vector<PeelParams> peel_params;
    for (size_t i = 0; i < peel_cmd.names.size(); i++) {
      const auto& X_WP = peel_cmd.X_WP[i];
      drake::log()->info("{} {}", peel_cmd.names[i], X_WP.GetAsMatrix4());
      peel_params.push_back(PeelParams{X_WP, 1e-3, 1});
    }

    const auto& tool_frame = plant.GetFrameByName("tool_frame");
    q_solutions = SolvePeel(plant, tool_frame, peel_params);
    drake::log()->info("qs size {}", q_solutions.size());
    if (q_solutions.empty()) {
      drake::log()->warn("Planning failed.");
      return -1;
    }

    t0 = simulator.get_context().get_time();
    q0 = station->GetIiwaPosition(station_context);
    std::vector<Eigen::MatrixXd> qs;
    std::vector<double> ts;
    qs.push_back(q0);
    t1 = t0 + 1;
    ts.push_back(t1);
    for (size_t i = 0; i < 1; i++) {
      qs.push_back(q_solutions[i]);
      t1 += 1.5;
      ts.push_back(t1);
    }
    q_traj = trajectories::PiecewisePolynomial<double>::
        CubicWithContinuousSecondDerivatives(ts, qs,
                                             Eigen::VectorXd::Zero(q0.size()),
                                             Eigen::VectorXd::Zero(q0.size()));
    robot_comm->GetInputPort("q_trajectory")
        .FixValue(&rb_context,
                  Value<trajectories::PiecewisePolynomial<double>>(q_traj));
    simulator.AdvanceTo(t1 + 0.5);

    getchar();

    std::vector<Eigen::Isometry3d> X_WPs;
    std::vector<double> times;
    t0 = simulator.get_context().get_time();

    q0 = station->GetIiwaPosition(station_context);
    {
      auto temp_context = rb_plant->CreateDefaultContext();
      rb_plant->SetPositions(temp_context.get(), q0);
      X_WT0 = rb_plant->CalcRelativeTransform(*temp_context,
                                              rb_plant->world_frame(), rb_tool);
      times.push_back(t0);
      X_WPs.push_back(X_WT0);
      t0 += 0.3;
    }

    for (size_t i = 0; i < peel_cmd.X_WP.size(); i++) {
      times.push_back(t0);
      X_WPs.push_back(peel_cmd.X_WP[i].GetAsIsometry3());
      t0 += 0.3;
    }

    const manipulation::PiecewiseCartesianTrajectory<double> tool_traj =
        manipulation::PiecewiseCartesianTrajectory<double>::
            MakeCubicLinearWithEndLinearVelocity(
                times, X_WPs, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    robot_comm->GetInputPort("tool_trajectory")
        .FixValue(&rb_context,
                  Value<manipulation::PiecewiseCartesianTrajectory<double>>(
                      tool_traj));
    simulator.AdvanceTo(times.back());
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

#include "drake/examples/manipulation_station/pdc_ik.h"

#include "drake/solvers/solve.h"

#include "drake/multibody/inverse_kinematics/angle_between_vectors_constraint.h"
#include "drake/multibody/inverse_kinematics/distance_constraint.h"
#include "drake/multibody/inverse_kinematics/gaze_target_constraint.h"
#include "drake/multibody/inverse_kinematics/minimum_distance_constraint.h"
#include "drake/multibody/inverse_kinematics/orientation_constraint.h"
#include "drake/multibody/inverse_kinematics/position_constraint.h"

namespace drake {
namespace examples {
namespace manipulation_station {

std::vector<Eigen::VectorXd> SolveScan(
    const multibody::MultibodyPlant<double>& plant,
    const multibody::Frame<double>& camera_frame,
    const Eigen::Vector3d& gaze_point_W, double dist_per_q,
    const std::vector<GazeParams>& gaze_params) {
  const auto iiwa = plant.GetModelInstanceByName("iiwa");
  std::vector<Eigen::VectorXd> results;

  std::vector<solvers::Binding<solvers::Constraint>> constraints;
  solvers::MathematicalProgram prog;
  std::vector<solvers::VectorXDecisionVariable> qs;

  auto context = plant.CreateDefaultContext();

  for (int seg_i = 0; seg_i < static_cast<int>(gaze_params.size()) - 1;
       seg_i++) {
    const Eigen::Vector3d& scan_end_cam_pt_W =
        gaze_params.at(seg_i + 1).waypoint_W;
    const Eigen::Vector3d& scan_start_cam_pt_W =
        gaze_params.at(seg_i).waypoint_W;
    const Eigen::Vector3d scan_dir =
        (scan_end_cam_pt_W - scan_start_cam_pt_W).normalized();
    int num_scan_points = static_cast<int>(std::ceil(
        (scan_end_cam_pt_W - scan_start_cam_pt_W).norm() / dist_per_q));
    num_scan_points = std::max(1, num_scan_points);
    drake::log()->info("num_scan_points: {}", num_scan_points);
    for (int i = 0; i < num_scan_points; i++) {
      qs.push_back(prog.NewContinuousVariables(
          plant.num_positions(), "q" + std::to_string(qs.size())));

      Eigen::Vector3d cam_pt;
      if (num_scan_points == 1) {
        cam_pt = scan_start_cam_pt_W;
      } else {
        cam_pt = scan_start_cam_pt_W +
                 scan_dir * (static_cast<double>(i) /
                             static_cast<double>(num_scan_points - 1));
      }
      drake::log()->info("cam_pt: {}", cam_pt.transpose());

      const Eigen::Vector3d gaze_dir_W = (gaze_point_W - cam_pt).normalized();
      drake::log()->info("gaze_dir_W: {}", gaze_dir_W.transpose());

      // Joint limit.
      {
        constraints.push_back(prog.AddBoundingBoxConstraint(
            plant.GetPositionLowerLimits(), plant.GetPositionUpperLimits(),
            qs.back()));
      }

      // Gaze dist constraint.
      {
        auto constraint = std::make_shared<multibody::PositionConstraint>(
            &plant, camera_frame,
            Eigen::Vector3d(-10, -10, gaze_params.at(seg_i).min_dist),
            Eigen::Vector3d(10, 10, gaze_params.at(seg_i).max_dist),
            plant.world_frame(), gaze_point_W, context.get());
        constraints.push_back(prog.AddConstraint(constraint, qs.back()));
      }

      // Gaze dir constraint.
      {
        auto constraint = std::make_shared<multibody::GazeTargetConstraint>(
            &plant, camera_frame, Eigen::Vector3d::Zero(),
            Eigen::Vector3d::UnitZ(), plant.world_frame(), gaze_point_W,
            gaze_params.at(seg_i).max_cone_deg * M_PI / 180., context.get());
        constraints.push_back(prog.AddConstraint(constraint, qs.back()));
      }

      // Spread out
      {
        auto constraint =
            std::make_shared<multibody::AngleBetweenVectorsConstraint>(
                &plant, plant.world_frame(), gaze_dir_W, camera_frame,
                Eigen::Vector3d::UnitZ(), 0,
                gaze_params.at(seg_i).max_cone_deg * M_PI / 180.,
                context.get());
        constraints.push_back(prog.AddConstraint(constraint, qs.back()));
      }
    }
  }

  // Add q box constraints between consecutive qs.
  if (qs.size() > 1) {
    Eigen::VectorXd lb = Eigen::VectorXd::Constant(plant.num_positions(), -10);
    Eigen::VectorXd ub = Eigen::VectorXd::Constant(plant.num_positions(), 10);
    Eigen::VectorXd q_thresh =
        Eigen::VectorXd::Constant(plant.num_positions(iiwa), 15 * M_PI / 180);
    plant.SetPositionsInArray(iiwa, q_thresh, &ub);
    plant.SetPositionsInArray(iiwa, -q_thresh, &lb);

    for (int i = 0; i < static_cast<int>(qs.size()) - 1; i++) {
      constraints.push_back(
          prog.AddLinearConstraint(qs[i] - qs[i + 1], lb, ub));
    }
  }

  const auto result = Solve(prog);
  drake::log()->info("Solve: {} using solver {}", result.is_success(),
                     result.get_solver_id());
  if (result.is_success()) {
    for (const auto& q_var : qs) {
      const auto q_full_sol = result.GetSolution(q_var);
      const auto q_iiwa = plant.GetPositionsFromArray(iiwa, q_full_sol);
      results.push_back(q_iiwa);
    }
  }

  return results;
}

std::vector<Eigen::VectorXd> SolvePeel(
    const multibody::MultibodyPlant<double>& plant,
    const multibody::Frame<double>& tool_frame,
    const std::vector<PeelParams>& peel_params) {
  const auto iiwa = plant.GetModelInstanceByName("iiwa");
  std::vector<Eigen::VectorXd> results;

  std::vector<solvers::Binding<solvers::Constraint>> constraints;
  solvers::MathematicalProgram prog;
  std::vector<solvers::VectorXDecisionVariable> qs;

  auto context = plant.CreateDefaultContext();
  for (size_t i = 0; i < peel_params.size(); i++) {
    const auto& param = peel_params.at(i);
    qs.push_back(prog.NewContinuousVariables(plant.num_positions(),
                                             "q" + std::to_string(qs.size())));
    // Joint limit.
    {
      constraints.push_back(prog.AddBoundingBoxConstraint(
          plant.GetPositionLowerLimits(), plant.GetPositionUpperLimits(),
          qs.back()));
    }

    // Position.
    {
      auto constraint = std::make_shared<multibody::PositionConstraint>(
          &plant, plant.world_frame(),
          param.X_WT_desired.translation() -
              Eigen::Vector3d::Constant(param.pos_thresh),
          param.X_WT_desired.translation() +
              Eigen::Vector3d::Constant(param.pos_thresh),
          tool_frame, Eigen::Vector3d(), context.get());
      constraints.push_back(prog.AddConstraint(constraint, qs.back()));
    }

    // Rotation.
    {
      auto constraint = std::make_shared<multibody::OrientationConstraint>(
          &plant, plant.world_frame(), param.X_WT_desired.rotation(),
          tool_frame, math::RotationMatrix<double>(),
          param.deg_thresh * M_PI / 180., context.get());
      constraints.push_back(prog.AddConstraint(constraint, qs.back()));
    }
  }

  // Add q box constraints between consecutive qs.
  if (qs.size() > 1) {
    Eigen::VectorXd lb = Eigen::VectorXd::Constant(plant.num_positions(), -10);
    Eigen::VectorXd ub = Eigen::VectorXd::Constant(plant.num_positions(), 10);
    Eigen::VectorXd q_thresh =
        Eigen::VectorXd::Constant(plant.num_positions(iiwa), 15 * M_PI / 180);
    plant.SetPositionsInArray(iiwa, q_thresh, &ub);
    plant.SetPositionsInArray(iiwa, -q_thresh, &lb);

    for (int i = 0; i < static_cast<int>(qs.size()) - 1; i++) {
      constraints.push_back(
          prog.AddLinearConstraint(qs[i] - qs[i + 1], lb, ub));
    }
  }

  const auto result = Solve(prog);
  drake::log()->info("Solve: {} using solver {}", result.is_success(),
                     result.get_solver_id());
  if (result.is_success()) {
    for (const auto& q_var : qs) {
      const auto q_full_sol = result.GetSolution(q_var);
      const auto q_iiwa = plant.GetPositionsFromArray(iiwa, q_full_sol);
      results.push_back(q_iiwa);
    }
  }

  return results;
}

}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake

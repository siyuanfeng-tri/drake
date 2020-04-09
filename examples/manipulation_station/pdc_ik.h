#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"

namespace drake {
namespace examples {
namespace manipulation_station {

struct GazeParams {
  Eigen::Vector3d waypoint_W{Eigen::Vector3d::Zero()};
  double max_cone_deg{90};
  double min_dist{0};
  double max_dist{std::numeric_limits<double>::infinity()};
};

struct PeelParams {
  math::RigidTransform<double> X_WT_desired;
  double pos_thresh{1e-3};
  double deg_thresh{1};
};

/**
 * Solves `num_scan_points` configurations for `plant` that are looking at
 * 'gaze_point_W'. You can make a scanning trajectory that's always looking
 * at `gaze_point_W` out of the result directly with spline generators given
 * `num_scan_points` is dense enough.
 *
 * For each configuration q_i, we define:
 * v_W = gaze_point_W - camera_origin_W(q_i), and
 * v_desired_W = gaze_point_W - linear_interp(
 *      scan_start_cam_pt_W, scan_end_cam_pt_W, i, num_scan_points).
 * q_i needs to make sure v_W and v_desired_W's directions are close enough.
 */
std::vector<Eigen::VectorXd> SolveScan(
    const multibody::MultibodyPlant<double>& plant,
    const multibody::Frame<double>& camera_frame,
    const Eigen::Vector3d& gaze_point_W, double dist_per_q,
    const std::vector<GazeParams>& gazea_params);

std::vector<Eigen::VectorXd> SolvePeel(
    const multibody::MultibodyPlant<double>& plant,
    const multibody::Frame<double>& tool_frame,
    const std::vector<PeelParams>& peel_params);

}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake

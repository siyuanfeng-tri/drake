#include "drake/manipulation/util/cartesian_trajectory_translator.h"

#include "drake/util/lcmUtil.h"

namespace drake {
namespace manipulation {

void CartesianKeyframeTranslator::InitializeMessage(lcmt_cartesian_keyframe* msg) {
  msg->timestamp = 0;
  msg->body_name = "unknown";

  EncodePose(Isometry3<double>::Identity(), msg->pose);
  EncodeTwist(Vector6<double>::Zero(), msg->velocity);
  EncodeTwist(Vector6<double>::Zero(), msg->acceleration);

  for (int i = 0; i < 6; ++i) {
    msg->constraint_mode[i] = 0;
  }

  // Expressed in the world frame.
  msg->expressed_in_frame = 0;
}

void CartesianKeyframeTranslator::EncodeMessage(
    const std::string& name,
    double time,
    const Isometry3<double>& pose,
    const Eigen::Ref<const Vector6<double>>& velocity,
    const Eigen::Ref<const Vector6<double>>& acceleration,
    lcmt_cartesian_keyframe* msg) {
  msg->timestamp = static_cast<int64_t>(time * 1e6);
  msg->body_name = name;
  EncodePose(pose, msg->pose);
  EncodeTwist(velocity, msg->velocity);
  EncodeTwist(acceleration, msg->acceleration);
}

void CartesianTrajectoryTranslator::InitializeMessage(lcmt_cartesian_trajectory* msg) {
  msg->timestamp = 0;
  msg->num_knots = 0;
  msg->knots.clear();
  msg->interpolation_mode.clear();
}

void CartesianTrajectoryTranslator::EncodeMessage(
    const std::string& name,
    const std::vector<double>& times,
    const std::vector<Isometry3<double>>& poses,
    const std::vector<Vector6<double>>& velocities,
    const std::vector<Vector6<double>>& accelerations,
    lcmt_cartesian_trajectory* msg) {
  DRAKE_DEMAND(poses.size() == times.size());
  DRAKE_DEMAND(poses.size() == velocities.size());
  DRAKE_DEMAND(poses.size() == accelerations.size());

  msg->num_knots = static_cast<int>(times.size());
  msg->knots.resize(msg->num_knots);
  msg->interpolation_mode.resize(msg->num_knots);

  for (int i = 0; i < msg->num_knots; ++i) {
    CartesianKeyframeTranslator::EncodeMessage(
        name, times[i], poses[i], velocities[i],
        accelerations[i], &(msg->knots[i]));

    // TODO
    msg->interpolation_mode[i] = 0;
  }
}

void CartesianTrajectoryTranslator::EncodeMessage(
    const std::string& name,
    const std::vector<double>& times,
    const PiecewiseCartesianTrajectory<double>& traj,
    lcmt_cartesian_trajectory* msg) {
  msg->num_knots = static_cast<int>(times.size());
  msg->knots.resize(msg->num_knots);
  msg->interpolation_mode.resize(msg->num_knots);

  for (int i = 0; i < msg->num_knots; ++i) {
    const double t = times[i];
    CartesianKeyframeTranslator::EncodeMessage(
        name, t, traj.get_pose(t), traj.get_velocity(t),
        traj.get_acceleration(t), &(msg->knots[i]));

    // TODO
    msg->interpolation_mode[i] = 0;
  }
}

void CartesianTrajectoryTranslator::DecodeMessage(
    const lcmt_cartesian_trajectory& msg,
    std::string* name,
    PiecewiseCartesianTrajectory<double>* traj) {
  if (msg.num_knots < 2)
    return;

  DRAKE_DEMAND(msg.num_knots == static_cast<int>(msg.knots.size()));
  DRAKE_DEMAND(msg.num_knots == static_cast<int>(msg.interpolation_mode.size()));

  // TODO: need to pay attention to interp mode.
  std::vector<double> times(msg.num_knots);
  std::vector<Isometry3<double>> poses(msg.num_knots);

  for (int i = 0; i < msg.num_knots; ++i) {
    const lcmt_cartesian_keyframe& knot = msg.knots[i];
    poses[i] = DecodePose(knot.pose);
    times[i] = static_cast<double>(knot.timestamp) / 1e6;
  }

  *name = msg.knots.front().body_name;

  *traj = PiecewiseCartesianTrajectory<double>::MakeCubicLinearWithEndLinearVelocity(
      times, poses, Vector3<double>::Zero(), Vector3<double>::Zero());
}

}  // namespace manipulation
}  // namespace drake

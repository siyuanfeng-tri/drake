#include "drake/manipulation/util/dof_trajectory_translator.h"

#include "drake/util/drakeUtil.h"

namespace drake {
namespace manipulation {

void DofKeyframeTranslator::InitializeMessage(int size, lcmt_dof_keyframe* msg) {
  msg->timestamp = 0;
  msg->num_dof = size;
  msg->position.resize(msg->num_dof, 0);
  msg->velocity.resize(msg->num_dof, 0);
  msg->acceleration.resize(msg->num_dof, 0);
  msg->constraint_mode.resize(msg->num_dof, 0);
}

void DofKeyframeTranslator::EncodeMessage(double time,
    const Eigen::Ref<const VectorX<double>>& position,
    const Eigen::Ref<const VectorX<double>>& velocity,
    const Eigen::Ref<const VectorX<double>>& acceleration,
    lcmt_dof_keyframe* msg) {
  DRAKE_DEMAND(position.size() == velocity.size());
  DRAKE_DEMAND(position.size() == acceleration.size());
  DRAKE_DEMAND(position.size() == msg->num_dof);

  msg->timestamp = time * 1e6;
  eigenVectorToStdVector(position, msg->position);
  eigenVectorToStdVector(velocity, msg->velocity);
  eigenVectorToStdVector(acceleration, msg->acceleration);
}

void DofKeyframeTranslator::DecodeMessage(const lcmt_dof_keyframe& msg,
    double* time,
    Eigen::Ref<VectorX<double>> position,
    Eigen::Ref<VectorX<double>> velocity,
    Eigen::Ref<VectorX<double>> acceleration) {
  DRAKE_DEMAND(position.size() == velocity.size());
  DRAKE_DEMAND(position.size() == acceleration.size());
  DRAKE_DEMAND(position.size() == msg.num_dof);

  *time = msg.timestamp / 1e6;
  stdVectorToEigenVector(msg.position, position);
  stdVectorToEigenVector(msg.velocity, velocity);
  stdVectorToEigenVector(msg.acceleration, acceleration);
}

void DofTrajectoryTranslator::InitializeMessage(lcmt_dof_trajectory* msg) {
  msg->timestamp = 0;
  msg->num_knots = 0;
  msg->knots.clear();
  msg->interpolation_mode.clear();
}

void DofTrajectoryTranslator::EncodeMessage(
    const std::vector<double>& times,
    const std::vector<VectorX<double>>& positions,
    const std::vector<VectorX<double>>& velocities,
    const std::vector<VectorX<double>>& accelerations,
    lcmt_dof_trajectory* msg) {
  DRAKE_DEMAND(times.size() == positions.size());
  DRAKE_DEMAND(times.size() == velocities.size());
  DRAKE_DEMAND(times.size() == accelerations.size());

  msg->num_knots = static_cast<int>(times.size());
  msg->knots.resize(msg->num_knots);
  msg->interpolation_mode.resize(msg->num_knots);

  for (size_t i = 0; i < positions.size(); ++i) {
    DofKeyframeTranslator::InitializeMessage(positions[i].size(), &(msg->knots[i]));
    DofKeyframeTranslator::EncodeMessage(times[i], positions[i], velocities[i], accelerations[i], &(msg->knots[i]));
    // TODO
    msg->interpolation_mode[i] = 0;
  }
}

void DofTrajectoryTranslator::DecodeMessage(
    const lcmt_dof_trajectory& msg,
    PiecewiseCubicTrajectory<double>* traj) {
  if (msg.num_knots < 2)
    return;

  DRAKE_DEMAND(msg.num_knots == static_cast<int>(msg.knots.size()));

  std::vector<double> times(msg.num_knots);
  std::vector<MatrixX<double>> positions(msg.num_knots);
  std::vector<MatrixX<double>> velocities(msg.num_knots);
  std::vector<MatrixX<double>> accelerations(msg.num_knots);

  for (int i = 0; i < msg.num_knots; ++i) {
    const auto& knot = msg.knots[i];
    positions[i].resize(knot.num_dof, 1);
    velocities[i].resize(knot.num_dof, 1);
    accelerations[i].resize(knot.num_dof, 1);

    DofKeyframeTranslator::DecodeMessage(knot, &times[i],
        positions[i].col(0), velocities[i].col(0),
        accelerations[i].col(0));
  }

  // TODO: Need to pay attention to interp mode.
  MatrixX<double> zero(msg.knots.front().num_dof, 1);
  *traj = PiecewiseCubicTrajectory<double>(
      PiecewisePolynomial<double>::Cubic(
          times, positions, zero, zero));
}

}  // namespace manipulation
}  // namespace drake

#include "dubins_interface.h"
#include <limits>
int num_samples = 1;
int ind_data = 0;
int StoreData(double q[3], double x, void* user_data) {
  if (x == 0) ind_data = 0;  // Reset index counter when distance is equal to 0

  ((double*)user_data)[4 * ind_data + 0] = q[0];  // Write to the output matrix
  ((double*)user_data)[4 * ind_data + 1] = q[1];
  ((double*)user_data)[4 * ind_data + 2] = q[2];
  ((double*)user_data)[4 * ind_data + 3] = x;  // Save distance along the path

  if (++ind_data >= num_samples)  // Prevent buffer overflow
    return 1;                     // Stop sampling
  return 0;
}

DubinsPushPlanner::DubinsPushPlanner(Eigen::Vector2d contact_point,
                                     Eigen::Vector2d contact_normal, double mu,
                                     double ls_a, double ls_b) {
  contact_point_ = contact_point;
  // In case the vector input is not normalized.
  contact_normal_ = contact_normal.normalized();

  ls_a_ = ls_a;
  ls_b_ = ls_b;
  unit_length_ = ls_a_ / ls_b_;
  mu_ = mu;

  // Compute all relevant parameters through the reduction logic.
  DubinsReduction();
}

DubinsPushPlanner::~DubinsPushPlanner() {}

void DubinsPushPlanner::DubinsPushPlanner::DubinsReduction() {
  ComputeFrictionCone();
  ComputeCenterOfRearAxleAndTurningRadius();
  ComputeDubinsFrame();
  ComputePusherFrame();
}

void DubinsPushPlanner::ComputeCenterOfRearAxleAndTurningRadius() {
  // Compute distances of the line of forces to the COM.
  double dist_fl_to_com = std::abs(contact_point_(1) * friction_cone_left_(0) -
                                   contact_point_(0) * friction_cone_left_(1));
  double dist_fr_to_com = std::abs(contact_point_(1) * friction_cone_right_(0) -
                                   contact_point_(0) * friction_cone_right_(1));

  // The distance of the line of CORs to COM is inverse proportion to the
  // distance from the contact point to COM.
  double dist_line_to_com =
      unit_length_ /
      (contact_point_.norm() + std::numeric_limits<double>::epsilon());

  // Compute the horizontal offset of the two CORs (dual points of the edges
  // of the friction cone) along the line of CORs.
  double cor_fl_dx =
      sqrt((unit_length_ / dist_fl_to_com) * (unit_length_ / dist_fl_to_com) -
           dist_line_to_com * dist_line_to_com);

  double cor_fr_dx = -1.0 * sqrt((unit_length_ / dist_fr_to_com) *
                                     (unit_length_ / dist_fr_to_com) -
                                 dist_line_to_com * dist_line_to_com);

  center_rear_axle_(0) = (cor_fl_dx + cor_fr_dx) / 2.0;
  center_rear_axle_(1) = dist_line_to_com;

  r_turn_ = (cor_fl_dx - cor_fr_dx) / 2.0;
}

void DubinsPushPlanner::ComputeDubinsFrame() {
  // The dubins push frame (rc_x, rc_y, theta): origin -> center of rear axle
  // +y axis -> the vector pointing from the contact point to COM.
  double angle_frame = atan2(contact_point_(0), -contact_point_(1));
  Eigen::Rotation2D<double> rotmat(angle_frame);
  tf_dubins_frame_.linear() = rotmat.toRotationMatrix();
  tf_dubins_frame_.translation() = rotmat * center_rear_axle_;
}

void DubinsPushPlanner::ComputePusherFrame() {
  // The pusher frame's y axis is aligned with the inward contact normal.
  double angle_frame = atan2(-contact_normal_(0), contact_normal_(1));
  tf_pusher_frame_.linear() =
      Eigen::Rotation2D<double>(angle_frame).toRotationMatrix();
  tf_pusher_frame_.translation() = contact_point_;
}

void DubinsPushPlanner::ComputeFrictionCone() {
  double fc_half_angle = atan2(mu_, 1);
  Eigen::Rotation2D<double> rotmat_l(fc_half_angle);
  Eigen::Rotation2D<double> rotmat_r(-fc_half_angle);
  friction_cone_left_ = rotmat_l * contact_normal_;
  friction_cone_right_ = rotmat_r * contact_normal_;
}

void DubinsPushPlanner::FlatSpaceToCartesianSpace(
    const Eigen::Matrix<double, Eigen::Dynamic, 2> flat_z,
    const Eigen::Matrix<double, Eigen::Dynamic, 2> flat_vz,
    Eigen::Matrix<double, Eigen::Dynamic, 3>* cartesian_poses) {
  int num_points = flat_z.rows();
  cartesian_poses->resize(num_points, 3);

  cartesian_poses->col(0) = flat_z.col(0);
  cartesian_poses->col(1) = flat_z.col(1);

  for (int i = 0; i < num_points; ++i) {
    (*cartesian_poses)(i, 2) = atan2(-flat_vz(i, 0), flat_vz(i, 1));
  }
}

void DubinsPushPlanner::CartesianSpaceToFlatSpace(
    const Eigen::Vector3d cartesian_pose, Eigen::Vector3d* flat_augmented_z) {
  (*flat_augmented_z)(0) = cartesian_pose(0);
  (*flat_augmented_z)(1) = cartesian_pose(1);
  (*flat_augmented_z)(2) =
      fmod(cartesian_pose(2) + M_PI / 2.0 + 2 * M_PI, 2 * M_PI);
}

void DubinsPushPlanner::GetDubinsPathFlatOutput(
    const Eigen::Vector3d flat_augmented_z_start,
    const Eigen::Vector3d flat_augmented_z_goal, const int num_way_points,
    Eigen::Matrix<double, Eigen::Dynamic, 2>* flat_traj_z) {
  DubinsPath path;
  // Call third party dubins curve generator. Return a path structure.
  double start[3];
  double goal[3];
  for (int i = 0; i < 3; ++i) {
    start[i] = flat_augmented_z_start(i);
    goal[i] = flat_augmented_z_goal(i);
  }
  dubins_init(start, goal, r_turn_, &path);
  // Get path length to determine the sampling segment length.
  double path_length = dubins_path_length(&path);
  int num_segments = num_way_points - 1;
  double step_size = path_length / num_segments;
  // Create output array.
  double* output_z = new double[4 * num_segments];
  // Set global variable number of samples.
  num_samples = num_segments;
  dubins_path_sample_many(&path, StoreData, step_size, output_z);

  // Extract from output_z.
  flat_traj_z->resize(num_way_points, 2);
  for (int i = 0; i < num_segments; ++i) {
    (*flat_traj_z)(i, 0) = *(output_z + 4 * i);
    (*flat_traj_z)(i, 1) = *(output_z + 4 * i + 1);
  }
  delete[] output_z;

  // Append flat_z_goal.
  (*flat_traj_z)(num_segments, 0) = flat_augmented_z_goal(0);
  (*flat_traj_z)(num_segments, 1) = flat_augmented_z_goal(1);
}

void DubinsPushPlanner::GetCartesianPathGivenFlatOutputPath(
    const Eigen::Matrix<double, Eigen::Dynamic, 2> flat_traj_z,
    Eigen::Matrix<double, Eigen::Dynamic, 3>* cartesian_traj) {
  const int num_points = flat_traj_z.rows();
  // Assume the entire trajectory takes 1 second. The exact number does not
  // matter since it's quasi-static.
  double dt = 1.0 / num_points;
  // Eigen::Matrix<double, num_points-1, 2> flat_traj_vz;
  Eigen::MatrixXd flat_traj_vz(num_points - 1, 2);
  // Compute the velocities through first order finite difference.
  for (int i = 1; i < num_points; ++i) {
    flat_traj_vz.row(i - 1) =
        (flat_traj_z.row(i) - flat_traj_z.row(i - 1)) / dt;
  }

  cartesian_traj->resize(num_points - 1, 3);
  FlatSpaceToCartesianSpace(flat_traj_z.topRows(num_points - 1), flat_traj_vz,
                            cartesian_traj);
}

void DubinsPushPlanner::PlanPath(
    const Eigen::Vector3d cart_pose_start, const Eigen::Vector3d cart_pose_goal,
    int num_way_points, Eigen::Matrix<double, Eigen::Dynamic, 3>* object_poses,
    Eigen::Matrix<double, Eigen::Dynamic, 3>* pusher_poses) {
  // Compute the dubins frame pose vectors given the object frame pose vectors.
  Eigen::Vector3d cart_pose_start_dubins =
      GetDubinsPoseVectorGivenObjectPoseVector(cart_pose_start);
  Eigen::Vector3d cart_pose_goal_dubins =
      GetDubinsPoseVectorGivenObjectPoseVector(cart_pose_goal);
  // Get the corresponding augmented state in flat space.
  Eigen::Vector3d flat_z_start;
  Eigen::Vector3d flat_z_goal;
  CartesianSpaceToFlatSpace(cart_pose_start_dubins, &flat_z_start);
  CartesianSpaceToFlatSpace(cart_pose_goal_dubins, &flat_z_goal);

  Eigen::Matrix<double, Eigen::Dynamic, 2> flat_z_traj;
  // Generate Dubins curve in flat space.
  GetDubinsPathFlatOutput(flat_z_start, flat_z_goal, num_way_points,
                          &flat_z_traj);

  Eigen::Matrix<double, Eigen::Dynamic, 3> cartesian_dubins_traj;
  // Map the flat space trajectory back to cartesian space (dubins frame).
  GetCartesianPathGivenFlatOutputPath(flat_z_traj, &cartesian_dubins_traj);

  // Get the trajectory of object frame from the trajectory of the dubins frame.
  object_poses->resize(num_way_points + 1, 3);
  pusher_poses->resize(num_way_points + 1, 3);
  // Add the initial pose. 
  object_poses->row(0) = cart_pose_start;
  for (int i = 0; i < num_way_points - 1; ++i) {
    object_poses->row(i+1) =
        GetObjectPoseVectorGivenDubinsPoseVector(cartesian_dubins_traj.row(i));
  }
  // Add the final goal pose.
  object_poses->row(num_way_points) = cart_pose_goal;
  // Compute the pusher pose.
  for (int i = 0; i < num_way_points + 1; ++i) {
    Eigen::Isometry2d tf_pusher =
        ConvertPoseVectorToSE3(object_poses->row(i)) * tf_pusher_frame_;
    pusher_poses->row(i) = ConvertSE3ToPoseVector(tf_pusher);
  }
}

double DubinsPushPlanner::GetPlannedDubinsCurveLength(
    const Eigen::Vector3d cart_pose_start, 
    const Eigen::Vector3d cart_pose_goal) {

  // Compute the dubins frame pose vectors given the object frame pose vectors.
  Eigen::Vector3d cart_pose_start_dubins =
      GetDubinsPoseVectorGivenObjectPoseVector(cart_pose_start);
  Eigen::Vector3d cart_pose_goal_dubins =
      GetDubinsPoseVectorGivenObjectPoseVector(cart_pose_goal);
  // Get the corresponding augmented state in flat space.
  Eigen::Vector3d flat_z_start;
  Eigen::Vector3d flat_z_goal;
  CartesianSpaceToFlatSpace(cart_pose_start_dubins, &flat_z_start);
  CartesianSpaceToFlatSpace(cart_pose_goal_dubins, &flat_z_goal);
  
  double start[3];
  double goal[3];
  for (int i = 0; i < 3; ++i) {
    start[i] = flat_z_start(i);
    goal[i] = flat_z_goal(i);
  }
  DubinsPath path;
  dubins_init(start, goal, r_turn_, &path);
  double path_length = dubins_path_length(&path);
  return path_length;
}


Eigen::Vector3d DubinsPushPlanner::GetDubinsPoseVectorGivenObjectPoseVector(
    const Eigen::Vector3d pose_vector_object) {
  Eigen::Isometry2d tf_object = ConvertPoseVectorToSE3(pose_vector_object);
  Eigen::Isometry2d tf_dubins = tf_object * tf_dubins_frame_;
  Eigen::Vector3d pose_vector_dubins = ConvertSE3ToPoseVector(tf_dubins);
  return pose_vector_dubins;
}

Eigen::Vector3d DubinsPushPlanner::GetObjectPoseVectorGivenDubinsPoseVector(
    const Eigen::Vector3d pose_vector_dubins) {
  Eigen::Isometry2d tf_dubins = ConvertPoseVectorToSE3(pose_vector_dubins);
  Eigen::Isometry2d tf_object = tf_dubins * tf_dubins_frame_.inverse();
  Eigen::Vector3d pose_vector_object = ConvertSE3ToPoseVector(tf_object);
  return pose_vector_object;
}

Eigen::Vector3d DubinsPushPlanner::ConvertSE3ToPoseVector(
    const Eigen::Isometry2d tf) {
  Eigen::Vector3d pose_vector;
  pose_vector.head(2) = tf.translation();
  pose_vector(2) = atan2(tf(1, 0), tf(0, 0));
  return pose_vector;
}

Eigen::Isometry2d DubinsPushPlanner::ConvertPoseVectorToSE3(
    const Eigen::Vector3d pose_vector) {
  Eigen::Isometry2d tf;
  tf.translation() = pose_vector.head(2);
  tf.linear() = Eigen::Rotation2D<double>(pose_vector(2)).toRotationMatrix();
  return tf;
}

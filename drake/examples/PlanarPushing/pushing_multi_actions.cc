#include "pushing_multi_actions.h"
#include <cassert>
#include <random>

MultiPushActionsPlanner::MultiPushActionsPlanner(
	std::vector<Eigen::Vector2d> all_contact_points, 
	std::vector<Eigen::Vector2d> all_contact_normals, double mu, double ls_a, 
	double ls_b) {
  
  assert(all_contact_points.size() == all_contact_normals.size());
  ls_a_ = ls_a;
  ls_b_ = ls_b;
  mu_ = mu;
  num_actions_ = all_contact_points.size();
  for (int i = 0; i < num_actions_; i++) {
  	action_planners_.push_back(DubinsPushPlanner(all_contact_points[i], 
  		all_contact_normals[i], mu_, ls_a_, ls_b_));
  }	
  // Initialize goal pose as the origin.
  goal_pose_ = Eigen::Vector3d::Zero();
  // Set default workspace constraint.
  SetWorkSpaceBoxConstraint();
  // Set default action switching cost.
  SetActionSwitchCost();
  // Set default graph size.
  SetGraphSize();
}


void MultiPushActionsPlanner::SetWorkSpaceBoxConstraint(double min_x, 
	double max_x, double min_y, double max_y, double buffer) {
  workspace_min_x_ = min_x;
  workspace_max_x_ = max_x;
  workspace_min_y_ = min_y;
  workspace_max_y_ = max_y;
  buffer_ = buffer;
}	

void MultiPushActionsPlanner::SetActionSwitchCost(double cost_switch_action) {
	cost_switch_action_ = cost_switch_action;
}

void MultiPushActionsPlanner::SetGraphSize(int num_sample_nodes) {
  num_sample_nodes_ = num_sample_nodes;
  num_expanded_nodes_ = num_actions_ * num_sample_nodes_;
}

void MultiPushActionsPlanner::ConstructPlanningGraph() {
  SampleNodesInBoxWorkSpace();
  shortest_distances_.resize(num_expanded_nodes_);
  bool mark_shortest[num_expanded_nodes_];
  // Initialize the distances for each expanded node ([sampled_id, action_id]).
  for (int i = 0; i < num_expanded_nodes_; ++i) {
  	int sample_id = i / num_expanded_nodes_;
  	int action_id = i - num_actions_ * sample_id;
  	shortest_distances_[i] = action_planners_[action_id].GetPlannedDubinsCurveLength(
  		sampled_pose_nodes_.row(sample_id).transpose(), goal_pose_);
  	mark_shortest[i] = false;
  }
  // Dijkstra process.

}

void MultiPushActionsPlanner::SampleNodesInBoxWorkSpace() {
  sampled_pose_nodes_.resize(num_sample_nodes_, 3);
  unsigned seed = 100;
  std::default_random_engine generator (seed);
  std::uniform_real_distribution<double> distribution(0.0, 1.0);
  // Uniform sampling.
  for (int i = 0; i < num_sample_nodes_; ++i) {
  	sampled_pose_nodes_(i, 0) =  workspace_min_x_ + distribution(generator) * 
  		(workspace_max_x_ - workspace_min_x_);
  	sampled_pose_nodes_(i, 1) =  workspace_min_y_ + distribution(generator) * 
  		(workspace_max_y_ - workspace_min_y_);
  	sampled_pose_nodes_(i, 2) = 2 * M_PI * distribution(generator);
  }	
}





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
	double max_x, double min_y, double max_y) {
  workspace_min_x_ = min_x;
  workspace_max_x_ = max_x;
  workspace_min_y_ = min_y;
  workspace_max_y_ = max_y;
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
  nxt_index_.resize(num_expanded_nodes_);
  bool mark_shortest[num_expanded_nodes_];
  // Initialize the distances for each expanded node ([sampled_id, action_id]).
  for (int i = 0; i < num_expanded_nodes_; ++i) {
  	int sample_id = i / num_actions_;
  	int action_id = i - num_actions_ * sample_id;
  	//shortest_distances_[i] = action_planners_[action_id].GetPlannedDubinsCurveLength(
  	//	sampled_pose_nodes_.row(sample_id).transpose(), goal_pose_);
  	bool flag_feasible;
  	double curve_length;
  	CheckPathAndGetPlannedCurveLength(
  		sampled_pose_nodes_.row(sample_id).transpose(), goal_pose_, action_id, 
  		&flag_feasible, &curve_length);
  	shortest_distances_[i] = curve_length;

  	mark_shortest[i] = false;
  	nxt_index_[i] = -1; 
  }
  // Dijkstra process.
  for (int i = 0; i < num_expanded_nodes_; ++i) {
  	// Find the shortest node to the goal. 
  	double min_dist = 1e+9;
  	int index_min_dist_node = -1;
  	for (int j = 0; j < num_expanded_nodes_; ++j) {
			if (!mark_shortest[j]) {
				if (shortest_distances_[j] < min_dist) {
					min_dist = shortest_distances_[j];
					index_min_dist_node = j;
				}		
			}
		mark_shortest[index_min_dist_node] = true;
		int sample_id_min_dist_node = index_min_dist_node / num_actions_;
		int action_id_min_dist_node = 
				index_min_dist_node - num_actions_ * sample_id_min_dist_node;
		// Try to Update all other nodes using this node as an immediate next way 
		// point to the goal.
		for (int j = 0; j < num_expanded_nodes_; ++j) {
			if (!mark_shortest[j]) {
		  	int sample_id = j / num_actions_;
				int action_id = j - num_actions_ * sample_id;
				// Compute the distance to the hop point with action_id. 
				//double dist_hop = action_planners_[action_id].GetPlannedDubinsCurveLength(
	  		//		sampled_pose_nodes_.row(sample_id).transpose(), 
	  		//		sampled_pose_nodes_.row(sample_id_min_dist_node).transpose());
				bool flag_feasible;
		  	double dist_hop;
  			CheckPathAndGetPlannedCurveLength(
  					sampled_pose_nodes_.row(sample_id).transpose(), 
  					sampled_pose_nodes_.row(sample_id_min_dist_node).transpose(), 
  					action_id, &flag_feasible, &dist_hop);
  			if (flag_feasible) {
					// Add the switching cost (if any).
					if (action_id_min_dist_node != action_id) {
						dist_hop = dist_hop + cost_switch_action_;
					}
					dist_hop = dist_hop + min_dist;
					// Check to see if we should update the current node.
					if (dist_hop < shortest_distances_[j]) {
						shortest_distances_[j] = dist_hop;
						nxt_index_[j] = index_min_dist_node;
					}
				}
			 } 
			}
  	}  // Dijkstra update.
  } // Outer for loop dijkstra. 

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


void MultiPushActionsPlanner::Plan(const Eigen::Vector3d cart_pose_start, 
  	std::vector<int>* all_action_ids, 
	  std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3> >* object_poses,
	  std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3> >* pusher_poses) {
	// First, try direction connection to the goal with all actions.
	double dist = 1e+9;
	int best_direct_action_id = -1;
	for (int i = 0; i < num_actions_; ++i) {
		bool flag_feasible;
  	double curve_length;
  	int direct_action_id = i;

		CheckPathAndGetPlannedCurveLength(cart_pose_start, goal_pose_, 
		    direct_action_id, &flag_feasible, &curve_length);
		
		if (curve_length < dist) {
			best_direct_action_id = direct_action_id;
			dist = curve_length;
		}
	}
	// Next, try all nodes in the graph and find the best hopping node.
	int best_hopping_node_id = -1;
	for (int i = 0; i < num_expanded_nodes_; ++i) {
		if (shortest_distances_[i] < dist) {
	  	int sample_id = i / num_actions_;
			int action_id = i - num_actions_ * sample_id;
			double dist_edge;
			bool flag_feasible;
			CheckPathAndGetPlannedCurveLength(cart_pose_start, 
				sampled_pose_nodes_.row(sample_id).transpose(), 
				action_id, &flag_feasible, &dist_edge);
			if (dist_edge + shortest_distances_[i] < dist) {
				best_hopping_node_id = i;
				dist = dist_edge + shortest_distances_[i];
			}
		}
	} // for all nodes considered as hopping node. 
	
	// Construct solution paths.
	std::vector<Eigen::Vector3d> sol_poses;
	std::vector<int> sol_action_ids;
	sol_poses.push_back(cart_pose_start);

	if (best_hopping_node_id != -1) {
		// The solution is to hop through nodes on the graph to the goal.
		int cur_id = best_hopping_node_id;
		int sample_id = cur_id / num_actions_;
		int action_id = cur_id - sample_id * num_actions_;
		// The first action will be the same as the immediate hopping node.
		sol_action_ids.push_back(action_id);
		while (cur_id != -1) {
			int sample_id = cur_id / num_actions_;
			int action_id = cur_id - sample_id * num_actions_;
			sol_action_ids.push_back(action_id);
			sol_poses.push_back(sampled_pose_nodes_.row(sample_id).transpose());
			cur_id = nxt_index_[cur_id];
		} 
	} else {
		// Otherwise directly go to the goal with best single action.
		sol_action_ids.push_back(best_direct_action_id);
	}
	ConstructAllPathSegments(sol_poses, sol_action_ids, object_poses, 
													 pusher_poses); 
	all_action_ids->clear();
	for (unsigned i = 0; i < sol_action_ids.size(); ++i) {
		all_action_ids->push_back(sol_action_ids[i]);
	}
}

bool MultiPushActionsPlanner::CheckPathWorkSpaceConstraint(
		const Eigen::Matrix<double, Eigen::Dynamic, 3>& object_poses) {
	int num_poses = object_poses.rows();
	for (int i = 0; i < num_poses; ++i) {
		if (!(object_poses(i, 0) >= workspace_min_x_ && 
				 object_poses(i, 0) <= workspace_max_x_ &&
				 object_poses(i, 1) >= workspace_min_y_ && 
				 object_poses(i, 1) <= workspace_max_y_)) {
			return false;
		}
	}
	return true;
}

void MultiPushActionsPlanner::CheckPathAndGetPlannedCurveLength(
		const Eigen::Vector3d cart_pose_start, const Eigen::Vector3d cart_pose_goal, 
		int action_id, bool* flag_inside_workspace, double* curve_length, 
    int num_way_pts_to_check) {
  Eigen::Matrix<double, Eigen::Dynamic, 3> object_poses;
	Eigen::Matrix<double, Eigen::Dynamic, 3> pusher_poses;
	action_planners_[action_id].PlanPath(cart_pose_start, cart_pose_goal, 
		num_way_pts_to_check, &object_poses, &pusher_poses);
	bool flag_inside = CheckPathWorkSpaceConstraint(object_poses);
	*flag_inside_workspace = flag_inside;
	if (flag_inside) {
		*curve_length = action_planners_[action_id].GetPlannedDubinsCurveLength(
				cart_pose_start, cart_pose_goal);
	} else {
		*curve_length = 1e+9;
	}
}

void MultiPushActionsPlanner::ConstructAllPathSegments(
    std::vector<Eigen::Vector3d> poses, std::vector<int> action_ids, 
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3> >* object_poses,
	  std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3> >* pusher_poses,
	  int num_way_pts_per_seg) {

	assert(poses.size() == action_ids.size());
	int num_segments = action_ids.size();
	for (int i = 0; i < num_segments; ++i) {
	  Eigen::Matrix<double, Eigen::Dynamic, 3> object_poses_seg;
		Eigen::Matrix<double, Eigen::Dynamic, 3> pusher_poses_seg;
		Eigen::Vector3d cart_pose_start = poses[i];
		Eigen::Vector3d cart_pose_end;
		if (i == num_segments - 1) {
			cart_pose_end = goal_pose_;
		} else {
			cart_pose_end = poses[i + 1];
		}
		int action_id = action_ids[i];
		action_planners_[action_id].PlanPath(cart_pose_start, cart_pose_end, 
				num_way_pts_per_seg, &object_poses_seg, &pusher_poses_seg);
		
		object_poses->push_back(object_poses_seg);
		pusher_poses->push_back(pusher_poses_seg);
	}
}





#pragma once

#include "dubins_interface.h"

// This class implements a planning logic that finds the best sequence of push
// actions to push from a given initial pose to the origin pose (0,0,0).
// Note that proper coordinate transformation needs to be applied such that the
// goal is at the origin. 

// Input to the planner consists of several push actions, usually choices of 
// push points and normal (in local frame). Also the limit surface parameters
// and coefficient of friction between pusher and object. A workspace box 
// constraint also needs to be specified. 

// 1) The planner constructs a graph in the workspace with specified number of
// sampled pose nodes.
// 2) Shortest paths (with specified action switching cost) are generated 
// between any two node. To deal with switching cost, the graph splits each node
// into copies representing different actions to take at this node.
// 3) For a given query initial pose, the planner either chooses to go directly
// to the goal or go to an intermediate node on the graph and follow the saved
// path afterwards, depending on which option gives shorter total cost. 


class MultiPushActionsPlanner {
 public:
	// Initialize with given contact points, normals, coefficient of contact 
	// friction, limit surface paramters. 
	MultiPushActionsPlanner(std::vector<Eigen::Vector2d> all_contact_points,
						   std::vector<Eigen::Vector2d> all_contact_normals,
						   double mu, double ls_a, double ls_b);

	void SetWorkSpaceBoxConstraint(double min_x = -0.5, double max_x = 0.5, 
								  double min_y = -0.5, double max_y = 0.5,
								  double buffer = 0.0);	

	void SetGraphSize(int num_sample_nodes = 1000);

	void SetActionSwitchCost(double cost_switch_action = 0.1);

	void ConstructPlanningGraph();

	// Given query start pose, return the action ids (index of the action planer),
	// object poses and pusher poses along each segment of trajectories.
	void Plan(const Eigen::Vector3d cart_pose_start, 
		 std::vector<int>* action_id,
		 std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3> >* object_poses,
		 std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3> >* pusher_poses);

   

 private:
   // Sample nodes in (expanded) workspace constraints.
   void SampleNodesInBoxWorkSpace();

   std::vector<DubinsPushPlanner> action_planners_;
   Eigen::Vector3d goal_pose_;

   int num_actions_;
   double mu_;
   double ls_a_;
   double ls_b_;

   double workspace_min_x_;
   double workspace_max_x_;
   double workspace_min_y_;
   double workspace_max_y_;
   double buffer_;
   
   double cost_switch_action_;
   
   int num_sample_nodes_;
   // num_expanded_nodes equals num_sampled_nodes * num_actions_.
   int num_expanded_nodes_;
   
   Eigen::Matrix<double, Eigen::Dynamic, 3> sampled_pose_nodes_;
   // The next expanded node index along the shortest paths.
   std::vector<int>  nxt_index_; 
   // The shortest distance to the goal.
   std::vector<double> shortest_distances_;
};
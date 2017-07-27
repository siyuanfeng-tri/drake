#include "drake/examples/PlanarPushing/dubins_interface.h"
#include "drake/examples/PlanarPushing/pushing_multi_actions.h"
#include <iostream>
#include <fstream>
#include <ctime>

int main() {
  double height = 0.234;
  double width = 0.178;
  //double height = 0.05;
  //double width = 0.04;
  double rho = width / 4;

  // Limit surface A_11. If you are unsure, just set it to be 1. 
  double ls_a = 1;
  // Limit surface A_33 / rho^2, where rho is a characteristic length, similar
  // to the minimum bounding circle radius. 
  //double ls_b = 4500;
  double ls_b = ls_a / (rho * rho);
  // Coefficient of contact friction
  double mu = 0.25;
  
  // Eigen::Vector2d pt;
  // pt << 0, -0.02;
  // Eigen::Vector2d normal;
  // normal << 0, 1;
  // DubinsPushPlanner planner(pt, normal, mu, ls_a, ls_b);

  // Eigen::Vector3d start_pose;
  // start_pose << 0, 0, 0;
  // Eigen::Vector3d goal_pose;
  // goal_pose << 0, 0, M_PI / 2.0;
  // int num_way_points = 100;
  // Eigen::Matrix<double, Eigen::Dynamic, 3> object_poses;
  // Eigen::Matrix<double, Eigen::Dynamic, 3> pusher_poses;
  // planner.PlanPath(start_pose, goal_pose, num_way_points, &object_poses,
  //                  &pusher_poses);
  // for (int i = 0; i < num_way_points; ++i) {
  //   std::cout << object_poses.row(i) << std::endl;
  //   std::cout << pusher_poses.row(i) << std::endl;
  // }
  int num_actions = 4;
  Eigen::Matrix<double, Eigen::Dynamic, 2> ct_pts(num_actions, 2);
  Eigen::Matrix<double, Eigen::Dynamic, 2> normal_pts(num_actions, 2);
  ct_pts << 0, -height/2, 
            -width/2, 0,
            0, height/2,
            width/2, 0;
  normal_pts << 0, 1,
                1, 0,
                0, -1,
                -1, 0;
  std::vector<Eigen::Vector2d> all_contact_points;
  std::vector<Eigen::Vector2d> all_normals;
  for (int i = 0; i < ct_pts.rows(); ++i) {
    all_contact_points.push_back(ct_pts.row(i).transpose());
    all_normals.push_back(normal_pts.row(i).transpose());
  }
  MultiPushActionsPlanner multi_action_planner(all_contact_points, all_normals, 
    mu, ls_a, ls_b);
  double xmin, xmax, ymin, ymax;
  xmin = -0.2; xmax = 0.2; ymin = 0; ymax = 0.4;
  //xmin = -0.2; xmax = 0.2; ymin = -0.2; ymax = 0.2;
  multi_action_planner.SetWorkSpaceBoxConstraint(xmin, xmax, ymin, ymax);
  //int nd = 5;
  //int num_samples_se2 = nd * nd * nd;
  int num_samples_se2 = 100;
  double switching_action_cost = 0.05;
  multi_action_planner.SetGraphSize(num_samples_se2);
  multi_action_planner.SetActionSwitchCost(switching_action_cost);

  // Call construction of the graph
  clock_t begin_time = clock();
  multi_action_planner.ConstructPlanningGraph();
  clock_t end_time = clock();
  std::cout << "Elapsed time: " << 
    double(end_time - begin_time) / CLOCKS_PER_SEC << std::endl; 

  Eigen::Vector3d query_pose;
  query_pose << 0.0, 0.01, M_PI/2;
  std::vector<int> action_id;
  std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3> > all_object_poses;
  std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3> > all_pusher_poses;

  clock_t begin_time2 = clock();
  int num_way_pts_perseg = 100;
  multi_action_planner.Plan(query_pose, num_way_pts_perseg, &action_id, 
      &all_object_poses, &all_pusher_poses);
  clock_t end_time2 = clock();
  std::cout << "Elapsed time: " << 
    double(end_time2 - begin_time2) / CLOCKS_PER_SEC << std::endl;   
  for (unsigned i = 0; i < all_object_poses.size(); ++i) {
    std::cout << "Segment Traj " << i << std::endl;
    std::cout << "Action id " << action_id[i] << std::endl;
    for (unsigned j = 0; j < all_object_poses[i].rows(); ++j) {
      std::cout << all_object_poses[i].row(j) << std::endl;
    }
  }
  std::string file_output = "test_output.txt";
  std::ofstream out(file_output);
  multi_action_planner.Serialize(out);
  out.close();
  MultiPushActionsPlanner multi_action_planner_load_test(file_output);
  std::cout << "Load file test" << std::endl;
  multi_action_planner_load_test.Plan(query_pose, num_way_pts_perseg, &action_id, 
      &all_object_poses, &all_pusher_poses);
  
  std::string file_output2 = "test_output_loaded.txt";
  std::ofstream out2(file_output2);
  multi_action_planner_load_test.Serialize(out2);
  out2.close();
  
  //multi_action_planner_load_test.Deserialize(in);
  //in.close();
}

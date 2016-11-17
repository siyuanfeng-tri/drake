#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/multibody/parser_yaml.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace {

GTEST_TEST(RigidBodyTreeYAMLParsingTest, TestJointGroup) {
  std::string path = drake::GetDrakePath() + "/multibody/test/rigid_body_tree/";
  std::string urdf = path + "two_dof_robot.urdf";
  std::string config = path + "two_dof_robot.config";
  RigidBodyTree<double> robot(urdf, drake::multibody::joints::kFixed);
  //RigidBodyTree<double> robot(urdf, drake::multibody::joints::kQuaternion);

  for (int i = 0; i < robot.get_num_positions(); i++)
    std::cout << robot.get_position_name(i) << std::endl;
  for (int i = 0; i < robot.get_num_velocities(); i++)
    std::cout << robot.get_velocity_name(i) << std::endl;

  YAML::Node file = YAML::LoadFile(config);
  parsers::ParseJointGroups(file, &robot);
  parsers::ParseBodyGroups(file, &robot);

  EXPECT_TRUE(robot.has_position_group("j_group1"));
  EXPECT_TRUE(robot.has_position_group("j_group2"));
  EXPECT_FALSE(robot.has_position_group("j_group3"));

  EXPECT_TRUE(robot.has_velocity_group("j_group1"));
  EXPECT_TRUE(robot.has_velocity_group("j_group2"));
  EXPECT_FALSE(robot.has_velocity_group("j_group3"));

  EXPECT_TRUE(robot.has_body_group("b_group1"));
  EXPECT_TRUE(robot.has_body_group("b_group2"));
  EXPECT_TRUE(robot.has_body_group("b_group3"));
  EXPECT_FALSE(robot.has_body_group("b_group55"));

  EXPECT_EQ(robot.get_position_group("j_group1").size(), 0);
  EXPECT_EQ(robot.get_position_group("j_group2").size(), 8);
  EXPECT_EQ(robot.get_velocity_group("j_group1").size(), 0);
  EXPECT_EQ(robot.get_velocity_group("j_group2").size(), 7);

  EXPECT_EQ(robot.get_body_group("b_group1").size(), 0);
  EXPECT_EQ(robot.get_body_group("b_group2").size(), 2);
  EXPECT_EQ(robot.get_body_group("b_group2")[0]->get_name(), "link1");
  EXPECT_EQ(robot.get_body_group("b_group2")[1]->get_name(), "link3");
  EXPECT_EQ(robot.get_body_group("b_group3").size(), 1);
  EXPECT_EQ(robot.get_body_group("b_group3")[0]->get_name(), "world");
}

}  // namespace
}  // namespace drake

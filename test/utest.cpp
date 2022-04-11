#include <ros/ros.h>
#include <gtest/gtest.h>
#include "husky/husky.hpp"

TEST(TestSuite, husky_pathLength)
{
  ros::NodeHandle nh;
  Husky test(nh);

  //diagonal path
  poseArray path;
  path.reserve(5);
  for(int i = 0; i < 5; ++i)
  {
    geometry_msgs::Pose2D pose;
    pose.x = i;
    pose.y = i;
    path.push_back(pose);
  }
  double length = test.pathLength(path);
  ASSERT_EQ(length, 4*sqrt(2));
  //------------------------------------
  //vertical path
  poseArray path2;
  path2.reserve(4);
  for(int i = 0; i < 4; ++i)
  {
    geometry_msgs::Pose2D pose;
    pose.x = 0;
    pose.y = i;
    path2.push_back(pose);
  }
  double length2 = test.pathLength(path2);
  ASSERT_EQ(length2, 3);
  //------------------------------------
  //horizontal path
  poseArray path3;
  path3.reserve(6);
  for(int i = 0; i < 6; ++i)
  {
    geometry_msgs::Pose2D pose;
    pose.x = -i;
    pose.y = 0;
    path3.push_back(pose);
  }
  double length3 = test.pathLength(path3);
  ASSERT_EQ(length3, 5);
}

TEST(TestSuite, husky_calculateArea)
{
  float MAP_RESOLUTION = 0.1;   //[m]
  int MAP_WIDTH = 30;           //[cells]
  int MAP_HEIGHT = 120;         //[cells]

  ros::NodeHandle nh;
  Husky test(nh);
  nav_msgs::OccupancyGrid grid;

  grid.data.resize(MAP_HEIGHT * MAP_WIDTH, 50);
  for (int i = 0; i<100; ++i)
  {
    grid.data[i] = 0;
  }
  double area = test.calculateArea(grid);
  double expected_area = 100 * MAP_RESOLUTION * MAP_RESOLUTION;
  ASSERT_EQ(area, expected_area);

  //-------------------------------
  grid.data.clear();
  grid.data.resize(MAP_HEIGHT * MAP_WIDTH, 50);
  for (int i = 0; i<50; ++i)
  {
    grid.data[i] = 0;
  }
  double area2 = test.calculateArea(grid);
  double expected_area2 = 50 * MAP_RESOLUTION * MAP_RESOLUTION;
  ASSERT_EQ(area2, expected_area2);
}

TEST(TestSuite, husky_cellIDToLinearCell)
{
  int MAP_WIDTH = 30;           //[cells]
  int MAP_HEIGHT = 120;         //[cells]

  ros::NodeHandle nh;
  Husky test(nh);
  int cell1 = test.cellIDToLinearCell(5, 5);
  int expected_cell1 = 5+MAP_WIDTH/2 + (5+MAP_HEIGHT/2)*MAP_WIDTH;
  ASSERT_EQ(cell1, expected_cell1);

  //---------------------
  int cell2 = test.cellIDToLinearCell(0, 0);
  int expected_cell2 = 0+MAP_WIDTH/2 + (0+MAP_HEIGHT/2)*MAP_WIDTH;
  ASSERT_EQ(cell2, expected_cell2);
}

int main(int argc, char **argv)
{
  // testing::InitGoogleTest(&argc, argv);
  // return RUN_ALL_TESTS();

  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "husky_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();

  return ret;
}
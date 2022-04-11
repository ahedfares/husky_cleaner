#include <ros/ros.h>
#include "husky/husky.hpp"

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "husky");
  ros::NodeHandle nh;

  Husky example(nh);
  example.init();
  example.run();
  
  ros::spin();
  return 0;
}

cmake_minimum_required(VERSION 3.0.2)
project(husky)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

add_library(best-fit depends/bestfit/BestFit.cpp
                     depends/bestfit/Double.cpp
                     depends/bestfit/Shapes.cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs 
  tf2           
  tf2_ros
  nav_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES husky
 CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs tf2 tf2_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  depends
)

add_library(${PROJECT_NAME}lib SHARED src/husky.cpp)
target_link_libraries(${PROJECT_NAME}lib ${catkin_LIBRARIES} best-fit)

add_executable(${PROJECT_NAME}_node src/husky_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PROJECT_NAME}lib)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(utest_node test/utest_launch.test test/utest.cpp)
  target_link_libraries(utest_node ${catkin_LIBRARIES} ${PROJECT_NAME}lib)
endif()
# Readme
This package contains ROS nodes that utlizes **ROS Navigation** to visualize a robot based on data given in a json file, the package consists of the following:
1. huskylib: This library includes all the functions to calculate the required parameters in the given assignment (i.e. total path covered, total cleaned area, and total time required to traverse the given path).
2. husky_node: This is a ros node that utilizes the functions designed in the husky class(from huskylib) to initialize the publishers, map, and to run the code required to calculate the required parameters.

## Required framesworks and packages
1. *ROS noetic*
2. *best-fit* library contains functions required to calculate the least square approximation for discrete data points to calculate the approximate curvature of consecutive datapoints in a given path.

## Launch
To launch the task, please install the ROS noetic, unzip the submitted package, and build the given package in a catkin workspace using the following lines of code.
```
mkdir -p catkin_ws/src
cp -r husky catkin_ws/src
cd catkin_ws
catkin build
source devel/setup.bash
roslaunch husky run.launch
```

## Notes
1. The submitted package provides many approximations to calculate the required parameters as it is based on using occupancy grid to calculate the total covered area.
2. The parameters are hard coded for the purpose of this task, but all the declared static global variables (e.g. map resolution, grid width, grid height, ...etc) can be turned into parameters for reusability if needed.
3. Calculating the curvature from discrete points is done via using an online package (submitted with the files in the depends folder), the math behind it relies on least square approximation and a chosen value of 5 look ahead and behind points is chosen and found to be accurate enough for the purpose of the task.
4. To calculate the total path, the distance between two data points along the path is checked and the linear distance between them is added to the total distance.
5. To calculate the total cleaned area, an occupancy grid map is designed and the cells are marked as visited for each pose in the given path with consideration of orientation at each pose, then an interpolation is done between each two poses to cover the cells while the robot is moving between pose[i] to pose[i+1].
6. To calculate the total time, the curvature at each pose is calculated (approximated) using the best-fit library, next the speed is assigned based on the formula given in the assignment, in the end, the distance between the current pose and the next pose is calculate and the step_time is given via *distance/speed*. Then all the step_times along the path are summed.
7. The executable node runs RVIZ as main simulation agent. It publishes the path, robot shape, and cleaning agent shape for the purpose of visualiztion.
8. The activated cells are shown via map frame in RVIZ to visualize how the robot is covering the area while in motion.
9. static global variables in the library husky.cpp file are chosen as found best suitable for the given task, I have no idea of the memory or performance capabilities on the robot, thus, the variables could be modified if needed (the code has to be rebuilt as these parameters are not provided as rosparams).
10. The header file of husky lib contains main documentation for each of the designed functions, it should be a good reference to get a feel of what each function should do.
11. To run the unit tests please run the following code:
```
roslaunch husky run.launch utest_launch.test
```

#include <iostream>
#include <ros/ros.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include "bestfit/BestFit.h"

typedef std::vector<geometry_msgs::Pose2D> poseArray; 

class Husky
{
    private:
        ros::NodeHandle m_nh;
        std::shared_ptr<tf2_ros::TransformBroadcaster> m_br;
        ros::Publisher m_path_publisher;
        ros::Publisher m_robot_publisher;
        ros::Publisher m_gadget_publisher;
        ros::Publisher m_map_publisher;
        nav_msgs::OccupancyGrid m_grid;
        geometry_msgs::Polygon m_robot_corners;
        geometry_msgs::Polygon m_gadget_corners;

    public:
        /**
         * @brief Constructors
         */ 
        Husky(ros::NodeHandle& nh);

        /**
         * @brief Initializes publishers and map.
         */ 
        bool init();

        /**
         * @brief Calculates the total time required for the robot to cover a given path.
         */ 
        bool run();

        /**
         * @brief Calculates the length of a given path(set of poses).
         * @param[in] path A vector of geometry_msgs::Pose2D containing the x and y coordinates of each pose.
         * @return Total path length in meters.
         */ 
        double pathLength(poseArray& path);

                /**
         * @brief Calculate the total cleaned area based on the number of the activated cells in the occupancy grid.
         * @param[in] grid An occupancy grid which has the activated cells.
         * @return Cleaned area [squared meters].
         */ 
        double calculateArea(nav_msgs::OccupancyGrid& grid);

        /**
         * @brief Calculates the total time required for the robot to cover a given path.
         * @param[in] path A vector of geometry_msgs::Pose2D containing the x and y coordinates of each pose on the path.
         * @param[in] curvatures A vector containing all the curvature values at each pose.
         * @return The total time required for the robot to cover a given path. 
         */ 
        double calculateTotalTime(poseArray& path, std::vector<double>& curvatures);

        /**
         * @brief Converts a given cellID [x,y] to a linear CellID(1-dimension).
         * @param[in] x X cell ID in the grid.
         * @param[in] y Y cell ID in the grid.
         * @return The linear cellID.
         */ 
        int cellIDToLinearCell(int x, int y);

    private:
        /**
         * @brief obtains polygon corners from a json file.
         * @param[in] param Parameter to be searched inside the json file.
         * @return A geometry_msgs::Polygon containing corner points of the robot.
         */ 
        geometry_msgs::Polygon obtainPolygon(const std::string& param);

        /**
         * @brief obtains poses from a json file.
         * @param[in] param Parameter to be searched inside the json file.
         * @return A vector of geometry_msgs::Pose2D containing the x and y coordinates of each pose.
         */ 
        std::vector<geometry_msgs::Pose2D> obtainPose(const std::string& param);

        /**
         * @brief Calculates the cell IDs which relate to the cleaning gadget geometry for a given pose.
         * @param[in] pose A geometry_msgs::Pose2D containing the x and y coordinates the robot pose.
         * @return A list of cell IDs in the occupancy grid which relates to the cleaning_gadged geometry. 
         */ 
        std::vector<int> obtainGridCells(geometry_msgs::Pose2D& pose);

        /**
         * @brief Publishes the path of the given set of 2D poses as a ROS ropic(used by RVIZ for visualization).
         * @param[in] poses A vector of geometry_msgs::Pose2D containing the x and y coordinates of each pose.
         */
        void publishPath(poseArray& poses);

        /**
         * @brief Publishes the current pose of the robot as a TF.
         * @param[in] pose A geometry_msgs::Pose2D containing the x and y coordinates the robot pose.
         */ 
        void publishRobotPose(const geometry_msgs::Pose2D& pose);

        /**
         * @brief Calculates the cell ID to be updated in an occupancy grid from a given pose.
         * @param[in] pose A geometry_msgs::Pose2D containing the x and y coordinates the pose.
         * @return The ID number of the cell which relates to the given pose.
         */ 
        int poseToCellID(geometry_msgs::Pose2D& pose);
        
        /**
         * @brief A simplified implementation of Bresenham's algorithm to interpolate the cells between two given poses.
         * @return A vecetor of all the cells' IDs between the given 2 poses.
         */ 
        std::vector<int> interpolateCells(int x0, int y0, int x1, int y1);

        /**
         * @brief Creates an occupancy grid map object that is used to visualize the robot motion.
         */ 
        void createMap();

        /**
         * @brief Updates the occupancy grid map based on the given cells' IDs.
         * @param[in] cells A vecetor of the cells' IDs to be activated.
         */ 
        void updateMap(const std::vector<int>& cells);

        /**
         * @brief Calculates the curvature of a path.
         * @param[in] path A vector of geometry_msgs::Pose2D containing the x and y coordinates of each pose on the path.
         * @return A vector containing all the curvature values at each given pose.
         */ 
        std::vector<double> calculateCurvature(poseArray& path);

};



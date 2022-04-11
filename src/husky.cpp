#include "husky/husky.hpp"

static float MAP_RESOLUTION = 0.1;   //[m]
static int MAP_WIDTH = 30;           //[cells]
static int MAP_HEIGHT = 120;         //[cells]
static double V_MAX = 1.1;           //[m/s]
static double V_MIN = 0.15;          //[m/s]
static double K_MAX = 10;            //[m]
static double K_CRITICAL = 0.5;      //[m]
static int CURVATURE_RESOLUTION = 5; //number of poses to consider when calculating the curvature

Husky::Husky(ros::NodeHandle& nh) 
  : m_nh(nh)
{
}

geometry_msgs::Polygon Husky::obtainPolygon(const std::string& param)
{
  XmlRpc::XmlRpcValue xml;
  m_nh.getParam(param, xml);
  ROS_ASSERT(xml.getType() == XmlRpc::XmlRpcValue::TypeArray);

  geometry_msgs::Polygon corners;
  corners.points.reserve(xml.size());

  for (int i = 0; i < xml.size(); ++i) 
  {
    ROS_ASSERT(xml[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
    geometry_msgs::Point corner;
    corner.x = xml[i][0];
    corner.y = xml[i][1];
    corner.z = 0;

    geometry_msgs::Point32 corner32;
    corner32.x = corner.x;
    corner32.y = corner.y;
    corner32.z = corner.z;
    corners.points.emplace_back(corner32);
  }
  return corners;
}

std::vector<geometry_msgs::Pose2D> Husky::obtainPose(const std::string& param)
{
  XmlRpc::XmlRpcValue xml;
  m_nh.getParam(param, xml);
  ROS_ASSERT(xml.getType() == XmlRpc::XmlRpcValue::TypeArray);

  std::vector<geometry_msgs::Pose2D> poses;
  poses.reserve(xml.size());

  for (int i = 0; i < xml.size(); ++i) 
  {
    ROS_ASSERT(xml[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
    geometry_msgs::Pose2D pose;
    pose.x = xml[i][0];
    pose.y = xml[i][1];

    if(i==0)
    {
      pose.theta = 0;
    }
    else
    {
      double dx = pose.x - poses[i-1].x;
      double dy = pose.y - poses[i-1].y;
      pose.theta = atan2(dy, dx) * (180 / M_PI);  //theta in degrees
      // ROS_WARN("pose[i].x: %f, pose[i-1].x: %f", pose.x, poses[i-1].x);
      // ROS_WARN("pose[i].y: %f, pose[i-1].y: %f", pose.y, poses[i-1].y);
      // ROS_WARN("dx: %f, dy: %f", dx,dy);
      // ROS_WARN("theta: %f", pose.theta);
      // ROS_WARN("----------");
    }
    poses.emplace_back(pose);
  }
  return poses;
}

std::vector<int> Husky::obtainGridCells(geometry_msgs::Pose2D& pose)
{
  std::vector<int> cells(6, 50);

  //convert x, y coordinates to grid cell ID
  int grid_x = pose.x/MAP_RESOLUTION;
  if(pose.x<0)
  {
    --grid_x;
  }
  int grid_y = pose.y/MAP_RESOLUTION;
  if(pose.y<0)
  {
    --grid_y;
  }

  int cell0, cell1, cell2, cell3, cell4, cell5;
  int grid_x0, grid_x1, grid_x2, grid_x3, grid_x4, grid_x5;
  int grid_y0, grid_y1, grid_y2, grid_y3, grid_y4, grid_y5;

  if((pose.theta >= 0 && pose.theta < 22.5) || (pose.theta < 0 && pose.theta >= -22.5))
  {
    grid_x0 = grid_x;
    grid_x1 = grid_x;
    grid_x2 = grid_x;
    grid_x3 = grid_x;
    grid_x4 = grid_x;
    grid_x5 = grid_x;

    grid_y0 = grid_y;
    grid_y1 = grid_y - 1;
    grid_y2 = grid_y - 2;
    grid_y3 = grid_y - 3;
    grid_y4 = grid_y + 1;
    grid_y5 = grid_y + 2;

  }else if((pose.theta >=22.5 && pose.theta < 67.5))
  {
    grid_x0 = grid_x; 
    grid_y0 = grid_y;

    grid_x1 = grid_x + 1; 
    grid_y1 = grid_y - 1;

    grid_x2 = grid_x + 2; 
    grid_y2 = grid_y - 2;

    grid_x3 = grid_x + 3; 
    grid_y3 = grid_y - 3;

    grid_x4 = grid_x - 1; 
    grid_y4 = grid_y + 1;

    grid_x5 = grid_x - 2; 
    grid_y5 = grid_y + 2;
    
  }else if(pose.theta > 67.5 && pose.theta <= 112.5)
  {
    grid_y0 = grid_y;
    grid_y1 = grid_y;
    grid_y2 = grid_y;
    grid_y3 = grid_y;
    grid_y4 = grid_y;
    grid_y5 = grid_y;

    grid_x0 = grid_x;
    grid_x1 = grid_x + 1;
    grid_x2 = grid_x + 2;
    grid_x3 = grid_x + 3;
    grid_x4 = grid_x - 1;
    grid_x5 = grid_x - 2;

  }else if(pose.theta >112.5 && pose.theta <= 157.5)
  {
    grid_x0 = grid_x; 
    grid_y0 = grid_y;

    grid_x1 = grid_x + 1; 
    grid_y1 = grid_y + 1;

    grid_x2 = grid_x + 2; 
    grid_y2 = grid_y + 2;

    grid_x3 = grid_x + 3; 
    grid_y3 = grid_y + 3;

    grid_x4 = grid_x - 1; 
    grid_y4 = grid_y - 1;

    grid_x5 = grid_x - 2; 
    grid_y5 = grid_y - 2;

  }else if((pose.theta > 157.5 && pose.theta <= 180) || (pose.theta < -157.5 && pose.theta >= -180))
  {
    grid_x0 = grid_x;
    grid_x1 = grid_x;
    grid_x2 = grid_x;
    grid_x3 = grid_x;
    grid_x4 = grid_x;
    grid_x5 = grid_x;

    grid_y0 = grid_y;
    grid_y1 = grid_y + 1;
    grid_y2 = grid_y + 2;
    grid_y3 = grid_y + 3;
    grid_y4 = grid_y - 1;
    grid_y5 = grid_y - 2;
  }else if(pose.theta < -22.5 && pose.theta >= -67.5)
  {
    grid_x0 = grid_x; 
    grid_y0 = grid_y;

    grid_x1 = grid_x - 1; 
    grid_y1 = grid_y - 1;

    grid_x2 = grid_x - 2; 
    grid_y2 = grid_y - 2;

    grid_x3 = grid_x - 3; 
    grid_y3 = grid_y - 3;

    grid_x4 = grid_x + 1; 
    grid_y4 = grid_y + 1;

    grid_x5 = grid_x + 2; 
    grid_y5 = grid_y + 2;
  }else if(pose.theta < -67.5 && pose.theta >= -112.5)
  {
    grid_y0 = grid_y;
    grid_y1 = grid_y;
    grid_y2 = grid_y;
    grid_y3 = grid_y;
    grid_y4 = grid_y;
    grid_y5 = grid_y;

    grid_x0 = grid_x;
    grid_x1 = grid_x - 1;
    grid_x2 = grid_x - 2;
    grid_x3 = grid_x - 3;
    grid_x4 = grid_x + 1;
    grid_x5 = grid_x + 2;
  }else if(pose.theta < -112.5 && pose.theta >= -157.5)
  {
    grid_x0 = grid_x; 
    grid_y0 = grid_y;

    grid_x1 = grid_x - 1; 
    grid_y1 = grid_y + 1;

    grid_x2 = grid_x - 2; 
    grid_y2 = grid_y + 2;

    grid_x3 = grid_x - 3; 
    grid_y3 = grid_y + 3;

    grid_x4 = grid_x + 1; 
    grid_y4 = grid_y - 1;

    grid_x5 = grid_x + 2; 
    grid_y5 = grid_y - 2;
  }
  
  cells[0] = cellIDToLinearCell(grid_x0, grid_y0);
  cells[1] = cellIDToLinearCell(grid_x1, grid_y1);
  cells[2] = cellIDToLinearCell(grid_x2, grid_y2);
  cells[3] = cellIDToLinearCell(grid_x3, grid_y3);
  cells[4] = cellIDToLinearCell(grid_x4, grid_y4);
  cells[5] = cellIDToLinearCell(grid_x5, grid_y5);
  return cells;
}

void Husky::publishPath(poseArray& poses)
{
  nav_msgs::Path path;  //used for rviz visualization
  path.header.seq = 0;
  path.header.frame_id = std::string("map");
  std::vector<int> cells(6, 0);
  std::vector<int> cells_old(6, 0);

  for(int i=0; i<poses.size(); ++i)
  {
    //publish pose as a path
    geometry_msgs::PoseStamped path_pose;
    path_pose.header.frame_id = path.header.frame_id;
    path_pose.header.stamp = ros::Time::now();
    path_pose.header.seq = i;
    path_pose.pose.position.x = poses[i].x;
    path_pose.pose.position.y = poses[i].y;
    path_pose.pose.position.z = 0;
    path.poses.emplace_back(path_pose);
    ++path.header.seq;

    //activate visited grid cells
    cells = obtainGridCells(poses[i]);
    
    if(i > 0)
    {
      for(int i = 0; i < cells.size(); ++i)
      {
        int x = cells[i] % MAP_WIDTH;
        int y = cells[i] / MAP_WIDTH;
        int x_old = cells_old[i] % MAP_WIDTH;
        int y_old = cells_old[i] / MAP_WIDTH;

        std::vector<int> interpolated_cells = interpolateCells(x_old, y_old, x, y);
        for(int c : interpolated_cells)
          updateMap(interpolated_cells);
      }
    }
    cells_old = cells;
    updateMap(cells);    

    //update occupancy grid
    m_map_publisher.publish(m_grid);
  
    //publish robot pose as TF frame, robot polygon, and cleaning_gadget polygon
    publishRobotPose(poses[i]);

    //publish path
    m_path_publisher.publish(path);

    ros::Duration(0.1).sleep();
  }
}

void Husky::publishRobotPose(const geometry_msgs::Pose2D& pose)
{
  //publish robot pose as TF frame
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "base_link";
  transformStamped.transform.translation.x = pose.x;
  transformStamped.transform.translation.y = pose.y;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, pose.theta*M_PI/180);  //convert degrees to quaternion
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  m_br->sendTransform(transformStamped);

  //publish robot polygon shape
  geometry_msgs::PolygonStamped robot_stamped;
  robot_stamped.polygon = m_robot_corners;
  robot_stamped.header.frame_id = "base_link";
  robot_stamped.header.stamp = transformStamped.header.stamp;
  m_robot_publisher.publish(robot_stamped);

  //publish cleaning_gadged polygon shape
  geometry_msgs::PolygonStamped cleaning_gadget_stamped;
  cleaning_gadget_stamped.polygon = m_gadget_corners;
  cleaning_gadget_stamped.header.frame_id = "base_link";
  cleaning_gadget_stamped.header.stamp = transformStamped.header.stamp;
  m_gadget_publisher.publish(cleaning_gadget_stamped);
}

int Husky::poseToCellID(geometry_msgs::Pose2D& pose)
{
  int x = pose.x/MAP_RESOLUTION;
  if(pose.x<0)
  {
    --x;
  }
  int y = pose.y/MAP_RESOLUTION;
  if(pose.y<0)
  {
    --y;
  }
  int cell = cellIDToLinearCell(x, y) ;

  return cell;
}

int Husky::cellIDToLinearCell(int x, int y)
{
  int cell = x+MAP_WIDTH/2 + (y+MAP_HEIGHT/2)*MAP_WIDTH ;
  return cell;
}

std::vector<int> Husky::interpolateCells(int x0, int y0, int x1, int y1)
{
  std::vector<int> cells;
  double dx = x1 - x0;
  double dy = y1 - y0;

  int x_direction, y_direction;
  if(dx > 0)
    x_direction = 1;
  else if(dx < 0)
    x_direction = -1;
  else
    x_direction = 0;

  if(dy > 0)
    y_direction = 1;
  else if (dy<0)
    y_direction = -1;
  else
    y_direction = 0;

  int x = x0;
  int y = y0;

  while(abs(x) !=  abs(x1) || abs(y) != abs(y1))
  {
    int cell = 0;
    if(x != x1)
    {
      x += x_direction;
      int cell = x + y * MAP_WIDTH ;  //convert cell grid index to linear index
      cells.push_back(cell);
    }
   
    if(y != y1)
    {
      y += y_direction;
      int cell = x + y * MAP_WIDTH ;  
      cells.push_back(cell);
    }
  }

  return cells;
}

void Husky::createMap()
{
  //creating map
  m_grid.header.frame_id = "map";
  m_grid.header.stamp = ros::Time::now();
  m_grid.header.seq = 0;

  m_grid.info.map_load_time = ros::Time::now();
  m_grid.info.resolution = MAP_RESOLUTION;  //[meter/cell]
  m_grid.info.width = MAP_WIDTH;            //[cells]
  m_grid.info.height = MAP_HEIGHT;          //[cells]
  m_grid.info.origin.position.x = -(MAP_WIDTH/2)*MAP_RESOLUTION;    //shift grid center
  m_grid.info.origin.position.y = -(MAP_HEIGHT/2)*MAP_RESOLUTION;   //shift grid center
  m_grid.info.origin.position.z = 0;
  m_grid.info.origin.orientation.w = 1;
  m_grid.info.origin.orientation.x = 0;
  m_grid.info.origin.orientation.y = 0;
  m_grid.info.origin.orientation.z = 0;
  m_grid.data.reserve(MAP_WIDTH*MAP_HEIGHT);

  //initilize map with 50 (unvisited-grey color)
  for(int i = 0; i < m_grid.info.width*m_grid.info.height; ++i)
  {
    m_grid.data.push_back(50);
  }
}

void Husky::updateMap(const std::vector<int>& cells)
{
  for(int cell : cells)
  {
    m_grid.data[cell] = 0;
  }
}

std::vector<double> Husky::calculateCurvature(poseArray& path)
{
  std::vector<double> curvatures;
  curvatures.reserve(path.size()-CURVATURE_RESOLUTION/2);

  for(int i = CURVATURE_RESOLUTION/2; i < path.size()-CURVATURE_RESOLUTION/2; ++i)
  {
    double points[10];
    points[0] = path[i-2].x;
    points[1] = path[i-2].y;
    points[2] = path[i-1].x;
    points[3] = path[i-1].y;
    points[4] = path[i].x;
    points[5] = path[i].y;
    points[6] = path[i+1].x;
    points[7] = path[i+1].y;
    points[8] = path[i+2].x;
    points[9] = path[i+2].y;
    
    BestFitIO input;
    input.numPoints = CURVATURE_RESOLUTION;
    input.points = points;
    input.verbosity = 0;
    BestFitIO output;
    int type = BestFitFactory::Circle;
    BestFit *b = BestFitFactory::Create(type, std::cout);
    b->Compute(input, output);
    delete b;
    
    double curvature = abs(1/(output.outputFields[BestFitIO::CircleRadius]));
    curvatures.emplace_back(curvature);
  }
  return curvatures;
}

double Husky::pathLength(poseArray& path)
{
  double path_length{0};
  for(int i=1; i<path.size(); ++i)
  {
    path_length += std::sqrt(pow((path[i].x - path[i-1].x), 2) + pow((path[i].y - path[i-1].y), 2));
  }

  return path_length;
}

double Husky::calculateArea(nav_msgs::OccupancyGrid& grid)
{
  int cell_count = 0;   //counter which holds the number of activated cells in the occupancy grid
  for(int cell : grid.data)
  {
    if(cell == 0)
      ++cell_count;
  }

  ROS_INFO("Number of activated cells = %d", cell_count);
  double area = cell_count * MAP_RESOLUTION * MAP_RESOLUTION;

  return area;
}

double Husky::calculateTotalTime(poseArray& path, std::vector<double>& curvatures)
{
  double total_time = 0;
  double v = 0;

  for(int i = 0; i < path.size() - CURVATURE_RESOLUTION/2; ++i)
  {
    //obtain relative curvature and the intented pose
    double k;
    if(i < CURVATURE_RESOLUTION/2 || i > curvatures.size())
    {
      k = K_MAX;
    }else
    {
      k = curvatures[i-CURVATURE_RESOLUTION/2];
    }

    //calculate speed(based on curvature at the relative pose)
    if(k < K_CRITICAL)
    {
      v = V_MAX;
    }else if(k >= K_CRITICAL && k < K_MAX)
    {
      v = V_MAX - (((V_MAX - V_MIN) / (K_MAX - K_CRITICAL)) * (k - K_CRITICAL));
    }else //if k >= K_MAX
    {
      v = V_MIN;
    }

    //obtain distance to be covered between two consecutive points
    double distance = std::sqrt(pow((path[i+1].x - path[i].x), 2) + pow((path[i+1].y - path[i].y), 2));

    //calculate step_time to the total time
    double step_time = distance/v;

    //add time_stem to the total time
    total_time += step_time;
    /* ROS_ERROR("path_id = %d", i);
    ROS_ERROR("curvature = %f", k);
    ROS_ERROR("distance = %f", distance);
    ROS_ERROR("speed = %f", v);
    ROS_ERROR("step_time = %f", step_time);
    ROS_ERROR("total_time = %f", total_time);
    ROS_ERROR("---------------"); */
  }
  
  return total_time;
}

bool Husky::init()
{
  m_br = std::make_shared<tf2_ros::TransformBroadcaster>();
  m_path_publisher = m_nh.advertise<nav_msgs::Path>("path", 1000);
  m_robot_publisher = m_nh.advertise<geometry_msgs::PolygonStamped>("robot", 1000);
  m_gadget_publisher = m_nh.advertise<geometry_msgs::PolygonStamped>("gadget", 1000);
  m_map_publisher = m_nh.advertise<nav_msgs::OccupancyGrid>("map", 1000);
  createMap();

  return true;
}

bool Husky::run( )
{
  //obtain data from json file (which is loaded as a ros parameter)
  m_robot_corners = obtainPolygon("/robot");
  m_gadget_corners = obtainPolygon("/cleaning_gadget");
  poseArray path = obtainPose("/path");
  
  //publish path (including robot frame, and cleaning gadged frame)
  publishPath(path);

  //calculate path_length, covered_area, and total required time.
  double path_length = pathLength(path);
  double area = calculateArea(m_grid);
  std::vector<double> curvatures_list = calculateCurvature(path);
  double time = calculateTotalTime(path, curvatures_list);

  //display results
  ROS_WARN("total path length = %fm", path_length);
  ROS_WARN("Total cleaned area = %f squared meters.", area);
  ROS_WARN("Total time required to cover the path = %fs", time);

  return true;
}





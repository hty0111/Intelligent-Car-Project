/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017.
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/teb_local_planner_ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

/************************************************/
// used to read .txt which is converted from .bag
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
/****************** test *****************/
#include <teb_local_planner/FeedbackMsg.h>

using namespace teb_local_planner; // it is ok here to import everything for testing purposes

// ============= Global Variables ================
// Ok global variables are bad, but here we only have a simple testing node.
PlannerInterfacePtr planner;
TebVisualizationPtr visual;
std::vector<ObstaclePtr> obst_vector;
ViaPointContainer via_points;
TebConfig config;

/**********************************************/
std::vector <std::vector <double> > global_path_plan;
// double linear_x, linear_y, angular_z;  // vel now
// std::vector <double> plan_linear_x, plan_linear_y, plan_angular_z;
/**********************************************/

boost::shared_ptr< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> > dynamic_recfg;
ros::Subscriber custom_obst_sub;
ros::Subscriber via_points_sub;
ros::Subscriber clicked_points_sub;
/**********************************************/
ros::Subscriber vel_sub;
ros::Publisher path_pub;
/**********************************************/
std::vector<ros::Subscriber> obst_vel_subs;
unsigned int no_fixed_obstacles;

// =========== Function declarations =============
std::vector <std::vector <double>> GetGlobalPath(double min_pathPoint_dist);
void DisplayGlobalPath();
void CB_mainCycle(const ros::TimerEvent& e);
void CB_publishCycle(const ros::TimerEvent& e);
void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level);
void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg);
void CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame, interactive_markers::InteractiveMarkerServer* marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb);
void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
void CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg);
void CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg);
/***************************************************************************/
// void CB_vel_now(const teb_local_planner::FeedbackMsg::ConstPtr& feedback_msg);
/***************************************************************************/
void CB_setObstacleVelocity(const geometry_msgs::TwistConstPtr& twist_msg, const unsigned int id);


// =============== Main function =================
int main( int argc, char** argv )
{
  ros::init(argc, argv, "test_optim_node");
  ros::NodeHandle n("~");
 
  
  // load ros parameters from node handle
  config.loadRosParamFromNodeHandle(n);
 
  ros::Timer cycle_timer = n.createTimer(ros::Duration(0.025), CB_mainCycle);
  ros::Timer publish_timer = n.createTimer(ros::Duration(0.1), CB_publishCycle);
  
  // setup dynamic reconfigure
  dynamic_recfg = boost::make_shared< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> >(n);
  dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(CB_reconfigure, _1, _2);
  dynamic_recfg->setCallback(cb);
  
  // setup callback for custom obstacles
  custom_obst_sub = n.subscribe("obstacles", 1, CB_customObstacle);
  
  // setup callback for clicked points (in rviz) that are considered as via-points
  clicked_points_sub = n.subscribe("/clicked_point", 5, CB_clicked_points);
  
  // setup callback for via-points (callback overwrites previously set via-points)
  via_points_sub = n.subscribe("via_points", 1, CB_via_points);
  
  /************************* subscribe vel **************************************/
  // vel_sub = n.subscribe("teb_feedback", 10, CB_vel_now);

  path_pub = n.advertise<nav_msgs::Path>("trajectory", 1);
  /******************************************************************************/

  // interactive marker server for simulated dynamic obstacles
  interactive_markers::InteractiveMarkerServer marker_server("marker_obstacles");

  // modified on 7.6
  obst_vector.push_back( boost::make_shared<PointObstacle>(2,-2) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(3,-3) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(4,-4) );

  // obst_vector.push_back( boost::make_shared<PointObstacle>(-3,-3) );
  // obst_vector.push_back( boost::make_shared<PointObstacle>(-7.6,0) );
  // obst_vector.push_back( boost::make_shared<PointObstacle>(3,3) );

  // obst_vector.push_back( boost::make_shared<PointObstacle>(-3,1) );
  // obst_vector.push_back( boost::make_shared<PointObstacle>(6,2) );
  // obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
//  obst_vector.push_back( boost::make_shared<LineObstacle>(1,1.5,1,-1.5) ); //90 deg
//  obst_vector.push_back( boost::make_shared<LineObstacle>(1,0,-1,0) ); //180 deg
//  obst_vector.push_back( boost::make_shared<PointObstacle>(-1.5,-0.5) );

  // Dynamic obstacles
  Eigen::Vector2d vel (0.1, -0.3);
  obst_vector.at(0)->setCentroidVelocity(vel);
  vel = Eigen::Vector2d(-0.3, -0.2);
  obst_vector.at(1)->setCentroidVelocity(vel);

  /*
  PolygonObstacle* polyobst = new PolygonObstacle;
  polyobst->pushBackVertex(1, -1);
  polyobst->pushBackVertex(0, 1);
  polyobst->pushBackVertex(1, 1);
  polyobst->pushBackVertex(2, 1);
 
  polyobst->finalizePolygon();
  obst_vector.emplace_back(polyobst);
  */
  
  for (unsigned int i=0; i<obst_vector.size(); ++i)
  {
    // setup callbacks for setting obstacle velocities
    std::string topic = "/test_optim_node/obstacle_" + std::to_string(i) + "/cmd_vel";
    obst_vel_subs.push_back(n.subscribe<geometry_msgs::Twist>(topic, 1, boost::bind(&CB_setObstacleVelocity, _1, i)));

    //CreateInteractiveMarker(obst_vector.at(i)[0],obst_vector.at(i)[1],i,&marker_server, &CB_obstacle_marker);  
    // Add interactive markers for all point obstacles
    boost::shared_ptr<PointObstacle> pobst = boost::dynamic_pointer_cast<PointObstacle>(obst_vector.at(i));
    if (pobst)
    {
      CreateInteractiveMarker(pobst->x(),pobst->y(),i, config.map_frame, &marker_server, &CB_obstacle_marker);  
    }
  }
  marker_server.applyChanges();
  
  // Setup visualization
  visual = TebVisualizationPtr(new TebVisualization(n, config));
  
  // Setup robot shape model
  RobotFootprintModelPtr robot_model = TebLocalPlannerROS::getRobotFootprintFromParamServer(n, config);
  
  // Setup planner (homotopy class planning or just the local teb planner)
  if (config.hcp.enable_homotopy_class_planning)
    planner = PlannerInterfacePtr(new HomotopyClassPlanner(config, &obst_vector, robot_model, visual, &via_points));
  else
    planner = PlannerInterfacePtr(new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points));
  
  // planner = PlannerInterfacePtr(new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points));

  no_fixed_obstacles = obst_vector.size();

  /**********************************************/
  // double display_dist = 0.1;
  // global_path_display = GetGlobalPath(display_dist);
  double plan_dist = 0.2;
  global_path_plan = GetGlobalPath(plan_dist);
  // DisplayGlobalPath();
  /**********************************************/

  ros::spin();

  return 0;
}

// Read .txt converted from .bag which contains gps messages
std::vector <std::vector <double>> GetGlobalPath(double min_pathPoint_dist)
{
  char *path = get_current_dir_name();
  std::string currentPath = path;
  // std::cout << currentPath << std::endl;
  free(path);
  path = NULL;
  int indexParent = currentPath.find_last_of("/");
  std::string parentPath = currentPath.substr(0, indexParent);
  // std::string targetFile = parentPath + "/10HZ_1.txt";
  std::string targetFile = parentPath + "/teb_ws/src/teb_local_planner/10HZ_1.txt";
  std::cout << targetFile << std::endl;
  std::ifstream infile(targetFile);

  std::vector <std::vector <double>> global_path;
  double point_dist = min_pathPoint_dist;
  double last_x = 0, last_y = 0;
  // int count1 = 0;

  // filter the first line
  std::string s;
  getline(infile, s);
  while (infile)
  {
    std::string s;
    if (!getline(infile, s)) break;

    std::istringstream ss(s);
    std::vector <std::string> record;
    std::vector <double>  global_point;
    int count2 = 0;
    while (ss)
    {
        std::string s;
        if (!getline(ss, s, ',')) break; // extract data according to ','
        // record x, y, quaternion, linear x,y, angular z
        if (count2 == 5 || count2 == 6 || count2 == 8 || count2 == 9 || count2 == 10 || count2 == 11 ||
            count2 == 48 || count2 == 49 || count2 == 53)
            record.push_back(s);
        count2++;
    }
    for (auto x : record)  global_point.push_back(atof(x.c_str()));  //string to double
    // dist between every two points should >= some value
    if (hypot(global_point[0]-last_x, global_point[1]-last_y) < point_dist)
        continue;
    last_x = global_point[0];
    last_y = global_point[1];
    tf::Quaternion q(global_point[2], global_point[3], global_point[4], global_point[5]);
    double yaw = tf::getYaw(q) + M_PI/2;
    if (yaw > M_PI)  yaw -= 2*M_PI;

    double odom_linear_x  = global_point[6];
    double odom_linear_y  = global_point[7];
    double odom_angular_z = global_point[8];

    global_point[2] = yaw;
    global_point[3] = odom_linear_x;
    global_point[4] = odom_linear_y;
    global_point[5] = odom_angular_z;
    global_point.resize(6);  // x, y, yaw, linear_x, linear_y, angular_z
    global_path.push_back(global_point);
    
    // count1++;
  }
  if (!infile.eof())
  {
    std::cerr << "Fooey!\n";
  }
  return global_path;
}


// This function isn't used
void DisplayGlobalPath(){
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = "gps";

  for (int i=0; i<global_path_plan.size(); i++){
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = global_path_plan[i][0];
    this_pose_stamped.pose.position.y = global_path_plan[i][1];

    this_pose_stamped.pose.orientation.x = 0.0;
    this_pose_stamped.pose.orientation.y = 0.0;
    this_pose_stamped.pose.orientation.z = 0.0;
    this_pose_stamped.pose.orientation.w = 1.0;

    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.header.frame_id = "gps";
    path_msg.poses.push_back(this_pose_stamped);
  }

  path_pub.publish(path_msg);
}


// Planning loop
void CB_mainCycle(const ros::TimerEvent& e)
{
  // planner->plan(PoseSE2(-4,0,0), PoseSE2(4,0,0)); // hardcoded start and goal for testing purposes
  
  /******************** vel_test *******************/
  // static int count = 0;
  // if (count == 100){
  //   std::cout << "x: " << std::endl;
  //   for (int i=0; i<plan_linear_x.size(); i++){
  //     std::cout << plan_linear_x[i] << std::endl;
  //   }
  //   // std::cout << "x: " << linear_x << ", y: " << linear_y << ", z: " << angular_z << std::endl;
  //   count = 0;
  // }
  // count++;

  /********************** test **********************/
  // TebOptimalPlanner teb_planner;
  // teb_planner.plan(PoseSE2(-4,0,0), PoseSE2(4,0,0));
  
  // FeedbackMsg msg;
  // msg.header.stamp = ros::Time::now();
  // // msg.header.frame_id = cfg_->map_frame;
  // // msg.header.frame_id = 0;
  // msg.selected_trajectory_idx = 0;
  
  // msg.trajectories.resize(1);
  // msg.trajectories.front().header = msg.header;
  // planner->getFullTrajectory(msg.trajectories.front().trajectory);

  
  static int count, start_idx;
  double dist_window = 1.0;
  int end_idx;
  if (count == 8)  count = 0;  // do planning every 8 times(0.2s)  v_update=1m/s
  // if (count == 14)  count = 0;  // do planning every 14 times(0.35s)  v_update≈0.6m/s
  if (count == 0){
    for(end_idx=start_idx; end_idx<global_path_plan.size(); end_idx++){
      if (hypot(global_path_plan[end_idx][0]-global_path_plan[start_idx][0],
                global_path_plan[end_idx][1]-global_path_plan[start_idx][1]) >= dist_window)
        break;
    }
    if (end_idx == global_path_plan.size())
      end_idx--;
    for(int i=start_idx+1; i<end_idx; i++)
      via_points.push_back( Eigen::Vector2d(global_path_plan[i][0], global_path_plan[i][1]) );
    
    /********************************************************************************/
    geometry_msgs::Twist* start_vel_ptr = NULL;
    geometry_msgs::Twist  start_vel;
    bool free_goal_vel = true;
    // if (plan_linear_x.size() > 0){
    //   start_vel_ptr->linear.x =  plan_linear_x[0];
    //   start_vel_ptr->linear.y =  plan_linear_y[0];
    //   start_vel_ptr->angular.z = plan_angular_z[0];
    // }

    start_vel.linear.x  = global_path_plan[start_idx][3];
    start_vel.linear.y  = global_path_plan[start_idx][4];
    start_vel.angular.z = global_path_plan[start_idx][5];
    start_vel_ptr = &start_vel;
    
    planner->plan(PoseSE2(global_path_plan[start_idx][0],global_path_plan[start_idx][1],global_path_plan[start_idx][2]),
                  PoseSE2(global_path_plan[end_idx][0],  global_path_plan[end_idx][1],  global_path_plan[end_idx][2]),
                  start_vel_ptr, free_goal_vel);
    /********************************************************************************/
    via_points.clear();
    if(start_idx < global_path_plan.size()-2)
      start_idx++;  // iterate forward
  }
  count++;
  
}

// Visualization loop
void CB_publishCycle(const ros::TimerEvent& e)
{
  planner->visualize();
  visual->publishObstacles(obst_vector);
  visual->publishViaPoints(via_points);
}

void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level)
{
  config.reconfigure(reconfig);
}

void CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame, interactive_markers::InteractiveMarkerServer* marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb)
{
  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker i_marker;
  i_marker.header.frame_id = frame;
  i_marker.header.stamp = ros::Time::now();
  std::ostringstream oss;
  //oss << "obstacle" << id;
  oss << id;
  i_marker.name = oss.str();
  i_marker.description = "Obstacle";
  i_marker.pose.position.x = init_x;
  i_marker.pose.position.y = init_y;
  i_marker.pose.orientation.w = 1.0f; // make quaternion normalized

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.id = id;
  box_marker.scale.x = 0.2;
  box_marker.scale.y = 0.2;
  box_marker.scale.z = 0.2;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;
  box_marker.pose.orientation.w = 1.0f; // make quaternion normalized

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  i_marker.controls.push_back( box_control );

  // create a control which will move the box, rviz will insert 2 arrows
  visualization_msgs::InteractiveMarkerControl move_control;
  move_control.name = "move_x";
  move_control.orientation.w = 0.707107f;
  move_control.orientation.x = 0;
  move_control.orientation.y = 0.707107f;
  move_control.orientation.z = 0;
  move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;


  // add the control to the interactive marker
  i_marker.controls.push_back(move_control);

  // add the interactive marker to our collection
  marker_server->insert(i_marker);
  marker_server->setCallback(i_marker.name,feedback_cb);
}

void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::stringstream ss(feedback->marker_name);
  unsigned int index;
  ss >> index;
  
  if (index>=no_fixed_obstacles) 
    return;
  PointObstacle* pobst = static_cast<PointObstacle*>(obst_vector.at(index).get());
  pobst->position() = Eigen::Vector2d(feedback->pose.position.x,feedback->pose.position.y);	  
}

void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg)
{
  // resize such that the vector contains only the fixed obstacles specified inside the main function
  obst_vector.resize(no_fixed_obstacles);
  
  // Add custom obstacles obtained via message (assume that all obstacles coordiantes are specified in the default planning frame)  
  for (size_t i = 0; i < obst_msg->obstacles.size(); ++i)
  {
    if (obst_msg->obstacles.at(i).polygon.points.size() == 1 )
    {
      if (obst_msg->obstacles.at(i).radius == 0) 
      {
        obst_vector.push_back(ObstaclePtr(new PointObstacle( obst_msg->obstacles.at(i).polygon.points.front().x,
                                                           obst_msg->obstacles.at(i).polygon.points.front().y )));
      }
      else
      {
        obst_vector.push_back(ObstaclePtr(new CircularObstacle( obst_msg->obstacles.at(i).polygon.points.front().x,
                                                            obst_msg->obstacles.at(i).polygon.points.front().y,
                                                            obst_msg->obstacles.at(i).radius )));
      }
    }
    else if (obst_msg->obstacles.at(i).polygon.points.empty())
    {
      ROS_WARN("Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
      continue;
    }
    else
    {
      PolygonObstacle* polyobst = new PolygonObstacle;
      for (size_t j=0; j<obst_msg->obstacles.at(i).polygon.points.size(); ++j)
      {
        polyobst->pushBackVertex( obst_msg->obstacles.at(i).polygon.points[j].x,
                                  obst_msg->obstacles.at(i).polygon.points[j].y );
      }
      polyobst->finalizePolygon();
      obst_vector.push_back(ObstaclePtr(polyobst));
    }

    if(!obst_vector.empty())
      obst_vector.back()->setCentroidVelocity(obst_msg->obstacles.at(i).velocities, obst_msg->obstacles.at(i).orientation);
  }
}


void CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg)
{
  // we assume for simplicity that the fixed frame is already the map/planning frame
  // consider clicked points as via-points
  via_points.push_back( Eigen::Vector2d(point_msg->point.x, point_msg->point.y) );
  ROS_INFO_STREAM("Via-point (" << point_msg->point.x << "," << point_msg->point.y << ") added.");
  if (config.optim.weight_viapoint<=0)
    ROS_WARN("Note, via-points are deactivated, since 'weight_via_point' <= 0");
}

void CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg)
{
  ROS_INFO_ONCE("Via-points received. This message is printed once.");
  via_points.clear();
  for (const geometry_msgs::PoseStamped& pose : via_points_msg->poses)
  {
    via_points.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
}

// void CB_vel_now(const teb_local_planner::FeedbackMsg::ConstPtr& feedback_msg)
// {
//   plan_linear_x.clear();
//   plan_linear_y.clear();
//   plan_angular_z.clear();
//   int num = feedback_msg->trajectories.front().trajectory.size();
//   for (int i=0; i<num; i++){
//     plan_linear_x.push_back(feedback_msg->trajectories.front().trajectory[i].velocity.linear.x);
//     plan_linear_y.push_back(feedback_msg->trajectories.front().trajectory[i].velocity.linear.y);
//     plan_angular_z.push_back(feedback_msg->trajectories.front().trajectory[i].velocity.angular.z);
//   }
//   // linear_x  = feedback_msg->trajectories.front().trajectory.front().velocity.linear.x;
//   // linear_y  = feedback_msg->trajectories.front().trajectory.front().velocity.linear.y;
//   // angular_z = feedback_msg->trajectories.front().trajectory.front().velocity.angular.z;
// }

void CB_setObstacleVelocity(const geometry_msgs::TwistConstPtr& twist_msg, const unsigned int id)
{
  if (id >= obst_vector.size())
  {
    ROS_WARN("Cannot set velocity: unknown obstacle id.");
    return;
  }

  Eigen::Vector2d vel (twist_msg->linear.x, twist_msg->linear.y);
  obst_vector.at(id)->setCentroidVelocity(vel);
}

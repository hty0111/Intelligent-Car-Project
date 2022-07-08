



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
#include <teb_local_planner/optimal_planner.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

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
std::vector <double> linear_x, linear_y, angular_z;
/**********************************************/


ros::Publisher path_pub;
/**********************************************/

// =========== Function declarations =============
std::vector <std::vector <double>> GetGlobalPath(double min_pathPoint_dist);



// =============== Main function =================
int main( int argc, char** argv )
{
  ros::init(argc, argv, "test_display_globalPath");
  ros::NodeHandle n;
  
  /**********************************************/
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = "odom";

  path_pub = n.advertise<nav_msgs::Path>("trajectory", 1, true);
  /**********************************************/

  // double display_dist = 0.1;
  // global_path_display = GetGlobalPath(display_dist);
  double plan_dist = 0.2;
  global_path_plan = GetGlobalPath(plan_dist);
  
  int count = 0;
  // ros::Rate loop_rate(100);
  while (ros::ok()){
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = global_path_plan[count][0];
    this_pose_stamped.pose.position.y = global_path_plan[count][1];

    this_pose_stamped.pose.orientation.x = 0.0;
    this_pose_stamped.pose.orientation.y = 0.0;
    this_pose_stamped.pose.orientation.z = 0.0;
    this_pose_stamped.pose.orientation.w = 1.0;

    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.header.frame_id = "odom";
    path_msg.poses.push_back(this_pose_stamped);

    path_pub.publish(path_msg);

    count++;
    if (count >= global_path_plan.size()-1)
      count = global_path_plan.size()-1;
    
    // loop_rate.sleep();
  }
  /**********************************************/

  // ros::spin();

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
        if (count2 == 5 || count2 == 6)  // pose.pose.position.x and pose.pose.position.y
            record.push_back(s);
        count2++;
    }
    for (auto x : record)  global_point.push_back(atof(x.c_str()));  //string to double
    // dist between every two points should >= some value
    if (hypot(global_point[0]-last_x, global_point[1]-last_y) < point_dist)
        continue;
    last_x = global_point[0];
    last_y = global_point[1];
    global_path.push_back(global_point);
    
    // count1++;
  }
  if (!infile.eof())
  {
    std::cerr << "Fooey!\n";
  }
  return global_path;
}



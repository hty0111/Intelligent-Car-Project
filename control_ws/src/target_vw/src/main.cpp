/*
 * @Description: 
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-07-03 20:27:19
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "target_vw");
    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/target_vw", 1);

    geometry_msgs::Twist vel_msg;
    ros::Rate rate = 10;

    while (ros::ok())
    {
        vel_msg.linear.x = -10;
        vel_msg.angular.x = 2;
        vel_pub.publish(vel_msg);

        ROS_INFO("PUBLISH | vel: %.2f, angle:%.2f", vel_msg.linear.x, vel_msg.angular.x);
        rate.sleep();
    }

    return 0;
}


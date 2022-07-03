/*
 * @Description:
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-07-02 21:21:03
 */

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "yhs_can_msgs/ctrl_cmd.h"
#include "yhs_can_msgs/io_cmd.h"
#include "yhs_can_msgs/ctrl_fb.h"
#include "yhs_can_msgs/lr_wheel_fb.h"
#include "yhs_can_msgs/rr_wheel_fb.h"
#include "yhs_can_msgs/io_fb.h"
#include "yhs_can_msgs/odo_fb.h"
#include "yhs_can_msgs/bms_Infor_fb.h"
#include "yhs_can_msgs/bms_flag_Infor_fb.h"
#include "yhs_can_msgs/Drive_MCUEcoder_fb.h"
#include "yhs_can_msgs/Veh_Diag_fb.h"
#include "pid.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <string>

using namespace chassis_control;

class ChassisControl
{
private:
    // The car state and parameters
    double wheelbase;
    double current_velocity, current_steering_angle;
    double target_velocity, target_steering_angle, target_w;
    double max_velocity, max_steering_angle;
    yhs_can_msgs::ctrl_cmd ctrl_msg;
    yhs_can_msgs::io_cmd io_msg;
//    teb_local_planner::FeedbackMsg teb_msg;

    // The controller params
    double vel_kp, vel_ki, vel_kd;
    double angle_kp, angle_ki, angle_kd;
    double max_vel_iout, max_angle_iout;

    // A ROS node
    ros::NodeHandle n;

    // A timer to update output
    ros::Timer update_timer;

    // Control rate
    double control_rate;

    // PID controller
    PID vel_pid, angle_pid;

    // Listen for target and feedback
    ros::Subscriber target_vw_sub;
    ros::Subscriber ctrl_fb_sub;
//    ros::Subscriber lr_wheel_fb_sub = n.subscribe<yhs_can_msgs::lr_wheel_fb>("lr_wheel_fb",5, &ChassisControl::lr_wheel_fbCallBack);
//    ros::Subscriber rr_wheel_fb_sub = n.subscribe<yhs_can_msgs::rr_wheel_fb>("rr_wheel_fb",5, &rr_wheel_fbCallBack);
//    ros::Subscriber io_fb_sub = n.subscribe<yhs_can_msgs::io_fb>("io_fb",5, &io_fbCallBack);
//    ros::Subscriber odo_fb_sub = n.subscribe<yhs_can_msgs::odo_fb>("odo_fb",5, &odo_fbCallBack);
//    ros::Subscriber bms_Infor_fb_sub = n.subscribe<yhs_can_msgs::bms_Infor_fb>("bms_Infor_fb",5, &bms_Infor_fbCallBack);
//    ros::Subscriber bms_flag_Infor_fb_sub = n.subscribe<yhs_can_msgs::bms_flag_Infor_fb>("bms_flag_Infor_fb",5, &bms_flag_Infor_fbCallback);
//    ros::Subscriber Drive_MCUEcoder_fb_sub = n.subscribe<yhs_can_msgs::Drive_MCUEcoder_fb>("Drive_MCUEcoder_fb",5, &Drive_MCUEcoder_fbCallBack);
//    ros::Subscriber Veh_Diag_fb_sub = n.subscribe<yhs_can_msgs::Veh_Diag_fb>("Veh_Diag_fb",5, &Veh_Diag_fbCallBack);

    // Publish velocity and angle
    ros::Publisher ctrl_cmd_pub;
    ros::Publisher io_cmd_pub;


public:
    ChassisControl()
    {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // Initialize the driving command
        target_velocity = 0;
        target_steering_angle = 0;
        current_velocity = 0;
        current_steering_angle = 0;
        io_msg.io_cmd_enable = 1;

        // Get the vehicle params
        n.getParam("wheelbase", wheelbase);
        n.getParam("max_velocity", max_velocity);
        n.getParam("max_steering_angle", max_steering_angle);

        // Get the pid controller params
        n.getParam("control_rate", control_rate);
        n.getParam("vel_kp", vel_kp);
        n.getParam("vel_ki", vel_ki);
        n.getParam("vel_kd", vel_kd);
        n.getParam("angle_kp", angle_kp);
        n.getParam("angle_ki", angle_ki);
        n.getParam("angle_kd", angle_kd);
        n.getParam("max_vel_iout", max_vel_iout);
        n.getParam("max_angle_iout", max_angle_iout);

        // Get the topic names
        std::string target_vw_topic, ctrl_fb_topic, ctrl_cmd_topic, io_cmd_topic;
        n.getParam("target_vw_topic", target_vw_topic);
        n.getParam("ctrl_fb_topic", ctrl_fb_topic);
        n.getParam("ctrl_cmd_topic", ctrl_cmd_topic);
        n.getParam("io_cmd_topic", io_cmd_topic);

        // Init pid controller
        vel_pid = PID(vel_kp, vel_ki, vel_kd, max_velocity, max_vel_iout);
        angle_pid = PID(angle_kp, angle_ki, angle_kd, max_steering_angle, max_angle_iout);

        // Start a subscriber to listen to target v and w
        target_vw_sub = n.subscribe(target_vw_topic, 1, &ChassisControl::target_vwCallBack, this);
        ctrl_fb_sub = n.subscribe<yhs_can_msgs::ctrl_fb>(ctrl_fb_topic,1, &ChassisControl::ctrl_fbCallBack, this);

        // Make a publisher for velocity and angle command
        ctrl_cmd_pub = n.advertise<yhs_can_msgs::ctrl_cmd>(ctrl_cmd_topic, 1);
        // Make a publisher for io command
        io_cmd_pub = n.advertise<yhs_can_msgs::io_cmd>(io_cmd_topic, 1);
    }
    
// TODO
//    void target_vwCallBack(const teb_local_planner::FeedbackMsg::ConstPtr & msg)
//    {
//        float linear_x = msg->trajectories.front().trajectory.front().velocity.linear.x;
//        float linear_y = msg->trajectories.front().trajectory.front().velocity.linear.y;
//        target_velocity = sqrt(linear_x * linear_x + linear_y * linear_y);
//        target_w = msg->trajectories.front().trajectory.front().velocity.angular.z;
//    }
    void target_vwCallBack(const geometry_msgs::Twist::ConstPtr & msg)
    {
//        ROS_INFO("Receive target v and w");
        target_velocity = msg->linear.x;
        target_w = msg->angular.x;
    }

    void ctrl_fbCallBack(const yhs_can_msgs::ctrl_fb::ConstPtr & msg)
    {
        current_velocity = msg->ctrl_fb_velocity;
        current_steering_angle = msg->ctrl_fb_steering;
    }

//    void lr_wheel_fbCallBack(const yhs_can_msgs::lr_wheel_fb msg){ROS_INFO("GET lr_wheel_fb!");}
//    void rr_wheel_fbCallBack(const yhs_can_msgs::rr_wheel_fb msg){ROS_INFO("GET rr_wheel_fb!");}
//    void io_fbCallBack(const yhs_can_msgs::io_fb msg){ROS_INFO("GET io_fb!");}
//    void odo_fbCallBack(const yhs_can_msgs::odo_fb msg){ROS_INFO("GET odo_fb!");}
//    void bms_Infor_fbCallBack(const yhs_can_msgs::bms_Infor_fb msg){ROS_INFO("GET bms_Infor_fb!");}
//    void bms_flag_Infor_fbCallback(const yhs_can_msgs::bms_flag_Infor_fb msg){ROS_INFO("GET bms_flag_Infor_fb!");}
//    void Drive_MCUEcoder_fbCallBack(const yhs_can_msgs::Drive_MCUEcoder_fb msg){ROS_INFO("GET Drive_MCUEcoder_fb!");}
//    void Veh_Diag_fbCallBack(const yhs_can_msgs::Veh_Diag_fb msg){ROS_INFO("GET Veh_Diag_fb!");}

    void run()
    {
        // Start a timer to output control messages
        update_timer = n.createTimer(ros::Duration(control_rate), &ChassisControl::update_output, this);
        ros::spin();
    }
    
    void update_output(const ros::TimerEvent&)
    {
        // calculate target angle from v and w
        if (target_velocity == 0)
            target_steering_angle = 0;
        else if (target_velocity > 0)
            target_steering_angle = atan2(wheelbase * target_w, target_velocity) / M_PI * 180;
        else
            target_steering_angle = -atan2(wheelbase * target_w, abs(target_velocity)) / M_PI * 180;

        ctrl_msg.ctrl_cmd_velocity = (float) abs(vel_pid.compute(target_velocity, current_velocity));
        ctrl_msg.ctrl_cmd_steering = (float) angle_pid.compute(target_steering_angle, current_steering_angle);

        if (ctrl_msg.ctrl_cmd_velocity >= 0)
            ctrl_msg.ctrl_cmd_gear = 4; // D
        else
            ctrl_msg.ctrl_cmd_gear = 2; // R

        ctrl_cmd_pub.publish(ctrl_msg);
//        ROS_INFO("tar_w: %.2f, tar_v: %.2f", target_w, target_velocity);
//        ROS_INFO("tar_angle: %.2f, fdb_angle: %.2f", target_steering_angle, current_steering_angle);
        ROS_INFO("vel: %.2f, angle: %.2f, gear: %d", ctrl_msg.ctrl_cmd_velocity, ctrl_msg.ctrl_cmd_steering, ctrl_msg.ctrl_cmd_gear);

        if (target_steering_angle > 15 && target_velocity > 0)
            io_msg.io_cmd_turn_lamp = 1;
        else if (target_steering_angle < -15 && target_velocity > 0)
            io_msg.io_cmd_turn_lamp = 2;
        else
            io_msg.io_cmd_turn_lamp = 0;
        io_cmd_pub.publish(io_msg);
    }
};


int main(int argc,char **argv)
{
    ros::init(argc,argv,"chassis_control");
    ChassisControl cc;
    cc.run();
    return 0;
}

/*
 * @Description:
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-07-02 21:21:03
 */

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
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
#include "teb_local_planner/FeedbackMsg.h"
#include "pid.h"

#include <cstdlib>
#include <cstring>

using namespace chassis_control;

class ChassisControl
{
private:
    // The car state and parameters
    double wheelbase;
    double current_velocity, current_steering_angle;
    double target_velocity, target_steering_angle, target_w;
    double max_velocity, max_steering_angle;
    uint8_t current_gear;

    // The control message
    yhs_can_msgs::ctrl_cmd ctrl_msg;

    // The controller params
    double vel_kp, vel_ki, vel_kd;
    double angle_kp, angle_ki, angle_kd;
    double max_vel_iout, max_angle_iout;

    // A ROS node
    ros::NodeHandle n;

    // Start to control
    bool AUTO_MODE;

    // A timer to update output
    ros::Timer update_timer;

    // PID controller
    double control_rate;
    bool USE_PID_CONTROLLER;
    PID vel_pid, angle_pid;

    // Listen for target and feedback
    ros::Subscriber target_vw_sub;
    ros::Subscriber ctrl_fb_sub;

    // Publish velocity and angle
    ros::Publisher ctrl_cmd_pub;

    // Server for auto mode
    ros::ServiceServer auto_mode_server;

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

        // Get the vehicle params
        n.getParam("wheelbase", wheelbase);
        n.getParam("max_velocity", max_velocity);
        n.getParam("max_steering_angle", max_steering_angle);

        // Get the pid controller params
        n.getParam("USE_PID_CONTROLLER", USE_PID_CONTROLLER);
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
        std::string target_vw_topic, ctrl_fb_topic, ctrl_cmd_topic, auto_mode_service;
        n.getParam("target_vw_topic", target_vw_topic);
        n.getParam("ctrl_fb_topic", ctrl_fb_topic);
        n.getParam("ctrl_cmd_topic", ctrl_cmd_topic);
        n.getParam("auto_mode_service", auto_mode_service);

        // Init pid controller
        vel_pid = PID(vel_kp, vel_ki, vel_kd, max_velocity, max_vel_iout);
        angle_pid = PID(angle_kp, angle_ki, angle_kd, max_steering_angle, max_angle_iout);

        // Start subscribers
        target_vw_sub = n.subscribe(target_vw_topic, 1, &ChassisControl::target_vw_Callback, this);
        ctrl_fb_sub = n.subscribe<yhs_can_msgs::ctrl_fb>(ctrl_fb_topic,1, &ChassisControl::ctrl_fb_Callback, this);

        // Make publishers
        ctrl_cmd_pub = n.advertise<yhs_can_msgs::ctrl_cmd>(ctrl_cmd_topic, 1);

        // Make service
        auto_mode_server = n.advertiseService(auto_mode_service, &ChassisControl::auto_mode_Callback, this);
    }
    
    // TODO teb发布的消息
    void target_vw_Callback(const teb_local_planner::FeedbackMsg::ConstPtr & msg)
    {
        double linear_x = msg->trajectories.front().trajectory.front().velocity.linear.x;
        double linear_y = msg->trajectories.front().trajectory.front().velocity.linear.y;
        target_velocity = sqrt(linear_x * linear_x + linear_y * linear_y);
        target_w = msg->trajectories.front().trajectory.front().velocity.angular.z;
    }

    /* for debug */
//    void target_vw_Callback(const geometry_msgs::Twist::ConstPtr & msg)
//    {
//        target_velocity = msg->linear.x;
//        target_w = msg->angular.x;
//    }

    void ctrl_fb_Callback(const yhs_can_msgs::ctrl_fb::ConstPtr & msg)
    {
        current_velocity = msg->ctrl_fb_velocity;
        current_steering_angle = msg->ctrl_fb_steering;
        current_gear = msg->ctrl_fb_gear;
    }

    // TODO
    bool auto_mode_Callback(std_srvs::SetBool::Request & req, std_srvs::SetBool::Response & res)
    {
        AUTO_MODE = req.data;
        res.success = true;
        res.message = "Start auto mode!";
        return true;
    }

    void run()
    {
        // Start a timer to output control messages
        update_timer = n.createTimer(ros::Duration(control_rate), &ChassisControl::update_output, this);
        ros::spin();
    }
    
    void update_output(const ros::TimerEvent&)
    {
        if (AUTO_MODE)
        {
            // Calculate target angle from v and w
            if (target_velocity == 0)
            {
                target_steering_angle = 0;
                ctrl_msg.ctrl_cmd_gear = 3;     // N
            }
            else if (target_velocity > 0)
            {
                target_steering_angle = atan2(wheelbase * target_w, target_velocity) / M_PI * 180;
                ctrl_msg.ctrl_cmd_gear = 4;     // D
            }
            else
            {
                target_steering_angle = -atan2(wheelbase * target_w, abs(target_velocity)) / M_PI * 180;
                ctrl_msg.ctrl_cmd_gear = 2;     // R
            }

            /* for debug */
//        n.getParam("debug_vel", target_velocity);
//        n.getParam("debug_angle", target_steering_angle);

            if (USE_PID_CONTROLLER)
            {
                ctrl_msg.ctrl_cmd_velocity = (float) vel_pid.compute(abs(target_velocity), current_velocity);
                ctrl_msg.ctrl_cmd_steering = (float) angle_pid.compute(target_steering_angle, current_steering_angle);
            }
            else
            {
                ctrl_msg.ctrl_cmd_velocity = (float) abs(target_velocity);
                ctrl_msg.ctrl_cmd_steering = (float) target_steering_angle;
            }
        }
        else
        {
            ctrl_msg.ctrl_cmd_gear = 0;
            ctrl_msg.ctrl_cmd_velocity = 0;
            ctrl_msg.ctrl_cmd_steering = 0;
        }

        ROS_INFO("Velocity --- output: %.2f | target: %.2f | current: %.2f", ctrl_msg.ctrl_cmd_velocity, target_velocity, current_velocity);
        ROS_INFO("PID --- pout: %.2f | iout: %.2f | dout: %.2f", vel_pid.get_pout(), vel_pid.get_iout(), vel_pid.get_dout());
        ROS_INFO("Angle --- output: %.2f | target: %.2f | current: %.2f", ctrl_msg.ctrl_cmd_steering, target_steering_angle, current_steering_angle);
        ROS_INFO("Gear --- target: %d | current: %d", ctrl_msg.ctrl_cmd_gear, current_gear);

//        /* for debug */
//        ctrl_msg.ctrl_cmd_gear = 2;
//        ctrl_msg.ctrl_cmd_velocity = 0.2;
//        ctrl_msg.ctrl_cmd_steering = 0;

        ctrl_cmd_pub.publish(ctrl_msg);
    }
};


int main(int argc,char **argv)
{
    ros::init(argc,argv,"chassis_control");
    ChassisControl cc;
    cc.run();
    return 0;
}

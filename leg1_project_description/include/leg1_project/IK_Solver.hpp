#ifndef ROBOT_H
#define ROBOT_H

#include <ros/ros.h>

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <std_srvs/Trigger.h>

namespace IK_Solver
{

class Robot 
{
    public:
        Robot();
        ~ Robot();

        std::vector<double> Leg_IK(std::vector<double> final_value);
        std::vector<double> Straight_Line_Trajectory(double time);
        std::vector<double> Cubic_Bezier_Curve_Trajectory(double time);
        bool getTrajectoryInfo(trajectory_msgs::JointTrajectory *jnt_tra_msg);
        bool moveCommandMsgCallback(std_srvs::Trigger::Request &req,
                                std_srvs::Trigger::Response &res);
        std::vector<double> joint_state__;
        


    private:
        ros::NodeHandle node_handle_;
        ros::NodeHandle priv_node_handle_;

        ros::Publisher joint_trajectory_pub_;
        // ROS Service Server
        ros::ServiceServer move_command_server_;
        // ROS Service Client
        trajectory_msgs::JointTrajectory *jnt_tra_msg_;
        bool is_loop_;

        double omega;
        double phi;
        double psi;

        // Center of body
        double xm, ym, zm;
        
        

};

}

#endif
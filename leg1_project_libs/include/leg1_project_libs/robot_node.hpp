#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <Eigen/Dense>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

class Leg1Project
{
    public:
        Leg1Project(ros::NodeHandle *nh, ros::NodeHandle *nh_priv);       // constructor
        ~Leg1Project();      // destructor
        void controlLoop(const ros::TimerEvent& event);
        std::vector<double> joint_state_;
        std::vector<double> solveInverseKinematics(std::vector<double> position_info);
        void publishJoints(float target_joint_position[12]);
        std::vector<double> straightLineStanceTrajectory(double time, double duration);

    private:
        ros::Subscriber teleop_input_subscriber;
        ros::Publisher joint_command_publisher;
        ros::Timer loop_timer;
        std::vector<std::string> joint_names_;
 

};


#endif
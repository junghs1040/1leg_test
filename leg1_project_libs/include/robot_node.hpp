#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>


class Leg1Project
{
    public:
        Leg1Project(ros::NodeHandle *nh, ros::NodeHandle *nh_priv);       // constructor
        //~Leg1Project();      // destructor
        void controlLoop(const ros::TimerEvent& event);
        std::vector<float> joint_state_;
        std::vector<float> solveInverseKinematics(std::vector<float> position_info);
        void publishJoints(std::vector<float> target_joint_position);
        void straightLineStanceTrajectory(double duration_);

    private:
        ros::Subscriber teleop_input_subscriber;
        ros::Publisher joint_command_publisher;
        ros::Timer loop_timer;
        std::vector<std::string> joint_names_;
        std::vector<float> target_joint_position;
        float time_ =0.0 ;
 

};


#endif
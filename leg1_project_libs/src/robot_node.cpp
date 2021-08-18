#include <iostream>
#include <vector>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "urdf/model.h"
#include "leg1_project_libs/robot_node.hpp"

Leg1Project::Leg1Project(ros::NodeHandle *nh, ros::NodeHandle *nh_priv)
{
    double loop_rate = 100.0;
    std::string joint_control_topic = "joint_group_position_controller/command";
    //*number received by controller check and choice the walking pattern
    //*store the trajectory information and publish that information
    
    //teleop_input_subscriber = nh->subscribe("cmd_vel", 1000, &Leg1Project::msgCallback, this);
    //Joint trajectory publisher to  Gazebo
    joint_command_publisher = nh->advertise<trajectory_msgs::JointTrajectory>("joint_group_position_controller/command", 1);

    //joint_state_publisher = nh->advertise<sensor_msgs::JointState>("joint_states", 1);
    //TODO : contact info publisher
    //contact_info_publisher = nh->advertise<champ_msgs::ContactsStamped>("foot_contacts", 1);
    joint_names_ = {"joint1","joint2","joint3"};

    loop_timer =nh_priv->createTimer(ros::Duration(1/loop_rate), &Leg1Project::controlLoop,this);

}

Leg1Project::~Leg1Project()
{}

void Leg1Project::controlLoop(const ros::TimerEvent& event)
{
    float target_joint_position[12];
    std::vector<double> target_foot_position;
    straightLineStanceTrajectory();

    publishJoints(target_joint_position);
}


void Leg1Project::publishJoints(float target_joint_position[12])
{
    trajectory_msgs::JointTrajectory joints_cmd_msg;
    joints_cmd_msg.header.stamp = ros::Time::now();
    joints_cmd_msg.joint_names = joint_names_;
    
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(12);

    point.time_from_start = ros::Duration(1.0/60.0);
    for (int i=0; i<12; i++)
    {
        point.positions[i] = target_joint_position[i];
    }

    joints_cmd_msg.points.push_back(point);
    joint_command_publisher.publish(joints_cmd_msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "leg1_project");
    
    ros::NodeHandle nh("");
    ros::NodeHandle nh_priv("~");
    
    
    Leg1Project leg(&nh, &nh_priv);

    ros::spin();
    return 0;
}

#include <iostream>
#include <vector>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include "urdf/model.h"
#include "leg1_project/IK_Solver.hpp"

using namespace IK_Solver;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"robot_node");
    ros::NodeHandle nh;
    Robot robot;
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 100);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(5); //50Hz = 0.5s loop

    std::string robot_description_string;
    nh.param("robot_description", robot_description_string, std::string());
    std::vector<std::string> joint_name = {"joint1", "joint2", "joint3"};
    sensor_msgs::JointState joint_state;
    std::vector<double> joint_state_;
    //std::vector<double> test_position ={ -185.55, 78.00, -215.7 };
    //std::vector<double> test_position ={ 80.00, 78.00, -215.7 };
    //std::vector<double> test_position ={ 0.0, 78.00, -215.7 };

    int time = 0;

    while (ros::ok())
    {
  
        ROS_INFO("update leg1 joint state");

        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(3); // joint_State의 name 3개로 확장
        joint_state.position.resize(3); // joint_state의 position 3개로 확장
        double now_time = ros::Time::now().toSec();
        //joint_state_ = {1.5,1.5,-1.5};
        //joint_state_ = robot.Leg_IK(test_position);
        std::vector<double> test_position = robot.Straight_Line_Trajectory((double)time);
        //std::vector<double> test_position = robot.Cubic_Bezier_Curve_Trajectory((double)time);
        joint_state_ = robot.Leg_IK(test_position);

        for (int i=0; i<3; i++)
        {
            joint_state.name[i] = joint_name[i];
        }

        joint_state.position[0] = joint_state_[0];
        joint_state.position[1] = joint_state_[1];
        joint_state.position[2] = joint_state_[2];

        time ++;
        ROS_INFO("end effector position : {%d, %d, %d}", (int)test_position[0],(int)test_position[1],(int)test_position[2]);
        joint_pub.publish(joint_state);
        
        if (time >=100)
        {
            ros::spin();
        }
        loop_rate.sleep();
    }

}
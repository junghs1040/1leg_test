#include <iostream>
#include <vector>
#include <string>
#include <robot_node.hpp>

Leg1Project::Leg1Project(ros::NodeHandle *nh, ros::NodeHandle *nh_priv)
{
    double loop_rate = 100.0;
    std::string joint_control_topic = "joint_group_position_controller/command";
    //*number received by controller check and choice the walking pattern
    //*store the trajectory information and publish that information
    
    //teleop_input_subscriber = nh->subscribe("cmd_vel", 1000, &Leg1Project::msgCallback, this);
    //Joint trajectory publisher to  Gazebo
    joint_command_publisher = nh_priv->advertise<trajectory_msgs::JointTrajectory>("joint_group_position_controller/command", 1);

    //joint_state_publisher = nh->advertise<sensor_msgs::JointState>("joint_states", 1);
    //TODO : contact info publisher
    //contact_info_publisher = nh->advertise<champ_msgs::ContactsStamped>("foot_contacts", 1);
    joint_names_ = {"joint1","joint2","joint3"};

    loop_timer =nh_priv->createTimer(ros::Duration(1/loop_rate), &Leg1Project::controlLoop, this);

}

//Leg1Project::~Leg1Project()
//{}

void Leg1Project::controlLoop(const ros::TimerEvent& event)
{

    straightLineStanceTrajectory(3.0);

    publishJoints(target_joint_position);
}


void Leg1Project::publishJoints(std::vector<float> target_joint_position)
{
    trajectory_msgs::JointTrajectory joints_cmd_msg;
    joints_cmd_msg.header.stamp = ros::Time::now();
    joints_cmd_msg.joint_names = joint_names_;
    
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(3);

    point.time_from_start = ros::Duration(1.0/60.0);
    for (int i=0; i<3; i++)
    {
        point.positions[i] = target_joint_position[i];
    }

    joints_cmd_msg.points.push_back(point);
    joint_command_publisher.publish(joints_cmd_msg);
    ROS_INFO("point.positions : %f, %f, %f", point.positions[0], point.positions[1], point.positions[2]);
}



std::vector<float> Leg1Project::solveInverseKinematics(std::vector<float> position_info)
{
	std::vector<float> joint_state;
	float theta1, theta2, theta3;
	float link1 = 78.96;
	float link2 = 187.96;
	float link3 = 200.00;

	float x = position_info[0];
	float y = position_info[1];
	float z = position_info[2];

	float F = sqrt(z*z + y * y - 78.0 * 78.0);
	float G = 12.3 + sqrt(z*z + y * y - 78.0 * 78.0);
	float H = sqrt(G*G + x * x);
    float alpa = asin(x/(sqrt(x*x+G*G)));

	theta1 = atan2(F, 78.0) - atan2(abs(z), abs(y));

	float D = -(H*H-link2*link2-link3*link3)/(2*link2*link3);

	theta3 = acos(D) - 3.14/2 - 0.1603;
	float alpha, beta;
	//alpha = atan2(x, G);
    alpha = atan2(G, x);
	beta = atan(link3*sin(3.14 / 2 - 0.1603 - theta3) / (link2 + link3 * cos(3.14 / 2 - 0.1603 - theta3)));
    
	//theta2 = 3.14 /2  - alpha - beta - 0.1603;
    theta2 = 3.14 - alpha - beta - 0.1603;

	joint_state_ = { theta1, theta2, theta3 };
    ROS_INFO( "joint state : %f, %f, %f", theta1, theta2, theta3 );

    return joint_state_;
}

void Leg1Project::straightLineStanceTrajectory(double duration_)
{

    double duration =  duration_;

    float x,y,z;
	std::vector <float> start_point = {-185.55, 78.00, -215.7};
    std::vector <float> end_point = {185.55, 78.00, -215.7};
    std::vector <float> position_info;

    float xs = start_point[0];
    float ys = start_point[1];
    float zs = start_point[2];

    float xe = end_point[0];
    float ye = end_point[1];
    float ze = end_point[2];

    x = xs + (time_/300)*(xe-xs);
    y = ys + (time_/300)*(ye-ys);
    z = zs + (time_/300)*(ze-zs);
    time_ += 1.0;
    // 300 means that trajectory time are 3s - TODO Change this 

    target_joint_position = solveInverseKinematics({x,y,z});
    if(time_ == 300.0)
    {
        start_point = {185.55, 78.00, -215.7};
        end_point = {-185.55, 78.00, -215.7};
        time_= 0;
    }
    ROS_INFO("time_step : %f", time_);
}
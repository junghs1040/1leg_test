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
    joint_command_publisher = nh->advertise<trajectory_msgs::JointTrajectory>("joint_group_position_controller/command", 1);

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
    point.positions.resize(12);

    point.time_from_start = ros::Duration(1.0/60.0);
    for (int i=0; i<12; i++)
    {
        point.positions[i] = target_joint_position[i];
    }

    joints_cmd_msg.points.push_back(point);
    joint_command_publisher.publish(joints_cmd_msg);
}



std::vector<double> Leg1Project::solveInverseKinematics(std::vector<double> position_info)
{
	std::vector<double> joint_state;
	double theta1, theta2, theta3;
	double link1 = 78.96;
	double link2 = 187.96;
	double link3 = 200.00;

	double x = position_info[0];
	double y = position_info[1];
	double z = position_info[2];

	double F = sqrt(z*z + y * y - 78.0 * 78.0);
	double G = 12.3 + sqrt(z*z + y * y - 78.0 * 78.0);
	double H = sqrt(G*G + x * x);
    double alpa = asin(x/(sqrt(x*x+G*G)));

	theta1 = atan2(F, 78.0) - atan2(abs(z), abs(y));

	double D = -(H*H-link2*link2-link3*link3)/(2*link2*link3);

	theta3 = acos(D) - 3.14/2 - 0.1603;
	double alpha, beta;
	//alpha = atan2(x, G);
    alpha = atan2(G, x);
	beta = atan(link3*sin(3.14 / 2 - 0.1603 - theta3) / (link2 + link3 * cos(3.14 / 2 - 0.1603 - theta3)));
    
	//theta2 = 3.14 /2  - alpha - beta - 0.1603;
    theta2 = 3.14 - alpha - beta - 0.1603;

	joint_state_ = { theta1, theta2, theta3 };

    return joint_state_;
}

void Leg1Project::straightLineStanceTrajectory(double duration_)
{

    double duration =  duration_;
    double time_ = 0;
    double x,y,z;
    std::vector <double> start_point = {0.0,0.0,0.0};
    std::vector <double> end_point = {0.0,0.0,0.0};
    std::vector <double> position_info;

    double xs = start_point[0];
    double ys = start_point[1];
    double zs = start_point[2];

    double xe = end_point[0];
    double ye = end_point[1];
    double ze = end_point[2];

    x = xs + (time_/300)*(xe-xs);
    y = ys + (time_/300)*(ye-ys);
    z = zs + (time_/300)*(ze-zs);
    time_ += 1;

    solveInverseKinematics({x,y,z});
}
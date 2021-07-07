#include <iostream>
#include "leg1_project/IK_Solver.hpp"

using namespace IK_Solver;

std::vector<double> Robot::Straight_Line_Trajectory(double time)
{
	double time_ = time;
	std::vector<double> final_value;
    std::vector<double> start_point;
	std::vector<double> end_point;
    std::vector<double> position;
	double position1, position2, position3;
	start_point = {-185.55, 78.00, -215.7};
    end_point = {185.55, 78.00, -215.7};

    double xs = start_point[0];
    double ys = start_point[1];
    double zs = start_point[2];

    double xe = end_point[0];
    double ye = end_point[1];
    double ze = end_point[2];

    
    position1 = xs + (time_/100.0)*(xe-xs);
    position2 = ys + (time_/100.0)*(ye-ys);
    position3 = zs + (time_/100.0)*(ze-zs);
	position = {position1,position2,position3};
    
    return position;
}

std::vector<double> Robot::Cubic_Bezier_Curve_Trajectory(double time)
{
	double time_ = time;
	double t = time_/100.0;
	std::vector<double> final_value;
    std::vector<double> P0, P1, P2, P3;
	
    std::vector<double> position;
	double position1, position2, position3;
	P3 = { 120, 78.00, -293.0};
	P2 = { 185.55, 78.00, -180.0};
	P1 = { 0, 78.00, -180.0};
    P0 = { -120.0, 78.00, -293.0};

    double x0 = P0[0];
    double y0 = P0[1];
    double z0 = P0[2];

    double x1 = P1[0];
    double y1 = P1[1];
    double z1 = P1[2];

	double x2 = P2[0];
    double y2 = P2[1];
    double z2 = P2[2];

	double x3 = P3[0];
    double y3 = P3[1];
    double z3 = P3[2];

    
    position1 = x0*pow((1-t),3)+3*pow((1-t),2)*t*x1+3*(1-t)*pow(t,2)*x2+pow(t,3)*x3;
    position2 = y0*pow((1-t),3)+3*pow((1-t),2)*t*y1+3*(1-t)*pow(t,2)*y2+pow(t,3)*y3;
    position3 = z0*pow((1-t),3)+3*pow((1-t),2)*t*z1+3*(1-t)*pow(t,2)*z2+pow(t,3)*z3;
	position = {position1,position2,position3};
    
    return position;
}

std::vector<double> Robot::Leg_IK(std::vector<double> final_value)
{

	std::vector<double> joint_state_;
	double theta1, theta2, theta3;
	double link1 = 78.96;
	double link2 = 187.96;
	double link3 = 200.00;

	double x = final_value[0];
	double y = final_value[1];
	double z = final_value[2];

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
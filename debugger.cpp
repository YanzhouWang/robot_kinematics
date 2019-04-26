#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include "kinematics_calculator.h"
#include "arm.h"

double g_x0{65}, g_y0{0}, g_z0{292};
double g_l1{155}, g_l2{150}, g_l3{10};
double g_theta_0{0.3};
double g_joint45_z_offset{0};

int main(int argc, char const *argv[])
{
	/*LEFT ARM BEGIN*/
	//direction and position of each revolute joint
	Eigen::Vector3d l_w1{0, 0, -1}, l_p1{g_x0, g_y0, g_z0}, l_w2{0, 1, 0}, l_p2{g_x0, g_y0, g_z0}, l_w3{1, 0, 0}, l_p3{g_x0, g_y0, g_z0}, l_w4{0, 0, -1}, l_p4{g_x0 + g_l1, g_y0, g_z0 + g_joint45_z_offset}, l_w5{cos(g_theta_0), -sin(g_theta_0), 0}, l_p5{g_x0 + g_l1, g_y0, g_z0 + g_joint45_z_offset}, l_w6{sin(g_theta_0), cos(g_theta_0), 0}, l_p6{g_x0 + g_l1 + g_l2 * cos(g_theta_0), g_y0 - g_l2 * sin(g_theta_0), g_z0 + g_joint45_z_offset};
	//tool frame location
	std::vector<double> l_offsets{g_x0 + g_l1 + (g_l2 + g_l3)*cos(g_theta_0), g_y0 - (g_l2 + g_l3)*sin(g_theta_0), g_z0 + g_joint45_z_offset};
	//tool frame orientation
	Eigen::Vector3d l_tool_Rot_axis{0, 0, -1}; //in this case the tool orientation is caused by the same pre-rotation. could be different in other scenarios
	double l_tool_Rot_angle = g_theta_0;
	//robot arm construction at zero configuration
	ARM	left_arm(l_w1, l_p1, l_w2, l_p2, l_w3, l_p3, l_w4, l_p4, l_w5, l_p5, l_w6, l_p6, l_offsets, l_tool_Rot_axis, l_tool_Rot_angle);
	/*LEFT ARM FINISH*/
	std::vector<vector<double>> joint_limits = left_arm.getJointLimits();
	double i = joint_limits[0][0];
	left_arm.changeAngle(0,i);
	left_arm.getAngles();
	return 0;
}
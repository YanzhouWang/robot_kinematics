#ifndef ARM_CLASS_H
#define ARM_CLASS_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <fstream>
#include <ctime>
#include "kinematics_calculator.h"

class ARM
{
public:
	//construct and fill in information for a 6-DOF robot manipulator
	ARM(Eigen::Vector3d w1, Eigen::Vector3d p1, Eigen::Vector3d w2, Eigen::Vector3d p2, Eigen::Vector3d w3, Eigen::Vector3d p3, Eigen::Vector3d w4, Eigen::Vector3d p4, Eigen::Vector3d w5, Eigen::Vector3d p5, Eigen::Vector3d w6, Eigen::Vector3d p6, vector<double> tool_offsets);
	ARM(Eigen::Vector3d w1, Eigen::Vector3d p1, Eigen::Vector3d w2, Eigen::Vector3d p2, Eigen::Vector3d w3, Eigen::Vector3d p3, Eigen::Vector3d w4, Eigen::Vector3d p4, Eigen::Vector3d w5, Eigen::Vector3d p5, Eigen::Vector3d w6, Eigen::Vector3d p6, vector<double> tool_offsets, Eigen::Vector3d tool_Rot_axis, double tool_Rot_angle);
	//change a joint coordinate (either a w, or a p)
	void changeCoordinate(int index, Eigen::Vector3d new_coord);
	//change a joint angle (default angles are 0)
	void changeAngle(int index, double new_angle);
	//show current joint angles
	void getAngles();
	//get result of whether tool frame is aligned with world frame
	bool getAlignment();
	//calculate and display current FK of tool frame based on tool alignment result
	Matrix4d calcArmFK();
	//calculate current spatial Jacobian
	JacobianMatrix calcSpatialJacobian();
	//calculate current body Jacobian
	JacobianMatrix calcBodyJacobian();
	//set upper and lower joint values for a specific joint
	void setJointLimits(int jointNum, double lower_value, double upper_value);
	//get and show upper and lower joint values for all joints
	vector<vector<double>> getJointLimits();
	//takes in lower-, and upper-limit for of joint angles for each joint, then iteratively check for singular regions
	//can change default increment and tolerance level based on needs
	//writes output to a file
	void findSpatialSingularity(double increment = 0.5, double tolerance = 0.001);
	void findBodySingularity(double increment = 0.5, double tolerance = 0.001);
	//runs through joint limits and output end-effector coordinates to a text file
	void findWorkSpace(double increment = 0.5);
private:
	kinematics_calculator calculator;
	Eigen::Vector3d c_w1, c_p1, c_w2, c_p2, c_w3, c_p3, c_w4, c_p4, c_w5, c_p5, c_w6, c_p6;
	vector<double> c_tool_offsets;
	double c_theta1{0}, c_theta2{0}, c_theta3{0}, c_theta4{0}, c_theta5{0}, c_theta6{0}; //assuming all initial angles are 0 wrt zero configuration
	vector<Eigen::Vector3d> c_coordinates;
	vector<double>c_thetas;
	//for cases where tool frame is not perfectly aligned with world frame in zero configuration
	Eigen::Vector3d c_tool_Rot_axis{0, 0, 0};
	double c_tool_Rot_angle = 0;
	bool c_check_tool_alignment = true; //a flag to indicate if the tool frame and world frame are aligned

	//joint limits
	vector<double> c_Joint1Lmts{ -0.8, 1.48}, c_Joint2Lmts{ -1.507, 1.507}, c_Joint3Lmts{ -1.507, 1.507}, c_Joint4Lmts{ 0.3, 2.269}, c_Joint5Lmts{ -1.507, 1.507}, c_Joint6Lmts{ -1.507, 1.507};
	vector<vector<double>> c_JointLimits{c_Joint1Lmts, c_Joint2Lmts, c_Joint3Lmts, c_Joint4Lmts, c_Joint5Lmts, c_Joint6Lmts};
};

#endif
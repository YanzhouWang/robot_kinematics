#ifndef KINEMATICS_CALCULATOR_CLASS_H
#define KINEMATICS_CALCULATOR_CLASS_H

#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<double, 6, Eigen::Dynamic> JacobianMatrix;
typedef Eigen::Matrix<double, 6, 6> AdjointMatrix;

//Note: the following functions do not work with prismatic joints (yets)
class kinematics_calculator
{
public:
	kinematics_calculator();

	Vector6d calcTwist(Eigen::Vector3d w, Eigen::Vector3d p);
	Eigen::Matrix3d calcVectorHat(Eigen::Vector3d v);
	Eigen::Matrix3d calcRotationfromTwist(Vector6d zeta, double theta);
	Matrix4d calcTransformationfromTwist(Vector6d zeta, double theta);
	Matrix4d calcInitialConfig(vector<double> tool_offsets);
	Matrix4d calcInitialConfig(vector<double> tool_offsets, Eigen::Vector3d axis, double theta);
	vector<Vector6d> assembleTwists(vector<Eigen::Vector3d> coordinates);
	Matrix4d calcProductofExp(vector<Vector6d> zetas, vector<double> thetas);
	Matrix4d calcFK(vector<Eigen::Vector3d> coordinates, vector<double> thetas, vector<double> tool_offsets);//for tool perfectly aligned with world frame
	Matrix4d calcFK(vector<Eigen::Vector3d> coordinates, vector<double> thetas, vector<double> tool_offsets, Eigen::Vector3d tool_Rot_axis, double tool_Rot_angle);//for tool not aligned with world frame

private:
};

#endif
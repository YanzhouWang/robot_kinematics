#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include "kinematics_calculator.h"
#include "arm.h"
#include <cmath>

using namespace std;
//All values in mm or rad
double g_x0{65}, g_y0{0}, g_z0{292};
double g_l1{155}, g_l2{150}, g_l3{10};
double g_theta_0{0.3};
double g_joint45_z_offset{0};

void showMenu();

int main(int argc, char const *argv[])
{
	/*LEFT ARM BEGIN*/
	//direction and position of each revolute joint
	Eigen::Vector3d l_w1{0, 0, -1}, l_p1{g_x0, g_y0, g_z0}, l_w2{0, 1, 0}, l_p2{g_x0, g_y0, g_z0}, l_w3{1, 0, 0}, l_p3{g_x0, g_y0, g_z0}, l_w4{0, 0, -1}, l_p4{g_x0 + g_l1, g_y0, g_z0 + g_joint45_z_offset}, l_w5{cos(g_theta_0), -sin(g_theta_0), 0}, l_p5{g_x0 + g_l1, g_y0, g_z0 + g_joint45_z_offset}, l_w6{sin(g_theta_0), cos(g_theta_0), 0}, l_p6{g_x0 + g_l1 + g_l2 * cos(g_theta_0), g_y0 - g_l2 * sin(g_theta_0), g_z0 + g_joint45_z_offset};
	//tool frame location
	vector<double> l_offsets{g_x0 + g_l1 + (g_l2 + g_l3)*cos(g_theta_0), g_y0 - (g_l2 + g_l3)*sin(g_theta_0), g_z0 + g_joint45_z_offset};
	//tool frame orientation
	Eigen::Vector3d l_tool_Rot_axis{0, 0, -1}; //in this case the tool orientation is caused by the same pre-rotation. could be different in other scenarios
	double l_tool_Rot_angle = g_theta_0;
	//robot arm construction at zero configuration
	ARM	left_arm(l_w1, l_p1, l_w2, l_p2, l_w3, l_p3, l_w4, l_p4, l_w5, l_p5, l_w6, l_p6, l_offsets, l_tool_Rot_axis, l_tool_Rot_angle);
	/*LEFT ARM FINISH*/


	/*RIGHT ARM BEGIN*/
	//direction and position of each revolute joint
	Eigen::Vector3d r_w1{0, 0, 1}, r_p1{ -g_x0, g_y0, g_z0}, r_w2{0, 1, 0}, r_p2{ -g_x0, g_y0, g_z0}, r_w3{ -1, 0, 0}, r_p3{ -g_x0, g_y0, g_z0}, r_w4{0, 0, 1}, r_p4{ -(g_x0 + g_l1), g_y0, g_z0 + g_joint45_z_offset}, r_w5{ -cos(g_theta_0), -sin(g_theta_0), 0}, r_p5{ -(g_x0 + g_l1), g_y0, g_z0 + g_joint45_z_offset}, r_w6{ -sin(g_theta_0), cos(g_theta_0), 0}, r_p6{ -(g_x0 + g_l1 + g_l2 * cos(g_theta_0)), g_y0 - g_l2 * sin(g_theta_0), g_z0 + g_joint45_z_offset};
	//tool frame location
	vector<double> r_offsets{ -(g_x0 + g_l1 + (g_l2 + g_l3)*cos(g_theta_0)), g_y0 - (g_l2 + g_l3)*sin(g_theta_0), g_z0 + g_joint45_z_offset};
	//tool frame orientation
	Eigen::Vector3d r_tool_Rot_axis{0, 0, 1};
	double r_tool_Rot_angle = g_theta_0;
	//robot arm construction at zero configuration
	ARM right_arm(r_w1, r_p1, r_w2, r_p2, r_w3, r_p3, r_w4, r_p4, r_w5, r_p5, r_w6, r_p6, r_offsets, r_tool_Rot_axis, r_tool_Rot_angle);
	/*RIGHT ARM FINISH*/



	//real functionalities
	cout << "Robot left and right arms are constructed." << endl;
	cout << "Default joint angles are uniformly 0." << endl;
	bool quit = false, change_arm = false;
	int left_or_right = 0; //0 for left arm, 1 for right arm
	int choice = 0, index = 0;//placeholder variables
	double coord1, coord2, coord3;//placeholder variables
	Eigen::Vector3d TDVec;//placeholder variables

	while (!quit) {
		cout << "Left (1) or Right (2) arm, 0 to quit: ";
		cin >> left_or_right;
		change_arm = false;

		if (left_or_right == 1)
		{
			cout << "Left arm selected. Please use the menu to select the respective function." << endl;
			while (change_arm == false) {
				showMenu();
				cin >> choice;
				switch (choice) {
				case 1: {
					cout << "Choose the coordinate index (0~11. Even number for direction w, odd number for position p): ";
					cin >> index;
					if (index >= 0 && index <= 11)
					{
						cout << "Enter 3 new coordinate values: ";
						cin >> coord1 >> coord2 >> coord3;
						if (index % 2 == 0)
						{
							double norm = sqrt(pow(coord1, 2) + pow(coord2, 2) + pow(coord3, 2));
							coord1 /= norm;
							coord2 /= norm;
							coord3 /= norm;
						}
						TDVec << coord1, coord2, coord3;
						left_arm.changeCoordinate(index, TDVec);
					}
					else {
						cout << "Selection Invalid." << endl;;
					}
					break;
				}
				case 2: {
					cout << "Choose the angle index (0~5): ";
					cin >> index;
					if (index >= 0 && index <= 6)
					{
						cout << "Enter the new angle for joint " << index << " in rad (regardless of joint limits): ";
						cin >> coord1;
						left_arm.changeAngle(index, coord1);
						left_arm.getAngles();
					}
					else {
						cout << "Selection Invalid." << endl;
					}
					break;
				}
				case 3: {
					cout << left_arm.calcArmFK() << endl;
					break;
				}
				case 4: {
					cout << left_arm.calcSpatialJacobian() << endl;
					break;
				}
				case 5: {
					cout << left_arm.calcBodyJacobian() << endl;
					break;
				}
				case 6: {
					left_arm.getJointLimits();
					break;
				}
				case 7: {
					cout << "Choose the joint index (0~5): ";
					cin >> index;
					if (index >= 0 && index <= 6)
					{
						cout << "Enter new lower and upper limits for joint " << index << " in rad: ";
						cin >> coord1 >> coord2;
						if (coord1 >= coord2)
						{
							cout << "The numbers you entered do not correspond to valid lower and upper limit values." << endl;
						}
						else {
							left_arm.setJointLimits(index, coord1, coord2);
						}
					}
					else {
						cout << "Selection Invalid." << endl;
					}
					break;
				}
				case 8: {
					cout << "Please put in desired increment and tolerance values (default: 0.5 and 0.01). Enter 0 0 to use default values: ";
					cin >> coord1 >> coord2;
					if (coord1 == 0 && coord2 == 0)
					{
						left_arm.findSpatialSingularity();
					}
					else {
						left_arm.findSpatialSingularity(coord1, coord2);
					}
					break;
				}
				case 9: {
					cout << "Please put in desired increment and tolerance values (default: 0.5 and 0.01). Enter 0 0 to use default values: ";
					cin >> coord1 >> coord2;
					if (coord1 == 0 && coord2 == 0)
					{
						left_arm.findBodySingularity();
					}
					else {
						left_arm.findBodySingularity(coord1, coord2);
					}
					break;
				}
				case 10: {
					left_arm.findWorkSpace();
				}
				case 11: change_arm = true; break;
				}

			}


		}
		else if (left_or_right == 2)
		{
			cout << "Right arm selected. Please use the menu to select the respective function." << endl;
			while (change_arm == false) {
				showMenu();
				cin >> choice;
				switch (choice) {
				case 1: {
					cout << "Choose the coordinate index (0~11. Even number for direction w, odd number for position p): ";
					cin >> index;
					if (index >= 0 && index <= 11)
					{
						cout << "Enter 3 new coordinate values: ";
						cin >> coord1 >> coord2 >> coord3;
						if (index % 2 == 0)
						{
							double norm = sqrt(pow(coord1, 2) + pow(coord2, 2) + pow(coord3, 2));
							coord1 /= norm;
							coord2 /= norm;
							coord3 /= norm;
						}
						TDVec << coord1, coord2, coord3;
						right_arm.changeCoordinate(index, TDVec);
					}
					else {
						cout << "Selection Invalid." << endl;;
					}
					break;
				}
				case 2: {
					cout << "Choose the angle index (0~5): ";
					cin >> index;
					if (index >= 0 && index <= 6)
					{
						cout << "Enter the new angle for joint " << index << " in rad (regardless of joint limits): ";
						cin >> coord1;
						right_arm.changeAngle(index, coord1);
						right_arm.getAngles();
					}
					else {
						cout << "Selection Invalid." << endl;
					}
					break;
				}
				case 3: {
					cout << right_arm.calcArmFK() << endl;
					break;
				}
				case 4: {
					cout << right_arm.calcSpatialJacobian() << endl;
					break;
				}
				case 5: {
					cout << right_arm.calcBodyJacobian() << endl;
					break;
				}
				case 6: {
					right_arm.getJointLimits();
					break;
				}
				case 7: {
					cout << "Choose the joint index (0~5): ";
					cin >> index;
					if (index >= 0 && index <= 6)
					{
						cout << "Enter new lower and upper limits for joint " << index << " in rad: ";
						cin >> coord1 >> coord2;
						if (coord1 >= coord2)
						{
							cout << "The numbers you entered do not correspond to valid lower and upper limit values." << endl;
						}
						else {
							right_arm.setJointLimits(index, coord1, coord2);
						}
					}
					else {
						cout << "Selection Invalid." << endl;
					}
					break;
				}
				case 8: {
					cout << "Please put in desired increment and tolerance values (default: 0.5 and 0.01). Enter 0 0 to use default values: ";
					cin >> coord1 >> coord2;
					if (coord1 == 0 && coord2 == 0)
					{
						right_arm.findSpatialSingularity();
					}
					else {
						right_arm.findSpatialSingularity(coord1, coord2);
					}
					break;
				}
				case 9: {
					cout << "Please put in desired increment and tolerance values (default: 0.5 and 0.01). Enter 0 0 to use default values: ";
					cin >> coord1 >> coord2;
					if (coord1 == 0 && coord2 == 0)
					{
						right_arm.findBodySingularity();
					}
					else {
						right_arm.findBodySingularity(coord1, coord2);
					}
					break;
				}
				case 10: {
					right_arm.findWorkSpace();
				}
				case 11: change_arm = true; break;
				}
			}

		}

		else if (left_or_right == 0) {
			quit = true;
		}
	}

	return 0;
}

void showMenu() {
	cout << "1. Change twist coordinate (either a direction or a position)" << endl;
	cout << "2. Change joint angle" << endl;
	cout << "3. Calculate current forward kinematics" << endl;
	cout << "4. Calculate current spatial jacobian" << endl;
	cout << "5. Calculate current body jacobian" << endl;
	cout << "6. Show joint limits" << endl;
	cout << "7. Change joint limits" << endl;
	cout << "8. Find spatial singularities" << endl;
	cout << "9. Find body singularities" << endl;
	cout << "10. Find workspace" << endl;
	cout << "11. Change arm selection" << endl;
}
#include "arm.h"
#include "kinematics_calculator.h"

ARM::ARM(Eigen::Vector3d w1, Eigen::Vector3d p1, Eigen::Vector3d w2, Eigen::Vector3d p2, Eigen::Vector3d w3, Eigen::Vector3d p3, Eigen::Vector3d w4, Eigen::Vector3d p4, Eigen::Vector3d w5, Eigen::Vector3d p5, Eigen::Vector3d w6, Eigen::Vector3d p6, vector<double> tool_offsets) {
	/*cout << "In ARM constructor." << endl;*/
	c_w1 << w1; c_p1 << p1; c_w2 << w2; c_p2 << p2; c_w3 << w3; c_p3 << p3; c_w4 << w4; c_p4 << p4; c_w5 << w5; c_p5 << p5; c_w6 << w6; c_p6 << p6;
	c_coordinates.push_back(c_w1);
	c_coordinates.push_back(c_p1);
	c_coordinates.push_back(c_w2);
	c_coordinates.push_back(c_p2);
	c_coordinates.push_back(c_w3);
	c_coordinates.push_back(c_p3);
	c_coordinates.push_back(c_w4);
	c_coordinates.push_back(c_p4);
	c_coordinates.push_back(c_w5);
	c_coordinates.push_back(c_p5);
	c_coordinates.push_back(c_w6);
	c_coordinates.push_back(c_p6);
	c_thetas.push_back(c_theta1);
	c_thetas.push_back(c_theta2);
	c_thetas.push_back(c_theta3);
	c_thetas.push_back(c_theta4);
	c_thetas.push_back(c_theta5);
	c_thetas.push_back(c_theta6);
	c_tool_offsets.push_back(tool_offsets[0]);
	c_tool_offsets.push_back(tool_offsets[1]);
	c_tool_offsets.push_back(tool_offsets[2]);

	c_check_tool_alignment = true;
};

ARM::ARM(Eigen::Vector3d w1, Eigen::Vector3d p1, Eigen::Vector3d w2, Eigen::Vector3d p2, Eigen::Vector3d w3, Eigen::Vector3d p3, Eigen::Vector3d w4, Eigen::Vector3d p4, Eigen::Vector3d w5, Eigen::Vector3d p5, Eigen::Vector3d w6, Eigen::Vector3d p6, vector<double> tool_offsets, Eigen::Vector3d tool_Rot_axis, double tool_Rot_angle) {
	/*cout << "In ARM constructor." << endl;*/
	c_w1 << w1; c_p1 << p1; c_w2 << w2; c_p2 << p2; c_w3 << w3; c_p3 << p3; c_w4 << w4; c_p4 << p4; c_w5 << w5; c_p5 << p5; c_w6 << w6; c_p6 << p6;
	c_coordinates.push_back(c_w1);
	c_coordinates.push_back(c_p1);
	c_coordinates.push_back(c_w2);
	c_coordinates.push_back(c_p2);
	c_coordinates.push_back(c_w3);
	c_coordinates.push_back(c_p3);
	c_coordinates.push_back(c_w4);
	c_coordinates.push_back(c_p4);
	c_coordinates.push_back(c_w5);
	c_coordinates.push_back(c_p5);
	c_coordinates.push_back(c_w6);
	c_coordinates.push_back(c_p6);
	c_thetas.push_back(c_theta1);
	c_thetas.push_back(c_theta2);
	c_thetas.push_back(c_theta3);
	c_thetas.push_back(c_theta4);
	c_thetas.push_back(c_theta5);
	c_thetas.push_back(c_theta6);
	c_tool_offsets.push_back(tool_offsets[0]);
	c_tool_offsets.push_back(tool_offsets[1]);
	c_tool_offsets.push_back(tool_offsets[2]);


	c_tool_Rot_axis = tool_Rot_axis;
	c_tool_Rot_angle = tool_Rot_angle;

	c_check_tool_alignment = false;
};

void ARM::changeCoordinate(int index, Eigen::Vector3d new_coord) {
	Eigen::Vector3d old_coord = c_coordinates[index];
	c_coordinates[index] = new_coord;
	cout << "Old coordinate:" << endl << old_coord << endl << " is changed to:" << endl << c_coordinates[index] << endl;
}


void ARM::changeAngle(int index, double new_angle) {
	/*double old_angle = c_thetas[index];*/
	c_thetas[index] = new_angle;
	/*cout << "Old angle " << old_angle << "(rad) is changed to " << c_thetas[index] << "(rad)" << endl;
	cout << "New set of angles: " << endl;
	*/
}

void ARM::getAngles() {
	for (int i = 0; i < 6; ++i)
	{
		cout << "Joint " << i << " at " << c_thetas[i] << "(rad)" << endl;
	}
}

bool ARM::getAlignment() {
	return c_check_tool_alignment;
}

Matrix4d ARM::calcArmFK() {
	/*cout << "Tool frame aligned: " << c_check_tool_alignment << endl;
	cout << "Showing FK" << endl;*/
	if (c_check_tool_alignment == true)
	{
		return (calculator.calcFK(c_coordinates, c_thetas, c_tool_offsets));
	}
	else {
		return (calculator.calcFK(c_coordinates, c_thetas, c_tool_offsets, c_tool_Rot_axis, c_tool_Rot_angle));
	}
}

JacobianMatrix ARM::calcSpatialJacobian() {
	int num_coordinates = c_coordinates.size();
	int num_thetas = c_thetas.size();

	if (num_coordinates % 2 != 0)
	{
		cout << "Number of 3x1 coordinate vectors are not even." << endl;
	}

	else if ((num_coordinates / 2) > (num_thetas + 1))
	{
		cout << "Need more angle inputs." << endl;
	}

	else {
		int num_redundant_angles = num_coordinates / 2 - num_thetas;
		if (num_redundant_angles > 1)
		{
			cout << "More than 1 extra angles in argument, ignoring the last " << num_redundant_angles << " angle inputs." << endl;
		}

		//construct the first element of the Jacobian
		JacobianMatrix Jacobian;
		AdjointMatrix Adjoint;

		vector<Vector6d> twists_holder{calculator.assembleTwists(c_coordinates)};

		Jacobian.resize(6, 1);
		Jacobian.col(0) << twists_holder[0];
		//construct the rest of the Jacobian
		Matrix4d g_tot, g_current;
		g_tot = calculator.calcTransformationfromTwist(twists_holder[0], c_thetas[0]); //generate g1
		Eigen::Matrix3d Rot;
		Eigen::Vector3d Dis;

		for (int i = 2, j = 1; i < num_coordinates; i += 2, j += 1)
		{
			//calculating zeta_primes
			Rot << g_tot.topLeftCorner(3, 3);
			Dis << g_tot.topRightCorner(3, 1);
			Adjoint.topLeftCorner(3, 3) << Rot;
			Adjoint.topRightCorner(3, 3) << (calculator.calcVectorHat(Dis)*Rot);
			Adjoint.bottomLeftCorner(3, 3) << Eigen::Matrix3d::Zero();
			Adjoint.bottomRightCorner(3, 3) << Rot;
			Jacobian.conservativeResize(6, j + 1);
			Jacobian.col(j) << Adjoint*twists_holder[j];
			//calculating new g's
			g_current = calculator.calcTransformationfromTwist(twists_holder[j], c_thetas[j]);
			//compounding transformations
			g_tot *= g_current;
		}
		return Jacobian;
	}
}

JacobianMatrix ARM::calcBodyJacobian() {
	JacobianMatrix Jacobian;
	AdjointMatrix Adjoint;
	Jacobian.resize(6, 6);
	vector<Vector6d> twist_holder{calculator.assembleTwists(c_coordinates)};
	Matrix4d g_tot, g_part;
	g_tot = calcArmFK();
	Eigen::Matrix3d Rot;
	Eigen::Vector3d Dis;
	int num_thetas = c_thetas.size();

	for (int i = 0; i < num_thetas; ++i)
	{
		Rot << g_tot.topLeftCorner(3, 3);
		Dis << g_tot.topRightCorner(3, 1);
		Adjoint.topLeftCorner(3, 3) << Rot.transpose();
		Adjoint.topRightCorner(3, 3) << (-Rot.transpose()*calculator.calcVectorHat(Dis));
		Adjoint.bottomLeftCorner(3, 3) << Eigen::Matrix3d::Zero();
		Adjoint.bottomRightCorner(3, 3) << Rot.transpose();
		Jacobian.col(i) << Adjoint*twist_holder[i];

		g_part = calculator.calcTransformationfromTwist(twist_holder[i], c_thetas[i]); //get e_i
		Rot << g_part.topLeftCorner(3, 3);
		Dis << g_part.topRightCorner(3, 1);
		g_part.topLeftCorner(3, 3) << Rot.transpose();
		g_part.topRightCorner(3, 1) << -Rot.transpose()*Dis; //get e_-i

		g_tot = g_part * g_tot;
	}
	return (Jacobian);
}

void ARM::setJointLimits(int jointNum, double lower_value, double upper_value) {
	double old_lower_value = c_JointLimits[jointNum][0];
	double old_upper_value = c_JointLimits[jointNum][1];
	if (3.1415 < lower_value || lower_value < -3.1415)
	{
		cout << "ERROR: Lower value out of range (-3.1415~3.1415)" << endl;
	}
	else if (3.1415 < upper_value || upper_value < -3.1415)
	{
		cout << "ERROR: Upper value out of range (-3.1415~3.1415)" << endl;
	}
	else if (lower_value > upper_value)
	{
		cout << "ERROR: Lower value greater than Upper value" << endl;
	}
	else {
		c_JointLimits[jointNum][0] = lower_value;
		c_JointLimits[jointNum][1] = upper_value;
		cout << "Joint " << jointNum << "'s value changed from" << endl;
		cout << "Lower: " << old_lower_value << " Upper: " << old_upper_value << " to" << endl;
		cout << "Lower: " << lower_value << " Upper: " << upper_value << endl;
	}
}

vector<vector<double>> ARM::getJointLimits() {
	cout << "Showing joint limits" << endl;
	for (int i = 0; i < 6; ++i)
	{
		cout << "Joint " << i << " L: " << c_JointLimits[i][0] << " U: " << c_JointLimits[i][1] << endl;
	}
	return c_JointLimits;
}


void ARM::findSpatialSingularity(double increment, double tolerance) {
	ofstream fileHandle;
	fileHandle.open("Spatial_Singularity_Regions.txt");
	time_t now = time(0);
	char* dt = ctime(&now);
	fileHandle << "Time stamp: " << dt << endl;
	cout << "Trying to find singularities. This might take a long time." << endl;
	JacobianMatrix J;
	vector<vector<double>> joint_limits = getJointLimits();
	double det;
	for (double i = joint_limits[0][0]; i < joint_limits[0][1]; i += increment)
	{
		changeAngle(0, i);
		for (double j = joint_limits[1][0]; j < joint_limits[1][1]; j += increment)
		{
			changeAngle(1, j);
			for (double k = joint_limits[2][0]; k < joint_limits[2][1]; k += increment)
			{
				changeAngle(2, k);
				for (double l = joint_limits[3][0]; l < joint_limits[3][1]; l += increment)
				{
					changeAngle(3, l);
					for (double m = joint_limits[4][0]; m < joint_limits[4][1]; m += increment)
					{
						changeAngle(4, m);
						for (double n = joint_limits[5][0]; n < joint_limits[5][1]; n += increment)
						{
							changeAngle(5, n);
							J = calcSpatialJacobian();
							det = J.determinant();
							if (det <= tolerance && det >= -tolerance)
							{
								cout << det << endl;
								cout << "CLOSE TO A SINGULARITY: " << i << " " << j << " " << k << " " << l << " " << m << " " << n << endl;
								fileHandle << det << " " << i << " " << j << " " << k << " " << l << " " << m << " " << n << endl;
							}
						}
					}
				}
			}
		}
	}
	fileHandle.close();
	cout << "DONE!" << endl;

}

void ARM::findBodySingularity(double increment, double tolerance) {
	ofstream fileHandle;
	fileHandle.open("Body_Singularity_Regions.txt");
	time_t now = time(0);
	char* dt = ctime(&now);
	fileHandle << "Time stamp: " << dt << endl;
	cout << "Trying to find singularities. This might take a long time." << endl;
	JacobianMatrix J;
	vector<vector<double>> joint_limits = getJointLimits();

	double det;
	for (double i = joint_limits[0][0]; i < joint_limits[0][1]; i += increment)
	{
		changeAngle(0, i);
		for (double j = joint_limits[1][0]; j < joint_limits[1][1]; j += increment)
		{
			changeAngle(1, j);
			for (double k = joint_limits[2][0]; k < joint_limits[2][1]; k += increment)
			{
				changeAngle(2, k);
				for (double l = joint_limits[3][0]; l < joint_limits[3][1]; l += increment)
				{
					changeAngle(3, l);
					for (double m = joint_limits[4][0]; m < joint_limits[4][1]; m += increment)
					{
						changeAngle(4, m);
						for (double n = joint_limits[5][0]; n < joint_limits[5][1]; n += increment)
						{
							changeAngle(5, n);
							J = calcBodyJacobian();
							det = J.determinant();
							if (det <= tolerance && det >= -tolerance)
							{
								cout << det << endl;
								cout << "CLOSE TO A SINGULARITY: " << i << " " << j << " " << k << " " << l << " " << m << " " << n << endl;
								fileHandle << det << " " << i << " " << j << " " << k << " " << l << " " << m << " " << n << endl;
							}
						}
					}
				}
			}
		}
	}
	fileHandle.close();
	cout << "DONE!" << endl;

}

void ARM::findWorkSpace(double increment) {
	ofstream fileHandle;
	fileHandle.open("Work_Space.csv");
	Matrix4d gst;
	Eigen::Vector3d XYZ;
	vector<vector<double>> joint_limits = getJointLimits();
	cout << "Generating workspace data..." << endl;
	for (double i = joint_limits[0][0]; i < joint_limits[0][1]; i += increment)
	{
		changeAngle(0, i);
		for (double j = joint_limits[1][0]; j < joint_limits[1][1]; j += increment)
		{
			changeAngle(1, j);
			for (double k = joint_limits[2][0]; k < joint_limits[2][1]; k += increment)
			{
				changeAngle(2, k);
				for (double l = joint_limits[3][0]; l < joint_limits[3][1]; l += increment)
				{
					changeAngle(3, l);
					for (double m = joint_limits[4][0]; m < joint_limits[4][1]; m += increment)
					{
						changeAngle(4, m);
						for (double n = joint_limits[5][0]; n < joint_limits[5][1]; n += increment)
						{
							changeAngle(5, n);
							gst = calcArmFK();
							XYZ = gst.topRightCorner(3, 1);
							fileHandle << XYZ(0) << "," << XYZ(1) << "," << XYZ(2) << "\n";
						}
					}
				}
			}
		}
	}
	fileHandle.close();
	cout << "Finished" << endl;
}
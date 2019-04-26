#include "kinematics_calculator.h"

kinematics_calculator::kinematics_calculator() {
	/*cout << "In kinematics_calculator constructor." << endl;*/
}

Vector6d kinematics_calculator::calcTwist(Eigen::Vector3d w, Eigen::Vector3d p) {
	//takes in a UNIT vector for direction of rotation and a point on the axis of rotation, and returns the 6x1 twist coordinate of the rotation
	Eigen::Vector3d v = -w.cross(p);
	Vector6d zeta;
	zeta << v, w;
	return zeta;
}

Eigen::Matrix3d kinematics_calculator::calcVectorHat(Eigen::Vector3d v) {
	Eigen::Matrix3d vector_hat;
	vector_hat << 0, -v(2), v(1),
	           v(2), 0, -v(0),
	           -v(1), v(0), 0;
	return vector_hat;
}

Eigen::Matrix3d kinematics_calculator::calcRotationfromTwist(Vector6d zeta, double theta) {
	//construct a 3x3 rotation matrix from a twist coordinate, assuming the rotation has UNIT angular velocity
	Eigen::Vector3d w;
	w = zeta.bottomLeftCorner(3, 1);
	Eigen::Matrix3d w_hat;
	w_hat = calcVectorHat(w);
	Eigen::Matrix3d Rot_theta = Eigen::MatrixXd::Identity(3, 3) + w_hat * sin(theta) + (w_hat * w_hat) * (1 - cos(theta));
	return Rot_theta;
}

Matrix4d kinematics_calculator::calcTransformationfromTwist(Vector6d zeta, double theta) {
	//construct the full transformation matrix from a twist coordinate
	Matrix4d Tf_zeta_theta;
	Eigen::Vector3d w, v;
	v = zeta.topLeftCorner(3, 1);
	w = zeta.bottomLeftCorner(3, 1);
	Eigen::Matrix3d Rot_theta = calcRotationfromTwist(zeta, theta);
	Eigen::Vector3d p;
	p = (Eigen::MatrixXd::Identity(3, 3) - Rot_theta) * (w.cross(v)) + w * w.transpose() * v * theta;

	Tf_zeta_theta.topLeftCorner(3, 3) = Rot_theta;
	Tf_zeta_theta.topRightCorner(3, 1) = p;
	Tf_zeta_theta.bottomLeftCorner(1, 3) << 0, 0, 0;
	Tf_zeta_theta(3, 3) = 1;

	return Tf_zeta_theta;
}

Matrix4d kinematics_calculator::calcInitialConfig(vector<double> tool_offsets) {
	//takes in the x,y,z coordinates of the end-effector coordinate frame's origin, generate g_st(0), assuming the end-effector coordinate frame is aligned with the world coordiate frame
	Matrix4d g_st0;
	g_st0.topLeftCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3);
	g_st0(0, 3) = tool_offsets[0];
	g_st0(1, 3) = tool_offsets[1];
	g_st0(2, 3) = tool_offsets[2];
	g_st0(3, 3) = 1;
	g_st0.bottomLeftCorner(1, 3) << 0, 0, 0;
	return g_st0;
}

Matrix4d kinematics_calculator::calcInitialConfig(vector<double> tool_offsets, Eigen::Vector3d axis, double theta) {
	//takes in the x, y, z coordinates of the end-effector coordinate frame's origin, and a pre-rotation around a given axis to an angle theta
	Matrix4d g_st0;
	Eigen::Matrix3d w_hat;
	w_hat << 0, -axis(2), axis(1),
	      axis(2), 0, -axis(0),
	      -axis(1), axis(0), 0;
	Eigen::Matrix3d Rot_theta = Eigen::MatrixXd::Identity(3, 3) + w_hat * sin(theta) + (w_hat * w_hat) * (1 - cos(theta));
	g_st0(0, 3) = tool_offsets[0];
	g_st0(1, 3) = tool_offsets[1];
	g_st0(2, 3) = tool_offsets[2];
	g_st0(3, 3) = 1;
	g_st0.bottomLeftCorner(1, 3) << 0, 0, 0;
	g_st0.topLeftCorner(3, 3) = Rot_theta;
	return g_st0;
}

vector<Vector6d> kinematics_calculator::assembleTwists(vector<Eigen::Vector3d> coordinates) {
	//compiles "paired" w's and p's (in <w_i,p_i> order) and assemble them into twists for later use, e.g. in calcProductofExp func
	//example useage:
	/*Eigen::Vector3d w1(0, 1, 0), p1(2, 0, 0), w2(0, 0, 1), p2(0, 3, 0);
	vector<double> theta {1,1.5};
	vector<Eigen::Vector3d> coordinates {w1,p1,w2,p2};
	cout<<calcProductofExp(assembleTwists(coordinates),theta)<<endl;*/


	int num_coordinates = coordinates.size();
	if (num_coordinates % 2 != 0)
	{
		cout << "Number of inputs is odd. Please retry with an even pair of inputs." << endl;
	}
	else {
		vector<Vector6d> twist_coordinates;
		for (int i = 0, j = 0; i < num_coordinates; i += 2, j += 1)
		{
			twist_coordinates.push_back(calcTwist(coordinates[i], coordinates[i + 1]));
			/*cout<<twist_coordinates[j]<<endl;*/
		}
		return twist_coordinates;
	}
}

Matrix4d kinematics_calculator::calcProductofExp(vector<Vector6d> zetas, vector<double> thetas) {
	//takes a vector of twist coordinates and their associated joint values, and returns the result of product of exponentials
	//note that the "zero configuration" is not taken into account yet in this step. It is left out intentionally for ease of
	//calculating the Jacobian
	int num_zeta = zetas.size();
	int num_theta = thetas.size();
	if (num_zeta == 0 || num_theta != num_zeta)
	{
		cout << "Something about twist coordinates are wrong. Cannot proceed." << endl;
	}
	else {
		Matrix4d temp, final_product;
		final_product = Eigen::MatrixXd::Identity(4, 4); //initialize the final product as an identity matrix

		for (int i = 0; i < num_zeta; i++)
		{
			temp = calcTransformationfromTwist(zetas[i], thetas[i]);
			final_product *= temp;
		}
		return final_product;
	}
}



Matrix4d kinematics_calculator::calcFK(vector<Eigen::Vector3d> coordinates, vector<double> thetas, vector<double> tool_offsets) {
	vector<Vector6d> twist_coordinates (assembleTwists(coordinates));
	Matrix4d product_of_exponentials (calcProductofExp(twist_coordinates, thetas));
	Matrix4d gst_0 (calcInitialConfig(tool_offsets));
	Matrix4d gst_final (product_of_exponentials * gst_0);
	return gst_final;
}

Matrix4d kinematics_calculator::calcFK(vector<Eigen::Vector3d> coordinates, vector<double> thetas, vector<double> tool_offsets, Eigen::Vector3d tool_Rot_axis, double tool_Rot_angle) {
	vector<Vector6d> twist_coordinates (assembleTwists(coordinates));
	Matrix4d product_of_exponentials (calcProductofExp(twist_coordinates, thetas));
	Matrix4d gst_0 (calcInitialConfig(tool_offsets, tool_Rot_axis, tool_Rot_angle));
	Matrix4d gst_final (product_of_exponentials * gst_0);
	return gst_final;
}


#include "user_math.h"

int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

Eigen::Matrix3d eul_to_rotm(Orientation euler_angles)
{
	/* Converts euler angles to a rotation matrix */
	Eigen::Matrix3d R;
	R = (Eigen::AngleAxisd(euler_angles.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * Eigen::AngleAxisd(euler_angles.pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(euler_angles.roll, Eigen::Vector3d::UnitX()).toRotationMatrix());
	return R;
}

Orientation rotm_to_eul(Eigen::Matrix3d R)
{
	/* Converts a rotation matrix to euler angles */
	Orientation euler_angles;
	euler_angles.yaw = atan2(R(1, 0), R(0, 0));
	euler_angles.pitch = atan2(-R(2, 0), sqrt(pow(R(2, 1), 2) + pow(R(2, 2), 2)));
	euler_angles.roll = atan2(R(2, 1), R(2, 2));
	return euler_angles;
}

Pose trans_to_pose(Eigen::Matrix4d T)
{
	/* Converts a transformation matrix to a pose */
	Pose pose;
	pose.position.x = T(0, 3);
	pose.position.y = T(1, 3);
	pose.position.z = T(2, 3);
	pose.orientation = rotm_to_eul(T.block<3, 3>(0, 0));
	return pose;
}

Eigen::Matrix4d pose_to_trans(Pose pose)
{
	/* Converts a pose to a transformation matrix */
	Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
	T.block<3, 1>(0, 3) << pose.position.x, pose.position.y, pose.position.z;
	T.block<3, 3>(0, 0) = eul_to_rotm(pose.orientation);
	return T;
}

Eigen::VectorXd skew_to_twist(Eigen::Matrix4d S)
{
	/* Converts a 4x4 skew symmetric matrix representation of screw axis [S] to a twist V = [w;v]*/
	Eigen::VectorXd twist(6);
	twist << S(2, 1), S(0, 2), S(1, 0), S(0, 3), S(1, 3), S(2, 3);
	return twist;
}

Eigen::Matrix4d twist_to_skew(Eigen::VectorXd twist)
{
	/* Converts a twist V = [w;v] to a 4x4 skew symmetric matrix representation of screw axis [S] */
	Eigen::Matrix4d S;
	S << 0, -twist(2), twist(1), twist(3),
		twist(2), 0, -twist(0), twist(4),
		-twist(1), twist(0), 0, twist(5),
		0, 0, 0, 0;
	return S;
}

Eigen::MatrixXd trans_to_adj(Eigen::Matrix4d T)
{
	/* Converts a transformation matrix to its adjoint representation */
	Eigen::Matrix3d R = T.block<3, 3>(0, 0);
	Eigen::Matrix3d p_brkt; // p_brkt = [p], the skew symmetric matrix of p
	p_brkt << 0, -T(2, 3), T(1, 3),
		T(2, 3), 0, -T(0, 3),
		-T(1, 3), T(0, 3), 0;
	Eigen::MatrixXd adjoint(6, 6);
	adjoint.block<3, 3>(0, 0) = R;
	adjoint.block<3, 3>(3, 3) = R;
	adjoint.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
	adjoint.block<3, 3>(3, 0) = p_brkt * R;
	return adjoint;
}
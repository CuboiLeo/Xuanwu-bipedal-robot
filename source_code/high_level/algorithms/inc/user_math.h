#ifndef USER_MATH_H
#define USER_MATH_H

#include "math.h"
#include "Eigen/Dense"
#include "robot_types.h"

#define PI (3.14159265358979323846f)
#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)
#define GRAVITY (9.81f)

int float_to_uint(float x_float, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
Eigen::Matrix3d eul_to_rotm(Orientation euler_angles);
Orientation rotm_to_eul(Eigen::Matrix3d R);
Pose trans_to_pose(Eigen::Matrix4d T);
Eigen::Matrix4d pose_to_trans(Pose pose);
Eigen::VectorXd skew_to_twist(Eigen::Matrix4d S);
Eigen::Matrix4d twist_to_skew(Eigen::VectorXd twist);
Eigen::MatrixXd trans_to_adj(Eigen::Matrix4d T);
Eigen::Matrix4d trans_inv(Eigen::Matrix4d T);
Eigen::Matrix4d trans_log(Eigen::Matrix4d T);

#endif
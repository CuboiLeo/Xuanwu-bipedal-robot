#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdint.h>
#include <iostream>

struct Joint_Angles
{ // Joint angles in radians
    double hip_yaw;
    double hip_roll;
    double hip_pitch;
    double knee_pitch;
    double ankle_pitch;
};

struct Joint_Angles_Two_Legs
{ // Joint angles for both legs
    Joint_Angles left;
    Joint_Angles right;
};

struct Joint_Data
{
    Joint_Angles ref;
    Joint_Angles act;
};

struct Leg_Data
{
    Joint_Data left;
    Joint_Data right;
};

struct Vector3D
{ // Vector in 3D space
    double x;
    double y;
    double z;
};
using Position = Vector3D;             // Position in 3D space (meters)
using Velocity = Vector3D;             // Velocity in 3D space (m/s)
using Acceleration = Vector3D;         // Acceleration in 3D space (m/s^2)
using Angular_Velocity = Vector3D;     // Angular velocity in 3D space (rad/s)
using Angular_Acceleration = Vector3D; // Angular acceleration in 3D space (rad/s^2)

struct Position_Data
{
    Position ref;
    Position act;
};

struct Orientation
{                 // Orientation in euler angles
    double roll;  // rotation around x-axis
    double pitch; // rotation around y-axis
    double yaw;   // rotation around z-axis
};

struct Pose
{ // Pose in 3D space
    Position position;
    Orientation orientation;
};

struct Pose_Data
{
    Pose ref;
    Pose act;
};

struct Foot_Data
{
    Pose_Data left;
    Pose_Data right;
};

#endif // ROBOT_TYPES_H
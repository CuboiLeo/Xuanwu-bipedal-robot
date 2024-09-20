#include "Kinematics.h"
#include "User_Math.h"

Kinematics kinematics;

Kinematics::Kinematics()
{
      // Initialize inverse kinematics joint angles
      IK_left_leg_angles1 = {0.0f, 0.0f, 0.0f, 0.0f};
      IK_left_leg_angles2 = {0.0f, 0.0f, 0.0f, 0.0f};
      IK_left_leg_angles_final = {0.0f, 0.0f, 0.0f, 0.0f};
      IK_right_leg_angles1 = {0.0f, 0.0f, 0.0f, 0.0f};
      IK_right_leg_angles2 = {0.0f, 0.0f, 0.0f, 0.0f};
      IK_right_leg_angles_final = {0.0f, 0.0f, 0.0f, 0.0f};
      // Initialize forward kinematics foot positions
      FK_left_foot_pos = {0.0f, 0.0f, 0.0f};
      FK_right_foot_pos = {0.0f, 0.0f, 0.0f};
}

void Kinematics::computeForwardKinematics(Robot* robot)
{
      /* Forward kinematics computation, the final transformation matrix is computed in MATLAB
      T05 = T01*T12*T23*T34*T45

      T01 = [cos(theta1), -sin(theta1), 0, robot->DH_Left_Leg[0].a]
            [sin(theta1),  cos(theta1), 0,  0]
            [          0,            0, 1,  0]
            [          0,            0, 0,  1]
      T12 = [            cos(theta2),            -sin(theta2),            0, robot->DH_Left_Leg[1].a]
            [cos(robot->DH_Left_Leg[1].alpha)*sin(theta2), cos(robot->DH_Left_Leg[1].alpha)*cos(theta2), -sin(robot->DH_Left_Leg[1].alpha),  0]
            [sin(robot->DH_Left_Leg[1].alpha)*sin(theta2), sin(robot->DH_Left_Leg[1].alpha)*cos(theta2),  cos(robot->DH_Left_Leg[1].alpha),  0]
            [                      0,                       0,            0,  1]
      T23 = [            cos(theta3),            -sin(theta3),            0, robot->DH_Left_Leg[2].a]
            [cos(robot->DH_Left_Leg[2].alpha)*sin(theta3), cos(robot->DH_Left_Leg[2].alpha)*cos(theta3), -sin(robot->DH_Left_Leg[2].alpha),  0]
            [sin(robot->DH_Left_Leg[2].alpha)*sin(theta3), sin(robot->DH_Left_Leg[2].alpha)*cos(theta3),  cos(robot->DH_Left_Leg[2].alpha),  0]
            [                      0,                       0,            0,  1]
      T34 = [cos(theta4), -sin(theta4), 0, robot->DH_Left_Leg[3].a]
            [sin(theta4),  cos(theta4), 0,  0]
            [          0,            0, 1,  0]
            [          0,            0, 0,  1]
      T45 = [1, 0, 0, robot->DH_Left_Leg[4].a]
            [0, 1, 0,  0]
            [0, 0, 1,  0]
            [0, 0, 0,  1]
      */
      // Left Leg
      Joint_Angle left_leg = robot->getActJointAnglesLeft();
      Joint_Angle right_leg = robot->getActJointAnglesRight();
      left_leg.hip_roll += PI/2;
      right_leg.hip_roll -= PI/2;
      FK_left_foot_pos.x = robot->DH_Left_Leg[0].a + robot->DH_Left_Leg[3].a*(cosf(left_leg.hip_pitch)*(cosf(left_leg.hip_yaw)*cosf(left_leg.hip_roll) 
                        - cosf(robot->DH_Left_Leg[1].alpha)*sinf(left_leg.hip_yaw)*sinf(left_leg.hip_roll)) - cosf(robot->DH_Left_Leg[2].alpha)*sinf(left_leg.hip_pitch)
                        *(cosf(left_leg.hip_yaw)*sinf(left_leg.hip_roll) + cosf(robot->DH_Left_Leg[1].alpha)*cosf(left_leg.hip_roll)
                        *sinf(left_leg.hip_yaw)) + sinf(robot->DH_Left_Leg[1].alpha)*sinf(robot->DH_Left_Leg[2].alpha)*sinf(left_leg.hip_yaw)*sinf(left_leg.hip_pitch)) 
                        + robot->DH_Left_Leg[4].a*(cosf(left_leg.knee_pitch)*(cosf(left_leg.hip_pitch)*(cosf(left_leg.hip_yaw)*cosf(left_leg.hip_roll)
                        - cosf(robot->DH_Left_Leg[1].alpha)*sinf(left_leg.hip_yaw)*sinf(left_leg.hip_roll)) - cosf(robot->DH_Left_Leg[2].alpha)*sinf(left_leg.hip_pitch)
                        *(cosf(left_leg.hip_yaw)*sinf(left_leg.hip_roll) + cosf(robot->DH_Left_Leg[1].alpha)*cosf(left_leg.hip_roll)
                        *sinf(left_leg.hip_yaw)) + sinf(robot->DH_Left_Leg[1].alpha)*sinf(robot->DH_Left_Leg[2].alpha)*sinf(left_leg.hip_yaw)*sinf(left_leg.hip_pitch)) 
                        - sinf(left_leg.knee_pitch)*(sinf(left_leg.hip_pitch)*(cosf(left_leg.hip_yaw)*cosf(left_leg.hip_roll) 
                        - cosf(robot->DH_Left_Leg[1].alpha)*sinf(left_leg.hip_yaw)*sinf(left_leg.hip_roll)) + cosf(robot->DH_Left_Leg[2].alpha)*cosf(left_leg.hip_pitch)
                        *(cosf(left_leg.hip_yaw)*sinf(left_leg.hip_roll) + cosf(robot->DH_Left_Leg[1].alpha)*cosf(left_leg.hip_roll)
                        *sinf(left_leg.hip_yaw)) - sinf(robot->DH_Left_Leg[1].alpha)*sinf(robot->DH_Left_Leg[2].alpha)*cosf(left_leg.hip_pitch)*sinf(left_leg.hip_yaw))) 
                        + robot->DH_Left_Leg[2].a*(cosf(left_leg.hip_yaw)*cosf(left_leg.hip_roll) - cosf(robot->DH_Left_Leg[1].alpha)*sinf(left_leg.hip_yaw)
                        *sinf(left_leg.hip_roll)) + robot->DH_Left_Leg[1].a*cosf(left_leg.hip_yaw);
      FK_left_foot_pos.y = robot->DH_Left_Leg[2].a*(cosf(left_leg.hip_roll)*sinf(left_leg.hip_yaw) + cosf(robot->DH_Left_Leg[1].alpha)*cosf(left_leg.hip_yaw)
                        *sinf(left_leg.hip_roll)) + robot->DH_Left_Leg[1].a*sinf(left_leg.hip_yaw) - robot->DH_Left_Leg[4].a*(sinf(left_leg.knee_pitch)
                        *(sinf(left_leg.hip_pitch)*(cosf(left_leg.hip_roll)*sinf(left_leg.hip_yaw) + cosf(robot->DH_Left_Leg[1].alpha)
                        *cosf(left_leg.hip_yaw)*sinf(left_leg.hip_roll)) + cosf(robot->DH_Left_Leg[2].alpha)*cosf(left_leg.hip_pitch)
                        *(sinf(left_leg.hip_yaw)*sinf(left_leg.hip_roll) - cosf(robot->DH_Left_Leg[1].alpha)*cosf(left_leg.hip_yaw)
                        *cosf(left_leg.hip_roll)) + sinf(robot->DH_Left_Leg[1].alpha)*sinf(robot->DH_Left_Leg[2].alpha)*cosf(left_leg.hip_yaw)*cosf(left_leg.hip_pitch)) 
                        + cosf(left_leg.knee_pitch)*(cosf(robot->DH_Left_Leg[2].alpha)*sinf(left_leg.hip_pitch)*(sinf(left_leg.hip_yaw)
                        *sinf(left_leg.hip_roll) - cosf(robot->DH_Left_Leg[1].alpha)*cosf(left_leg.hip_yaw)*cosf(left_leg.hip_roll)) 
                        - cosf(left_leg.hip_pitch)*(cosf(left_leg.hip_roll)*sinf(left_leg.hip_yaw) + cosf(robot->DH_Left_Leg[1].alpha)
                        *cosf(left_leg.hip_yaw)*sinf(left_leg.hip_roll)) + sinf(robot->DH_Left_Leg[1].alpha)*sinf(robot->DH_Left_Leg[2].alpha)*cosf(left_leg.hip_yaw)
                        *sinf(left_leg.hip_pitch))) - robot->DH_Left_Leg[3].a*(cosf(robot->DH_Left_Leg[2].alpha)*sinf(left_leg.hip_pitch)*(sinf(left_leg.hip_yaw)
                        *sinf(left_leg.hip_roll) - cosf(robot->DH_Left_Leg[1].alpha)*cosf(left_leg.hip_yaw)*cosf(left_leg.hip_roll)) 
                        - cosf(left_leg.hip_pitch)*(cosf(left_leg.hip_roll)*sinf(left_leg.hip_yaw) + cosf(robot->DH_Left_Leg[1].alpha)
                        *cosf(left_leg.hip_yaw)*sinf(left_leg.hip_roll)) + sinf(robot->DH_Left_Leg[1].alpha)*sinf(robot->DH_Left_Leg[2].alpha)*cosf(left_leg.hip_yaw)
                        *sinf(left_leg.hip_pitch));
      FK_left_foot_pos.z = robot->DH_Left_Leg[4].a*(cosf(left_leg.knee_pitch)*(cosf(robot->DH_Left_Leg[1].alpha)*sinf(robot->DH_Left_Leg[2].alpha)*sinf(left_leg.hip_pitch) + sinf(robot->DH_Left_Leg[1].alpha)
                        *cosf(left_leg.hip_pitch)*sinf(left_leg.hip_roll) + cosf(robot->DH_Left_Leg[2].alpha)*sinf(robot->DH_Left_Leg[1].alpha)*cosf(left_leg.hip_roll)
                        *sinf(left_leg.hip_pitch)) + sinf(left_leg.knee_pitch)*(cosf(robot->DH_Left_Leg[1].alpha)*sinf(robot->DH_Left_Leg[2].alpha)*cosf(left_leg.hip_pitch) 
                        - sinf(robot->DH_Left_Leg[1].alpha)*sinf(left_leg.hip_roll)*sinf(left_leg.hip_pitch) + cosf(robot->DH_Left_Leg[2].alpha)*sinf(robot->DH_Left_Leg[1].alpha)*cosf(left_leg.hip_roll)
                        *cosf(left_leg.hip_pitch))) + robot->DH_Left_Leg[3].a*(cosf(robot->DH_Left_Leg[1].alpha)*sinf(robot->DH_Left_Leg[2].alpha)*sinf(left_leg.hip_pitch) + sinf(robot->DH_Left_Leg[1].alpha)
                        *cosf(left_leg.hip_pitch)*sinf(left_leg.hip_roll) + cosf(robot->DH_Left_Leg[2].alpha)*sinf(robot->DH_Left_Leg[1].alpha)*cosf(left_leg.hip_roll)
                        *sinf(left_leg.hip_pitch)) + robot->DH_Left_Leg[2].a*sinf(robot->DH_Left_Leg[1].alpha)*sinf(left_leg.hip_roll);

      FK_right_foot_pos.x = robot->DH_Right_Leg[0].a + robot->DH_Right_Leg[3].a*(cosf(right_leg.hip_pitch)*(cosf(right_leg.hip_yaw)*cosf(right_leg.hip_roll)
                        - cosf(robot->DH_Right_Leg[1].alpha)*sinf(right_leg.hip_yaw)*sinf(right_leg.hip_roll)) - cosf(robot->DH_Right_Leg[2].alpha)*sinf(right_leg.hip_pitch)
                        *(cosf(right_leg.hip_yaw)*sinf(right_leg.hip_roll) + cosf(robot->DH_Right_Leg[1].alpha)*cosf(right_leg.hip_roll)
                        *sinf(right_leg.hip_yaw)) + sinf(robot->DH_Right_Leg[1].alpha)*sinf(robot->DH_Right_Leg[2].alpha)*sinf(right_leg.hip_yaw)*sinf(right_leg.hip_pitch)) 
                        + robot->DH_Right_Leg[4].a*(cosf(right_leg.knee_pitch)*(cosf(right_leg.hip_pitch)*(cosf(right_leg.hip_yaw)*cosf(right_leg.hip_roll)
                        - cosf(robot->DH_Right_Leg[1].alpha)*sinf(right_leg.hip_yaw)*sinf(right_leg.hip_roll)) - cosf(robot->DH_Right_Leg[2].alpha)*sinf(right_leg.hip_pitch)
                        *(cosf(right_leg.hip_yaw)*sinf(right_leg.hip_roll) + cosf(robot->DH_Right_Leg[1].alpha)*cosf(right_leg.hip_roll)
                        *sinf(right_leg.hip_yaw)) + sinf(robot->DH_Right_Leg[1].alpha)*sinf(robot->DH_Right_Leg[2].alpha)*sinf(right_leg.hip_yaw)*sinf(right_leg.hip_pitch)) 
                        - sinf(right_leg.knee_pitch)*(sinf(right_leg.hip_pitch)*(cosf(right_leg.hip_yaw)*cosf(right_leg.hip_roll) 
                        - cosf(robot->DH_Right_Leg[1].alpha)*sinf(right_leg.hip_yaw)*sinf(right_leg.hip_roll)) + cosf(robot->DH_Right_Leg[2].alpha)*cosf(right_leg.hip_pitch)
                        *(cosf(right_leg.hip_yaw)*sinf(right_leg.hip_roll) + cosf(robot->DH_Right_Leg[1].alpha)*cosf(right_leg.hip_roll)
                        *sinf(right_leg.hip_yaw)) - sinf(robot->DH_Right_Leg[1].alpha)*sinf(robot->DH_Right_Leg[2].alpha)*cosf(right_leg.hip_pitch)*sinf(right_leg.hip_yaw)))
                        + robot->DH_Right_Leg[2].a*(cosf(right_leg.hip_yaw)*cosf(right_leg.hip_roll) - cosf(robot->DH_Right_Leg[1].alpha)*sinf(right_leg.hip_yaw)
                        *sinf(right_leg.hip_roll)) + robot->DH_Right_Leg[1].a*cosf(right_leg.hip_yaw);
      FK_right_foot_pos.y = robot->DH_Right_Leg[2].a*(cosf(right_leg.hip_roll)*sinf(right_leg.hip_yaw) + cosf(robot->DH_Right_Leg[1].alpha)*cosf(right_leg.hip_yaw)
                        *sinf(right_leg.hip_roll)) + robot->DH_Right_Leg[1].a*sinf(right_leg.hip_yaw) - robot->DH_Right_Leg[4].a*(sinf(right_leg.knee_pitch)
                        *(sinf(right_leg.hip_pitch)*(cosf(right_leg.hip_roll)*sinf(right_leg.hip_yaw) + cosf(robot->DH_Right_Leg[1].alpha)
                        *cosf(right_leg.hip_yaw)*sinf(right_leg.hip_roll)) + cosf(robot->DH_Right_Leg[2].alpha)*cosf(right_leg.hip_pitch)
                        *(sinf(right_leg.hip_yaw)*sinf(right_leg.hip_roll) - cosf(robot->DH_Right_Leg[1].alpha)*cosf(right_leg.hip_yaw)
                        *cosf(right_leg.hip_roll)) + sinf(robot->DH_Right_Leg[1].alpha)*sinf(robot->DH_Right_Leg[2].alpha)*cosf(right_leg.hip_yaw)*cosf(right_leg.hip_pitch)) 
                        + cosf(right_leg.knee_pitch)*(cosf(robot->DH_Right_Leg[2].alpha)*sinf(right_leg.hip_pitch)*(sinf(right_leg.hip_yaw)
                        *sinf(right_leg.hip_roll) - cosf(robot->DH_Right_Leg[1].alpha)*cosf(right_leg.hip_yaw)*cosf(right_leg.hip_roll)) 
                        - cosf(right_leg.hip_pitch)*(cosf(right_leg.hip_roll)*sinf(right_leg.hip_yaw) + cosf(robot->DH_Right_Leg[1].alpha)
                        *cosf(right_leg.hip_yaw)*sinf(right_leg.hip_roll)) + sinf(robot->DH_Right_Leg[1].alpha)*sinf(robot->DH_Right_Leg[2].alpha)*cosf(right_leg.hip_yaw)
                        *sinf(right_leg.hip_pitch))) - robot->DH_Right_Leg[3].a*(cosf(robot->DH_Right_Leg[2].alpha)*sinf(right_leg.hip_pitch)*(sinf(right_leg.hip_yaw)
                        *sinf(right_leg.hip_roll) - cosf(robot->DH_Right_Leg[1].alpha)*cosf(right_leg.hip_yaw)*cosf(right_leg.hip_roll))
                        - cosf(right_leg.hip_pitch)*(cosf(right_leg.hip_roll)*sinf(right_leg.hip_yaw) + cosf(robot->DH_Right_Leg[1].alpha)
                        *cosf(right_leg.hip_yaw)*sinf(right_leg.hip_roll)) + sinf(robot->DH_Right_Leg[1].alpha)*sinf(robot->DH_Right_Leg[2].alpha)*cosf(right_leg.hip_yaw)
                        *sinf(right_leg.hip_pitch));
      FK_right_foot_pos.z = robot->DH_Right_Leg[4].a*(cosf(right_leg.knee_pitch)*(cosf(robot->DH_Right_Leg[1].alpha)*sinf(robot->DH_Right_Leg[2].alpha)*sinf(right_leg.hip_pitch) + sinf(robot->DH_Right_Leg[1].alpha)
                        *cosf(right_leg.hip_pitch)*sinf(right_leg.hip_roll) + cosf(robot->DH_Right_Leg[2].alpha)*sinf(robot->DH_Right_Leg[1].alpha)*cosf(right_leg.hip_roll)
                        *sinf(right_leg.hip_pitch)) + sinf(right_leg.knee_pitch)*(cosf(robot->DH_Right_Leg[1].alpha)*sinf(robot->DH_Right_Leg[2].alpha)*cosf(right_leg.hip_pitch) 
                        - sinf(robot->DH_Right_Leg[1].alpha)*sinf(right_leg.hip_roll)*sinf(right_leg.hip_pitch) + cosf(robot->DH_Right_Leg[2].alpha)*sinf(robot->DH_Right_Leg[1].alpha)*cosf(right_leg.hip_roll)
                        *cosf(right_leg.hip_pitch))) + robot->DH_Right_Leg[3].a*(cosf(robot->DH_Right_Leg[1].alpha)*sinf(robot->DH_Right_Leg[2].alpha)*sinf(right_leg.hip_pitch) + sinf(robot->DH_Right_Leg[1].alpha)
                        *cosf(right_leg.hip_pitch)*sinf(right_leg.hip_roll) + cosf(robot->DH_Right_Leg[2].alpha)*sinf(robot->DH_Right_Leg[1].alpha)*cosf(right_leg.hip_roll)
                        *sinf(right_leg.hip_pitch)) + robot->DH_Right_Leg[2].a*sinf(robot->DH_Right_Leg[1].alpha)*sinf(right_leg.hip_roll);

      // Update the robot's foot positions
      robot->setActFootPosLeft(FK_left_foot_pos);
      robot->setActFootPosRight(FK_right_foot_pos);
}

void Kinematics::computeInverseKinematics(Robot* robot)
{
      Foot_Position left_foot = robot->getRefFootPosLeft();
      Foot_Position right_foot = robot->getRefFootPosRight();
      
      // Inverse Kinematics for left leg
      IK_left_leg_angles1.hip_yaw = atan2f(left_foot.y,left_foot.x)-atan2f(robot->DH_Left_Leg[1].a,sqrtf(left_foot.x*left_foot.x
                                     +left_foot.y*left_foot.y-robot->DH_Left_Leg[1].a*robot->DH_Left_Leg[1].a))-3*PI/2;
      IK_left_leg_angles2.hip_yaw = PI+atan2f(left_foot.y,left_foot.x)+atan2f(-sqrtf(left_foot.x*left_foot.x
                                          +left_foot.y*left_foot.y-robot->DH_Left_Leg[1].a*robot->DH_Left_Leg[1].a),robot->DH_Left_Leg[1].a);

      IK_left_leg_angles1.knee_pitch = atan2f(sqrtf(1.0f-((left_foot.x*left_foot.x+left_foot.y*left_foot.y
                                          +left_foot.z*left_foot.z-robot->DH_Left_Leg[1].a*robot->DH_Left_Leg[1].a-robot->DH_Left_Leg[3].a*robot->DH_Left_Leg[3].a-robot->DH_Left_Leg[4].a*robot->DH_Left_Leg[4].a)/(2*robot->DH_Left_Leg[3].a*robot->DH_Left_Leg[4].a))
                                          *((left_foot.x*left_foot.x+left_foot.y*left_foot.y
                                          +left_foot.z*left_foot.z-robot->DH_Left_Leg[1].a*robot->DH_Left_Leg[1].a-robot->DH_Left_Leg[3].a*robot->DH_Left_Leg[3].a-robot->DH_Left_Leg[4].a*robot->DH_Left_Leg[4].a)/(2*robot->DH_Left_Leg[3].a*robot->DH_Left_Leg[4].a)))
                                          ,((left_foot.x*left_foot.x+left_foot.y*left_foot.y+left_foot.z
                                          *left_foot.z-robot->DH_Left_Leg[1].a*robot->DH_Left_Leg[1].a-robot->DH_Left_Leg[3].a*robot->DH_Left_Leg[3].a-robot->DH_Left_Leg[4].a*robot->DH_Left_Leg[4].a)/(2*robot->DH_Left_Leg[3].a*robot->DH_Left_Leg[4].a)));
      IK_left_leg_angles2.knee_pitch = atan2f(-sqrtf(1.0f-((left_foot.x*left_foot.x+left_foot.y*left_foot.y
                                          +left_foot.z*left_foot.z-robot->DH_Left_Leg[1].a*robot->DH_Left_Leg[1].a-robot->DH_Left_Leg[3].a*robot->DH_Left_Leg[3].a-robot->DH_Left_Leg[4].a*robot->DH_Left_Leg[4].a)/(2*robot->DH_Left_Leg[3].a*robot->DH_Left_Leg[4].a))
                                          *((left_foot.x*left_foot.x+left_foot.y*left_foot.y
                                          +left_foot.z*left_foot.z-robot->DH_Left_Leg[1].a*robot->DH_Left_Leg[1].a-robot->DH_Left_Leg[3].a*robot->DH_Left_Leg[3].a-robot->DH_Left_Leg[4].a*robot->DH_Left_Leg[4].a)/(2*robot->DH_Left_Leg[3].a*robot->DH_Left_Leg[4].a)))
                                          ,((left_foot.x*left_foot.x+left_foot.y*left_foot.y+left_foot.z
                                          *left_foot.z-robot->DH_Left_Leg[1].a*robot->DH_Left_Leg[1].a-robot->DH_Left_Leg[3].a*robot->DH_Left_Leg[3].a-robot->DH_Left_Leg[4].a*robot->DH_Left_Leg[4].a)/(2*robot->DH_Left_Leg[3].a*robot->DH_Left_Leg[4].a)));

      IK_left_leg_angles1.hip_pitch = atan2f(left_foot.z,sqrtf(left_foot.x*left_foot.x+left_foot.y
                                          *left_foot.y-robot->DH_Left_Leg[1].a*robot->DH_Left_Leg[1].a))-atan2f(robot->DH_Left_Leg[4].a*sin(IK_left_leg_angles1.knee_pitch),robot->DH_Left_Leg[3].a+robot->DH_Left_Leg[4].a
                                          *cos(IK_left_leg_angles1.knee_pitch))-PI/2;
      IK_left_leg_angles2.hip_pitch = atan2f(left_foot.z,sqrtf(left_foot.x*left_foot.x+left_foot.y
                                          *left_foot.y-robot->DH_Left_Leg[1].a*robot->DH_Left_Leg[1].a))-atan2f(robot->DH_Left_Leg[4].a*sin(IK_left_leg_angles2.knee_pitch),robot->DH_Left_Leg[3].a+robot->DH_Left_Leg[4].a
                                          *cos(IK_left_leg_angles2.knee_pitch));

      // Update the robot's joint angles
      robot->setActJointAnglesLeft(IK_left_leg_angles1);
}



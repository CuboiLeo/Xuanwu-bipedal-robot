#include "Orin_NX.h"

void Orin::sendData(const Motor &motor, const IMU &imu, const Direction_Vector &ref_vel, const Direction_Vector &ref_angular_vel)
{
    uint16_t pos_tmp, vel_tmp, tor_tmp;
    pos_tmp = float_to_uint(motor.getPos(Left_Hip_Yaw),  P_MIN,  P_MAX,  16);
    vel_tmp = float_to_uint(motor.getVel(Left_Hip_Yaw),  V_MIN,  V_MAX,  12);
    tor_tmp = float_to_uint(motor.getTor(Left_Hip_Yaw), T_MIN,  T_MAX,  12);
    send_data[0][0] = 0x01;
    send_data[0][1] = (pos_tmp >> 8);
    send_data[0][2] = pos_tmp;
    send_data[0][3] = (vel_tmp >> 4);
    send_data[0][4] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[0][5] = tor_tmp;

    pos_tmp = float_to_uint(motor.getPos(Left_Hip_Roll),  P_MIN,  P_MAX,  16);
    vel_tmp = float_to_uint(motor.getVel(Left_Hip_Roll),  V_MIN,  V_MAX,  12);
    tor_tmp = float_to_uint(motor.getTor(Left_Hip_Roll), T_MIN,  T_MAX,  12);
    send_data[0][6] = (pos_tmp >> 8);
    send_data[0][7] = pos_tmp;
    send_data[1][0] = (vel_tmp >> 4);
    send_data[1][1] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[1][2] = tor_tmp;

    pos_tmp = float_to_uint(motor.getPos(Left_Hip_Pitch),  P_MIN,  P_MAX,  16);
    vel_tmp = float_to_uint(motor.getVel(Left_Hip_Pitch),  V_MIN,  V_MAX,  12);
    tor_tmp = float_to_uint(motor.getTor(Left_Hip_Pitch), T_MIN,  T_MAX,  12);
    send_data[1][3] = (pos_tmp >> 8);
    send_data[1][4] = pos_tmp;
    send_data[1][5] = (vel_tmp >> 4);
    send_data[1][6] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[1][7] = tor_tmp;

    pos_tmp = float_to_uint(motor.getPos(Left_Knee_Pitch),  P_MIN,  P_MAX,  16);
    vel_tmp = float_to_uint(motor.getVel(Left_Knee_Pitch),  V_MIN,  V_MAX,  12);
    tor_tmp = float_to_uint(motor.getTor(Left_Knee_Pitch), T_MIN,  T_MAX,  12);
    send_data[2][0] = (pos_tmp >> 8);
    send_data[2][1] = pos_tmp;
    send_data[2][2] = (vel_tmp >> 4);
    send_data[2][3] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[2][4] = tor_tmp;

    pos_tmp = float_to_uint(motor.getPos(Right_Hip_Yaw),  P_MIN,  P_MAX,  16);
    vel_tmp = float_to_uint(motor.getVel(Right_Hip_Yaw),  V_MIN,  V_MAX,  12);
    tor_tmp = float_to_uint(motor.getTor(Right_Hip_Yaw), T_MIN,  T_MAX,  12);
    send_data[2][5] = (vel_tmp >> 4);
    send_data[2][6] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[2][7] = tor_tmp;
    send_data[3][0] = 0x02;
    send_data[3][1] = (pos_tmp >> 8);
    send_data[3][2] = pos_tmp;

    pos_tmp = float_to_uint(motor.getPos(Right_Hip_Roll),  P_MIN,  P_MAX,  16);
    vel_tmp = float_to_uint(motor.getVel(Right_Hip_Roll),  V_MIN,  V_MAX,  12);
    tor_tmp = float_to_uint(motor.getTor(Right_Hip_Roll), T_MIN,  T_MAX,  12);
    send_data[3][3] = (pos_tmp >> 8);
    send_data[3][4] = pos_tmp;
    send_data[3][5] = (vel_tmp >> 4);
    send_data[3][6] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[3][7] = tor_tmp;

    pos_tmp = float_to_uint(motor.getPos(Right_Hip_Pitch),  P_MIN,  P_MAX,  16);
    vel_tmp = float_to_uint(motor.getVel(Right_Hip_Pitch),  V_MIN,  V_MAX,  12);
    tor_tmp = float_to_uint(motor.getTor(Right_Hip_Pitch), T_MIN,  T_MAX,  12);
    send_data[4][0] = (pos_tmp >> 8);
    send_data[4][1] = pos_tmp;
    send_data[4][2] = (vel_tmp >> 4);
    send_data[4][3] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[4][4] = tor_tmp;

    pos_tmp = float_to_uint(motor.getPos(Right_Knee_Pitch),  P_MIN,  P_MAX,  16);
    vel_tmp = float_to_uint(motor.getVel(Right_Knee_Pitch),  V_MIN,  V_MAX,  12);
    tor_tmp = float_to_uint(motor.getTor(Right_Knee_Pitch), T_MIN,  T_MAX,  12);
    send_data[4][5] = (vel_tmp >> 4);
    send_data[4][6] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[4][7] = tor_tmp;
    send_data[5][0] = pos_tmp >> 8;
    send_data[5][1] = pos_tmp;

    uint8_t vx, vy, vz, wz;
    vx = float_to_uint(ref_vel.x, -ROBOT_MAX_VEL, ROBOT_MAX_VEL, 4);
    vy = float_to_uint(ref_vel.y, -ROBOT_MAX_VEL, ROBOT_MAX_VEL, 4);
    vz = float_to_uint(ref_vel.z, -ROBOT_MAX_VEL, ROBOT_MAX_VEL, 4);
    wz = float_to_uint(ref_angular_vel.z, -ROBOT_MAX_ANG_VEL, ROBOT_MAX_ANG_VEL, 4);
    send_data[5][2] = vx << 4 | vy;
    send_data[5][3] = vz << 4 | wz;

    uint16_t acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
    FusionVector acc = imu.getAccel();
    FusionVector gyro = imu.getGyro();
    acc_x = float_to_uint(acc.axis.x, -3*GRAVITY, 3*GRAVITY, 16);
    acc_y = float_to_uint(acc.axis.y, -3*GRAVITY, 3*GRAVITY, 16);
    acc_z = float_to_uint(acc.axis.z, -3*GRAVITY, 3*GRAVITY, 16);
    gyro_x = float_to_uint(gyro.axis.x, -2000, 2000, 16);
    gyro_y = float_to_uint(gyro.axis.y, -2000, 2000, 16);
    gyro_z = float_to_uint(gyro.axis.z, -2000, 2000, 16);
    send_data[5][4] = acc_x >> 8;
    send_data[5][5] = acc_x;
    send_data[5][6] = acc_y >> 8;
    send_data[5][7] = acc_y;
    send_data[6][0] = acc_z >> 8;
    send_data[6][1] = acc_z;
    send_data[6][2] = gyro_x >> 8;
    send_data[6][3] = gyro_x;
    send_data[6][4] = gyro_y >> 8;
    send_data[6][5] = gyro_y;
    send_data[6][6] = gyro_z >> 8;
    send_data[6][7] = gyro_z;

    for(uint8_t i = 0, n = CAN_SEND_START_ID; i < SEND_PACKAGE_NUM; i++)
    {
        fdcanx_send_data(&hfdcan3, n + i, send_data[i], sizeof(send_data[i]));
    }
}

void Orin::receiveData(Motor &motor)
{
    for(uint8_t i = 0, n = CAN_RECEIVE_START_ID; i < RECEIVE_PACKAGE_NUM; i++)
    {
        fdcanx_receive(&hfdcan3, n + i, receive_data[i]);
    }

    uint16_t pos_tmp, vel_tmp, tor_tmp;
    pos_tmp = receive_data[0][1] << 8 | receive_data[0][2];
    vel_tmp = receive_data[0][3] << 4 | receive_data[0][4] >> 4;
    tor_tmp = (receive_data[0][4] & 0xF) << 8 | receive_data[0][5];
    motor.setPos(Left_Hip_Yaw, uint_to_float(pos_tmp,  P_MIN,  P_MAX,  16));
    motor.setVel(Left_Hip_Yaw, uint_to_float(vel_tmp,  V_MIN,  V_MAX,  12));
    motor.setTor(Left_Hip_Yaw, uint_to_float(tor_tmp, T_MIN,  T_MAX,  12));

    pos_tmp = receive_data[0][6] << 8 | receive_data[0][7];
    vel_tmp = receive_data[1][0] << 4 | receive_data[1][1] >> 4;
    tor_tmp = (receive_data[1][1] & 0xF) << 8 | receive_data[1][2];
    motor.setPos(Left_Hip_Roll, uint_to_float(pos_tmp,  P_MIN,  P_MAX,  16));
    motor.setVel(Left_Hip_Roll, uint_to_float(vel_tmp,  V_MIN,  V_MAX,  12));
    motor.setTor(Left_Hip_Roll, uint_to_float(tor_tmp, T_MIN,  T_MAX,  12));

    pos_tmp = receive_data[1][3] << 8 | receive_data[1][4];
    vel_tmp = receive_data[1][5] << 4 | receive_data[1][6] >> 4;
    tor_tmp = (receive_data[1][6] & 0xF) << 8 | receive_data[1][7];
    motor.setPos(Left_Hip_Pitch, uint_to_float(pos_tmp,  P_MIN,  P_MAX,  16));
    motor.setVel(Left_Hip_Pitch, uint_to_float(vel_tmp,  V_MIN,  V_MAX,  12));
    motor.setTor(Left_Hip_Pitch, uint_to_float(tor_tmp, T_MIN,  T_MAX,  12));

    pos_tmp = receive_data[2][0] << 8 | receive_data[2][1];
    vel_tmp = receive_data[2][2] << 4 | receive_data[2][3] >> 4;
    tor_tmp = (receive_data[2][3] & 0xF) << 8 | receive_data[2][4];
    motor.setPos(Left_Knee_Pitch, uint_to_float(pos_tmp,  P_MIN,  P_MAX,  16));
    motor.setVel(Left_Knee_Pitch, uint_to_float(vel_tmp,  V_MIN,  V_MAX,  12));
    motor.setTor(Left_Knee_Pitch, uint_to_float(tor_tmp, T_MIN,  T_MAX,  12));

    pos_tmp = receive_data[3][1] << 8 | receive_data[3][2];
    vel_tmp = receive_data[2][5] << 4 | receive_data[2][6] >> 4;
    tor_tmp = (receive_data[2][6] & 0xF) << 8 | receive_data[2][7];
    motor.setPos(Right_Hip_Yaw, uint_to_float(pos_tmp,  P_MIN,  P_MAX,  16));
    motor.setVel(Right_Hip_Yaw, uint_to_float(vel_tmp,  V_MIN,  V_MAX,  12));
    motor.setTor(Right_Hip_Yaw, uint_to_float(tor_tmp, T_MIN,  T_MAX,  12));

    pos_tmp = receive_data[3][3] << 8 | receive_data[3][4];
    vel_tmp = receive_data[3][5] << 4 | receive_data[3][6] >> 4;
    tor_tmp = (receive_data[3][6] & 0xF) << 8 | receive_data[3][7];
    motor.setPos(Right_Hip_Roll, uint_to_float(pos_tmp,  P_MIN,  P_MAX,  16));
    motor.setVel(Right_Hip_Roll, uint_to_float(vel_tmp,  V_MIN,  V_MAX,  12));
    motor.setTor(Right_Hip_Roll, uint_to_float(tor_tmp, T_MIN,  T_MAX,  12));

    pos_tmp = receive_data[4][0] << 8 | receive_data[4][1];
    vel_tmp = receive_data[4][2] << 4 | receive_data[4][3] >> 4;
    tor_tmp = (receive_data[4][3] & 0xF) << 8 | receive_data[4][4];
    motor.setPos(Right_Hip_Pitch, uint_to_float(pos_tmp,  P_MIN,  P_MAX,  16));
    motor.setVel(Right_Hip_Pitch, uint_to_float(vel_tmp,  V_MIN,  V_MAX,  12));
    motor.setTor(Right_Hip_Pitch, uint_to_float(tor_tmp, T_MIN,  T_MAX,  12));

    pos_tmp = receive_data[5][0] << 8 | receive_data[5][1];
    vel_tmp = receive_data[4][5] << 4 | receive_data[4][6] >> 4;
    tor_tmp = (receive_data[4][6] & 0xF) << 8 | receive_data[4][7];
    motor.setPos(Right_Knee_Pitch, uint_to_float(pos_tmp,  P_MIN,  P_MAX,  16));
    motor.setVel(Right_Knee_Pitch, uint_to_float(vel_tmp,  V_MIN,  V_MAX,  12));
    motor.setTor(Right_Knee_Pitch, uint_to_float(tor_tmp, T_MIN,  T_MAX,  12));
}
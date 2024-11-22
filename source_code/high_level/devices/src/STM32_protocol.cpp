#include "STM32_protocol.h"

void STM32::sendData(CAN &can, Motor &motor)
{
    uint16_t pos_tmp, vel_tmp, tor_tmp;
    pos_tmp = float_to_uint(motor.getPos(Left_Hip_Yaw), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getVel(Left_Hip_Yaw), V_MIN, V_MAX, 12);
    tor_tmp = float_to_uint(motor.getTor(Left_Hip_Yaw), T_MIN, T_MAX, 12);
    send_data[0][0] = 0x01;
    send_data[0][1] = (pos_tmp >> 8);
    send_data[0][2] = pos_tmp;
    send_data[0][3] = (vel_tmp >> 4);
    send_data[0][4] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[0][5] = tor_tmp;

    pos_tmp = float_to_uint(motor.getPos(Left_Hip_Roll), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getVel(Left_Hip_Roll), V_MIN, V_MAX, 12);
    tor_tmp = float_to_uint(motor.getTor(Left_Hip_Roll), T_MIN, T_MAX, 12);
    send_data[0][6] = (pos_tmp >> 8);
    send_data[0][7] = pos_tmp;
    send_data[1][0] = (vel_tmp >> 4);
    send_data[1][1] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[1][2] = tor_tmp;

    pos_tmp = float_to_uint(motor.getPos(Left_Hip_Pitch), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getVel(Left_Hip_Pitch), V_MIN, V_MAX, 12);
    tor_tmp = float_to_uint(motor.getTor(Left_Hip_Pitch), T_MIN, T_MAX, 12);
    send_data[1][3] = (pos_tmp >> 8);
    send_data[1][4] = pos_tmp;
    send_data[1][5] = (vel_tmp >> 4);
    send_data[1][6] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[1][7] = tor_tmp;

    pos_tmp = float_to_uint(motor.getPos(Left_Knee_Pitch), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getVel(Left_Knee_Pitch), V_MIN, V_MAX, 12);
    tor_tmp = float_to_uint(motor.getTor(Left_Knee_Pitch), T_MIN, T_MAX, 12);
    send_data[2][0] = (pos_tmp >> 8);
    send_data[2][1] = pos_tmp;
    send_data[2][2] = (vel_tmp >> 4);
    send_data[2][3] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[2][4] = tor_tmp;

    pos_tmp = float_to_uint(motor.getPos(Right_Hip_Yaw), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getVel(Right_Hip_Yaw), V_MIN, V_MAX, 12);
    tor_tmp = float_to_uint(motor.getTor(Right_Hip_Yaw), T_MIN, T_MAX, 12);
    send_data[2][5] = (vel_tmp >> 4);
    send_data[2][6] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[2][7] = tor_tmp;
    send_data[3][0] = 0x02;
    send_data[3][1] = (pos_tmp >> 8);
    send_data[3][2] = pos_tmp;

    pos_tmp = float_to_uint(motor.getPos(Right_Hip_Roll), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getVel(Right_Hip_Roll), V_MIN, V_MAX, 12);
    tor_tmp = float_to_uint(motor.getTor(Right_Hip_Roll), T_MIN, T_MAX, 12);
    send_data[3][3] = (pos_tmp >> 8);
    send_data[3][4] = pos_tmp;
    send_data[3][5] = (vel_tmp >> 4);
    send_data[3][6] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[3][7] = tor_tmp;

    pos_tmp = float_to_uint(motor.getPos(Right_Hip_Pitch), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getVel(Right_Hip_Pitch), V_MIN, V_MAX, 12);
    tor_tmp = float_to_uint(motor.getTor(Right_Hip_Pitch), T_MIN, T_MAX, 12);
    send_data[4][0] = (pos_tmp >> 8);
    send_data[4][1] = pos_tmp;
    send_data[4][2] = (vel_tmp >> 4);
    send_data[4][3] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[4][4] = tor_tmp;

    pos_tmp = float_to_uint(motor.getPos(Right_Knee_Pitch), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getVel(Right_Knee_Pitch), V_MIN, V_MAX, 12);
    tor_tmp = float_to_uint(motor.getTor(Right_Knee_Pitch), T_MIN, T_MAX, 12);
    send_data[4][5] = (vel_tmp >> 4);
    send_data[4][6] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[4][7] = tor_tmp;
    send_data[5][0] = pos_tmp >> 8;
    send_data[5][1] = pos_tmp;

    can_frame frame = {};
    frame.can_id = CAN_SEND_START_ID;
    frame.can_dlc = PACKAGE_SIZE;
    for (int i = 0; i < SEND_PACKAGE_NUM; i++)
    {
        frame.can_id = CAN_SEND_START_ID + i;
        memcpy(frame.data, send_data[i], PACKAGE_SIZE);
        can.send(frame);
    }
}

void STM32::receiveData(CAN &can, Motor &motor, IMU &imu, Command &command)
{
    can_frame frame = {};
    frame.can_id = CAN_RECEIVE_START_ID;
    frame.can_dlc = PACKAGE_SIZE;
    can.receive(frame);
    switch (frame.can_id)
    {
    case CAN_RECEIVE_START_ID:
        memcpy(receive_data[0], frame.data, PACKAGE_SIZE);
        break;
    case CAN_RECEIVE_START_ID + 1:
        memcpy(receive_data[1], frame.data, PACKAGE_SIZE);
        break;
    case CAN_RECEIVE_START_ID + 2:
        memcpy(receive_data[2], frame.data, PACKAGE_SIZE);
        break;
    case CAN_RECEIVE_START_ID + 3:
        memcpy(receive_data[3], frame.data, PACKAGE_SIZE);
        break;
    case CAN_RECEIVE_START_ID + 4:
        memcpy(receive_data[4], frame.data, PACKAGE_SIZE);
        break;
    case CAN_RECEIVE_START_ID + 5:
        memcpy(receive_data[5], frame.data, PACKAGE_SIZE);
        break;
    case CAN_RECEIVE_START_ID + 6:
        memcpy(receive_data[6], frame.data, PACKAGE_SIZE);
        break;
    }

    uint16_t pos_tmp, vel_tmp, tor_tmp;
    pos_tmp = receive_data[0][1] << 8 | receive_data[0][2];
    vel_tmp = receive_data[0][3] << 4 | receive_data[0][4] >> 4;
    tor_tmp = (receive_data[0][4] & 0xF) << 8 | receive_data[0][5];
    motor.setPos(Left_Hip_Yaw, uint_to_float(pos_tmp, P_MIN, P_MAX, 16));
    motor.setVel(Left_Hip_Yaw, uint_to_float(vel_tmp, V_MIN, V_MAX, 12));
    motor.setTor(Left_Hip_Yaw, uint_to_float(tor_tmp, T_MIN, T_MAX, 12));

    pos_tmp = receive_data[0][6] << 8 | receive_data[0][7];
    vel_tmp = receive_data[1][0] << 4 | receive_data[1][1] >> 4;
    tor_tmp = (receive_data[1][1] & 0xF) << 8 | receive_data[1][2];
    motor.setPos(Left_Hip_Roll, uint_to_float(pos_tmp, P_MIN, P_MAX, 16));
    motor.setVel(Left_Hip_Roll, uint_to_float(vel_tmp, V_MIN, V_MAX, 12));
    motor.setTor(Left_Hip_Roll, uint_to_float(tor_tmp, T_MIN, T_MAX, 12));

    pos_tmp = receive_data[1][3] << 8 | receive_data[1][4];
    vel_tmp = receive_data[1][5] << 4 | receive_data[1][6] >> 4;
    tor_tmp = (receive_data[1][6] & 0xF) << 8 | receive_data[1][7];
    motor.setPos(Left_Hip_Pitch, uint_to_float(pos_tmp, P_MIN, P_MAX, 16));
    motor.setVel(Left_Hip_Pitch, uint_to_float(vel_tmp, V_MIN, V_MAX, 12));
    motor.setTor(Left_Hip_Pitch, uint_to_float(tor_tmp, T_MIN, T_MAX, 12));

    pos_tmp = receive_data[2][0] << 8 | receive_data[2][1];
    vel_tmp = receive_data[2][2] << 4 | receive_data[2][3] >> 4;
    tor_tmp = (receive_data[2][3] & 0xF) << 8 | receive_data[2][4];
    motor.setPos(Left_Knee_Pitch, uint_to_float(pos_tmp, P_MIN, P_MAX, 16));
    motor.setVel(Left_Knee_Pitch, uint_to_float(vel_tmp, V_MIN, V_MAX, 12));
    motor.setTor(Left_Knee_Pitch, uint_to_float(tor_tmp, T_MIN, T_MAX, 12));

    pos_tmp = receive_data[3][1] << 8 | receive_data[3][2];
    vel_tmp = receive_data[2][5] << 4 | receive_data[2][6] >> 4;
    tor_tmp = (receive_data[2][6] & 0xF) << 8 | receive_data[2][7];
    motor.setPos(Right_Hip_Yaw, uint_to_float(pos_tmp, P_MIN, P_MAX, 16));
    motor.setVel(Right_Hip_Yaw, uint_to_float(vel_tmp, V_MIN, V_MAX, 12));
    motor.setTor(Right_Hip_Yaw, uint_to_float(tor_tmp, T_MIN, T_MAX, 12));

    pos_tmp = receive_data[3][3] << 8 | receive_data[3][4];
    vel_tmp = receive_data[3][5] << 4 | receive_data[3][6] >> 4;
    tor_tmp = (receive_data[3][6] & 0xF) << 8 | receive_data[3][7];
    motor.setPos(Right_Hip_Roll, uint_to_float(pos_tmp, P_MIN, P_MAX, 16));
    motor.setVel(Right_Hip_Roll, uint_to_float(vel_tmp, V_MIN, V_MAX, 12));
    motor.setTor(Right_Hip_Roll, uint_to_float(tor_tmp, T_MIN, T_MAX, 12));

    pos_tmp = receive_data[4][0] << 8 | receive_data[4][1];
    vel_tmp = receive_data[4][2] << 4 | receive_data[4][3] >> 4;
    tor_tmp = (receive_data[4][3] & 0xF) << 8 | receive_data[4][4];
    motor.setPos(Right_Hip_Pitch, uint_to_float(pos_tmp, P_MIN, P_MAX, 16));
    motor.setVel(Right_Hip_Pitch, uint_to_float(vel_tmp, V_MIN, V_MAX, 12));
    motor.setTor(Right_Hip_Pitch, uint_to_float(tor_tmp, T_MIN, T_MAX, 12));

    pos_tmp = receive_data[5][0] << 8 | receive_data[5][1];
    vel_tmp = receive_data[4][5] << 4 | receive_data[4][6] >> 4;
    tor_tmp = (receive_data[4][6] & 0xF) << 8 | receive_data[4][7];
    motor.setPos(Right_Knee_Pitch, uint_to_float(pos_tmp, P_MIN, P_MAX, 16));
    motor.setVel(Right_Knee_Pitch, uint_to_float(vel_tmp, V_MIN, V_MAX, 12));
    motor.setTor(Right_Knee_Pitch, uint_to_float(tor_tmp, T_MIN, T_MAX, 12));

    uint8_t vx_tmp, vy_tmp, vz_tmp, wz_tmp;
    vx_tmp = receive_data[5][2] >> 4;
    vy_tmp = receive_data[5][2] & 0xF;
    vz_tmp = receive_data[5][3] >> 4;
    wz_tmp = receive_data[5][3] & 0xF;
    command.setLinearVel({uint_to_float(vx_tmp, LINEAR_VEL_MIN, LINEAR_VEL_MAX, 4), uint_to_float(vy_tmp, LINEAR_VEL_MIN, LINEAR_VEL_MAX, 4), uint_to_float(vz_tmp, LINEAR_VEL_MIN, LINEAR_VEL_MAX, 4)});
    command.setAngularVel({0.0f, 0.0f, uint_to_float(wz_tmp, ANGULAR_VEL_MIN, ANGULAR_VEL_MAX, 4)});

    uint16_t acc_x_tmp, acc_y_tmp, acc_z_tmp, gyro_x_tmp, gyro_y_tmp, gyro_z_tmp;
    acc_x_tmp = receive_data[5][4] << 8 | receive_data[5][5];
    acc_y_tmp = receive_data[5][6] << 8 | receive_data[5][7];
    acc_z_tmp = receive_data[6][0] << 8 | receive_data[6][1];
    gyro_x_tmp = receive_data[6][2] << 8 | receive_data[6][3];
    gyro_y_tmp = receive_data[6][4] << 8 | receive_data[6][5];
    gyro_z_tmp = receive_data[6][6] << 8 | receive_data[6][7];
    imu.setAccel({uint_to_float(acc_x_tmp, A_MIN, A_MAX, 16), uint_to_float(acc_y_tmp, A_MIN, A_MAX, 16), uint_to_float(acc_z_tmp, A_MIN, A_MAX, 16)});
    imu.setGyro({uint_to_float(gyro_x_tmp, G_MIN, G_MAX, 16), uint_to_float(gyro_y_tmp, G_MIN, G_MAX, 16), uint_to_float(gyro_z_tmp, G_MIN, G_MAX, 16)});
}
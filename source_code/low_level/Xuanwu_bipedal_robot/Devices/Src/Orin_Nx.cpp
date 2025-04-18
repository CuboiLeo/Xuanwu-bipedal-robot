#include "Orin_NX.h"
#include <string.h>

void Orin::sendData(const Motor &motor, const IMU &imu, const Direction_Vector &ref_vel, const Direction_Vector &ref_angular_vel)
{
    uint16_t pos_tmp, vel_tmp, tor_tmp;
    pos_tmp = float_to_uint(motor.getPos(Left_Hip_Yaw), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getVel(Left_Hip_Yaw), V_MIN, V_MAX, 12);
    tor_tmp = float_to_uint(motor.getTor(Left_Hip_Yaw), T_MIN, T_MAX, 12);
    send_data[0][0] = 0x01; // reserved bit 1
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

    pos_tmp = float_to_uint(motor.getPos(Left_Ankle_Pitch), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getVel(Left_Ankle_Pitch), V_MIN, V_MAX, 12);
    tor_tmp = float_to_uint(motor.getTor(Left_Ankle_Pitch), T_MIN, T_MAX, 12);
    send_data[2][5] = (vel_tmp >> 4);
    send_data[2][6] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[2][7] = tor_tmp;
    send_data[3][0] = 0x02; // reserved bit 2
    send_data[3][1] = (pos_tmp >> 8);
    send_data[3][2] = pos_tmp;

    pos_tmp = float_to_uint(motor.getPos(Right_Hip_Yaw), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getVel(Right_Hip_Yaw), V_MIN, V_MAX, 12);
    tor_tmp = float_to_uint(motor.getTor(Right_Hip_Yaw), T_MIN, T_MAX, 12);
    send_data[3][3] = (pos_tmp >> 8);
    send_data[3][4] = pos_tmp;
    send_data[3][5] = (vel_tmp >> 4);
    send_data[3][6] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[3][7] = tor_tmp;

    pos_tmp = float_to_uint(motor.getPos(Right_Hip_Roll), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getVel(Right_Hip_Roll), V_MIN, V_MAX, 12);
    tor_tmp = float_to_uint(motor.getTor(Right_Hip_Roll), T_MIN, T_MAX, 12);
    send_data[4][0] = (pos_tmp >> 8);
    send_data[4][1] = pos_tmp;
    send_data[4][2] = (vel_tmp >> 4);
    send_data[4][3] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[4][4] = tor_tmp;

    pos_tmp = float_to_uint(motor.getPos(Right_Hip_Pitch), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getVel(Right_Hip_Pitch), V_MIN, V_MAX, 12);
    tor_tmp = float_to_uint(motor.getTor(Right_Hip_Pitch), T_MIN, T_MAX, 12);
    send_data[4][5] = (vel_tmp >> 4);
    send_data[4][6] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[4][7] = tor_tmp;
    send_data[5][0] = 0x03; // reserved bit 3
    send_data[5][1] = pos_tmp >> 8;
    send_data[5][2] = pos_tmp;

    pos_tmp = float_to_uint(motor.getPos(Right_Knee_Pitch), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getVel(Right_Knee_Pitch), V_MIN, V_MAX, 12);
    tor_tmp = float_to_uint(motor.getTor(Right_Knee_Pitch), T_MIN, T_MAX, 12);
    send_data[5][3] = (pos_tmp >> 8);
    send_data[5][4] = pos_tmp;
    send_data[5][5] = (vel_tmp >> 4);
    send_data[5][6] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[5][7] = tor_tmp;

    pos_tmp = float_to_uint(motor.getPos(Right_Ankle_Pitch), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getVel(Right_Ankle_Pitch), V_MIN, V_MAX, 12);
    tor_tmp = float_to_uint(motor.getTor(Right_Ankle_Pitch), T_MIN, T_MAX, 12);
    send_data[6][0] = (pos_tmp >> 8);
    send_data[6][1] = pos_tmp;
    send_data[6][2] = (vel_tmp >> 4);
    send_data[6][3] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[6][4] = tor_tmp;

    uint8_t vx, vy, wz;
    vx = float_to_uint(ref_vel.x, -ROBOT_MAX_VEL, ROBOT_MAX_VEL, 8);
    vy = float_to_uint(ref_vel.y, -ROBOT_MAX_VEL, ROBOT_MAX_VEL, 8);
    wz = float_to_uint(ref_angular_vel.z, -ROBOT_MAX_ANG_VEL, ROBOT_MAX_ANG_VEL, 8);
    send_data[6][5] = vx;
    send_data[6][6] = vy;
    send_data[6][7] = wz;

    uint16_t acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
    FusionVector acc = imu.getAccel();
    FusionVector gyro = imu.getGyro();
    acc_x = float_to_uint(acc.axis.x, -3.0f * GRAVITY, 3.0f * GRAVITY, 16);
    acc_y = float_to_uint(acc.axis.y, -3.0f * GRAVITY, 3.0f * GRAVITY, 16);
    acc_z = float_to_uint(acc.axis.z, -3.0f * GRAVITY, 3.0f * GRAVITY, 16);
    gyro_x = float_to_uint(gyro.axis.x, -2000.0f * DEG2RAD, 2000.0f * DEG2RAD, 16);
    gyro_y = float_to_uint(gyro.axis.y, -2000.0f * DEG2RAD, 2000.0f * DEG2RAD, 16);
    gyro_z = float_to_uint(gyro.axis.z, -2000.0f * DEG2RAD, 2000.0f * DEG2RAD, 16);
    send_data[7][0] = acc_x >> 8;
    send_data[7][1] = acc_x;
    send_data[7][2] = acc_y >> 8;
    send_data[7][3] = acc_y;
    send_data[7][4] = acc_z >> 8;
    send_data[7][5] = acc_z;
    send_data[7][6] = gyro_x >> 8;
    send_data[7][7] = gyro_x;
    send_data[8][0] = gyro_y >> 8;
    send_data[8][1] = gyro_y;
    send_data[8][2] = gyro_z >> 8;
    send_data[8][3] = gyro_z;

    if (watchdog > 0)
    {
        watchdog--;
        for (uint8_t i = 0, n = CAN_SEND_START_ID; i < SEND_PACKAGE_NUM; i++)
        {
            fdcanx_send_data(&hfdcan3, n + i, send_data[i], sizeof(send_data[i]));
        }
    }
    else
    {
        watchdog = 0;
    }
}

void Orin::sendDataLite(const Motor &motor, const IMU &imu, const Direction_Vector &ref_vel, const Direction_Vector &ref_angular_vel)
{
    // only send motor's pos and vel to reduce package size for faster communication
    uint16_t pos_tmp, vel_tmp;
    pos_tmp = float_to_uint(motor.getPos(Left_Hip_Yaw), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getVel(Left_Hip_Yaw), V_MIN, V_MAX, 12);
    send_data_lite[0][0] = (pos_tmp >> 8);
    send_data_lite[0][1] = pos_tmp;
    send_data_lite[0][2] = (vel_tmp >> 4);
    send_data_lite[0][3] = ((vel_tmp & 0xF) << 4);

    pos_tmp = float_to_uint(motor.getPos(Left_Hip_Roll), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getVel(Left_Hip_Roll), V_MIN, V_MAX, 12);
    send_data_lite[0][4] = (pos_tmp >> 8);
    send_data_lite[0][5] = pos_tmp;
    send_data_lite[0][6] = (vel_tmp >> 4);
    send_data_lite[0][7] = ((vel_tmp & 0xF) << 4);

    pos_tmp = float_to_uint(motor.getPos(Left_Hip_Pitch), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getVel(Left_Hip_Pitch), V_MIN, V_MAX, 12);
    send_data_lite[1][0] = (pos_tmp >> 8);
    send_data_lite[1][1] = pos_tmp;
    send_data_lite[1][2] = (vel_tmp >> 4);
    send_data_lite[1][3] = ((vel_tmp & 0xF) << 4);

    pos_tmp = float_to_uint(motor.getPos(Left_Knee_Pitch), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getVel(Left_Knee_Pitch), V_MIN, V_MAX, 12);
    send_data_lite[1][4] = (pos_tmp >> 8);
    send_data_lite[1][5] = pos_tmp;
    send_data_lite[1][6] = (vel_tmp >> 4);
    send_data_lite[1][7] = ((vel_tmp & 0xF) << 4);

    pos_tmp = float_to_uint(motor.getPos(Left_Ankle_Pitch), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getVel(Left_Ankle_Pitch), V_MIN, V_MAX, 12);
    send_data_lite[2][0] = (pos_tmp >> 8);
    send_data_lite[2][1] = pos_tmp;
    send_data_lite[2][2] = (vel_tmp >> 4);
    send_data_lite[2][3] = ((vel_tmp & 0xF) << 4);

    pos_tmp = float_to_uint(motor.getPos(Right_Hip_Yaw), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getVel(Right_Hip_Yaw), V_MIN, V_MAX, 12);
    send_data_lite[2][4] = (pos_tmp >> 8);
    send_data_lite[2][5] = pos_tmp;
    send_data_lite[2][6] = (vel_tmp >> 4);
    send_data_lite[2][7] = ((vel_tmp & 0xF) << 4);

    pos_tmp = float_to_uint(motor.getPos(Right_Hip_Roll), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getVel(Right_Hip_Roll), V_MIN, V_MAX, 12);
    send_data_lite[3][0] = (pos_tmp >> 8);
    send_data_lite[3][1] = pos_tmp;
    send_data_lite[3][2] = (vel_tmp >> 4);
    send_data_lite[3][3] = ((vel_tmp & 0xF) << 4);

    pos_tmp = float_to_uint(motor.getPos(Right_Hip_Pitch), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getVel(Right_Hip_Pitch), V_MIN, V_MAX, 12);
    send_data_lite[3][4] = (pos_tmp >> 8);
    send_data_lite[3][5] = pos_tmp;
    send_data_lite[3][6] = (vel_tmp >> 4);
    send_data_lite[3][7] = ((vel_tmp & 0xF) << 4);

    pos_tmp = float_to_uint(motor.getPos(Right_Knee_Pitch), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getVel(Right_Knee_Pitch), V_MIN, V_MAX, 12);
    send_data_lite[4][0] = (pos_tmp >> 8);
    send_data_lite[4][1] = pos_tmp;
    send_data_lite[4][2] = (vel_tmp >> 4);
    send_data_lite[4][3] = ((vel_tmp & 0xF) << 4);

    pos_tmp = float_to_uint(motor.getPos(Right_Ankle_Pitch), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getVel(Right_Ankle_Pitch), V_MIN, V_MAX, 12);
    send_data_lite[4][4] = (pos_tmp >> 8);
    send_data_lite[4][5] = pos_tmp;
    send_data_lite[4][6] = (vel_tmp >> 4);
    send_data_lite[4][7] = ((vel_tmp & 0xF) << 4);

    uint8_t vx, vy, wz;
    vx = float_to_uint(ref_vel.x, -ROBOT_MAX_VEL, ROBOT_MAX_VEL, 8);
    vy = float_to_uint(ref_vel.y, -ROBOT_MAX_VEL, ROBOT_MAX_VEL, 8);
    wz = float_to_uint(ref_angular_vel.z, -ROBOT_MAX_ANG_VEL, ROBOT_MAX_ANG_VEL, 8);
    send_data_lite[5][0] = vx;
    send_data_lite[5][1] = vy;
    send_data_lite[5][2] = wz;

    uint16_t acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
    FusionVector acc = imu.getAccel();
    FusionVector gyro = imu.getGyro();
    acc_x = float_to_uint(acc.axis.x, -3.0f * GRAVITY, 3.0f * GRAVITY, 16);
    acc_y = float_to_uint(acc.axis.y, -3.0f * GRAVITY, 3.0f * GRAVITY, 16);
    acc_z = float_to_uint(acc.axis.z, -3.0f * GRAVITY, 3.0f * GRAVITY, 16);
    gyro_x = float_to_uint(gyro.axis.x, -2000.0f * DEG2RAD, 2000.0f * DEG2RAD, 16);
    gyro_y = float_to_uint(gyro.axis.y, -2000.0f * DEG2RAD, 2000.0f * DEG2RAD, 16);
    gyro_z = float_to_uint(gyro.axis.z, -2000.0f * DEG2RAD, 2000.0f * DEG2RAD, 16);
    send_data_lite[5][3] = acc_x >> 8;
    send_data_lite[5][4] = acc_x;
    send_data_lite[5][5] = acc_y >> 8;
    send_data_lite[5][6] = acc_y;
    send_data_lite[5][7] = acc_z >> 8;
    send_data_lite[6][0] = acc_z;
    send_data_lite[6][1] = gyro_x >> 8;
    send_data_lite[6][2] = gyro_x;
    send_data_lite[6][3] = gyro_y >> 8;
    send_data_lite[6][4] = gyro_y;
    send_data_lite[6][5] = gyro_z >> 8;
    send_data_lite[6][6] = gyro_z;

    if (watchdog > 0)
    {
        watchdog--;
        for (uint8_t i = 0, n = CAN_SEND_START_ID; i < SEND_PACKAGE_NUM_LITE; i++)
        {
            fdcanx_send_data(&hfdcan3, n + i, send_data_lite[i], sizeof(send_data_lite[i]));
        }
    }
    else
    {
        watchdog = 0;
        received_data_flag = 0;
    }
}

void Orin::receiveData(void)
{
    watchdog = 10;
    uint16_t rec_id;
    uint8_t rx_data[8] = {0};
    fdcanx_receive(&hfdcan3, &rec_id, rx_data);
    switch (rec_id)
    {
    case CAN_RECEIVE_START_ID:
        memcpy(receive_data[0], rx_data, sizeof(rx_data));
        break;
    case CAN_RECEIVE_START_ID + 1:
        memcpy(receive_data[1], rx_data, sizeof(rx_data));
        break;
    case CAN_RECEIVE_START_ID + 2:
        memcpy(receive_data[2], rx_data, sizeof(rx_data));
        break;
    case CAN_RECEIVE_START_ID + 3:
        memcpy(receive_data[3], rx_data, sizeof(rx_data));
        break;
    case CAN_RECEIVE_START_ID + 4:
        memcpy(receive_data[4], rx_data, sizeof(rx_data));
        break;
    case CAN_RECEIVE_START_ID + 5:
        memcpy(receive_data[5], rx_data, sizeof(rx_data));
        break;
    case CAN_RECEIVE_START_ID + 6:
        memcpy(receive_data[6], rx_data, sizeof(rx_data));
        break;
    }
}

void Orin::receiveDataLite(void)
{
    watchdog = 10;
    received_data_flag = 1;
    uint16_t rec_id;
    uint8_t rx_data[8] = {0};
    fdcanx_receive(&hfdcan3, &rec_id, rx_data);
    switch (rec_id)
    {
    case CAN_RECEIVE_START_ID:
        memcpy(receive_data_lite[0], rx_data, sizeof(rx_data));
        break;
    case CAN_RECEIVE_START_ID + 1:
        memcpy(receive_data_lite[1], rx_data, sizeof(rx_data));
        break;
    case CAN_RECEIVE_START_ID + 2:
        memcpy(receive_data_lite[2], rx_data, sizeof(rx_data));
        break;
    }
}

void Orin::decodeData(Motor &motor)
{
    if (receive_data[0][0] == 0x01)
    {
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
        motor.setPos(Left_Ankle_Pitch, uint_to_float(pos_tmp, P_MIN, P_MAX, 16));
        motor.setVel(Left_Ankle_Pitch, uint_to_float(vel_tmp, V_MIN, V_MAX, 12));
        motor.setTor(Left_Ankle_Pitch, uint_to_float(tor_tmp, T_MIN, T_MAX, 12));

        pos_tmp = receive_data[3][3] << 8 | receive_data[3][4];
        vel_tmp = receive_data[3][5] << 4 | receive_data[3][6] >> 4;
        tor_tmp = (receive_data[3][6] & 0xF) << 8 | receive_data[3][7];
        motor.setPos(Right_Hip_Yaw, uint_to_float(pos_tmp, P_MIN, P_MAX, 16));
        motor.setVel(Right_Hip_Yaw, uint_to_float(vel_tmp, V_MIN, V_MAX, 12));
        motor.setTor(Right_Hip_Yaw, uint_to_float(tor_tmp, T_MIN, T_MAX, 12));

        pos_tmp = receive_data[4][0] << 8 | receive_data[4][1];
        vel_tmp = receive_data[4][2] << 4 | receive_data[4][3] >> 4;
        tor_tmp = (receive_data[4][3] & 0xF) << 8 | receive_data[4][4];
        motor.setPos(Right_Hip_Roll, uint_to_float(pos_tmp, P_MIN, P_MAX, 16));
        motor.setVel(Right_Hip_Roll, uint_to_float(vel_tmp, V_MIN, V_MAX, 12));
        motor.setTor(Right_Hip_Roll, uint_to_float(tor_tmp, T_MIN, T_MAX, 12));

        pos_tmp = receive_data[5][1] << 8 | receive_data[5][2];
        vel_tmp = receive_data[4][5] << 4 | receive_data[4][6] >> 4;
        tor_tmp = (receive_data[4][6] & 0xF) << 8 | receive_data[4][7];
        motor.setPos(Right_Hip_Pitch, uint_to_float(pos_tmp, P_MIN, P_MAX, 16));
        motor.setVel(Right_Hip_Pitch, uint_to_float(vel_tmp, V_MIN, V_MAX, 12));
        motor.setTor(Right_Hip_Pitch, uint_to_float(tor_tmp, T_MIN, T_MAX, 12));

        pos_tmp = receive_data[5][3] << 8 | receive_data[5][4];
        vel_tmp = receive_data[5][5] << 4 | receive_data[5][6] >> 4;
        tor_tmp = (receive_data[5][6] & 0xF) << 8 | receive_data[5][7];
        motor.setPos(Right_Knee_Pitch, uint_to_float(pos_tmp, P_MIN, P_MAX, 16));
        motor.setVel(Right_Knee_Pitch, uint_to_float(vel_tmp, V_MIN, V_MAX, 12));
        motor.setTor(Right_Knee_Pitch, uint_to_float(tor_tmp, T_MIN, T_MAX, 12));

        pos_tmp = receive_data[6][0] << 8 | receive_data[6][1];
        vel_tmp = receive_data[6][2] << 4 | receive_data[6][3] >> 4;
        tor_tmp = (receive_data[6][3] & 0xF) << 8 | receive_data[6][4];
        motor.setPos(Right_Ankle_Pitch, uint_to_float(pos_tmp, P_MIN, P_MAX, 16));
        motor.setVel(Right_Ankle_Pitch, uint_to_float(vel_tmp, V_MIN, V_MAX, 12));
        motor.setTor(Right_Ankle_Pitch, uint_to_float(tor_tmp, T_MIN, T_MAX, 12));
    }
}

void Orin::decodeDataLite(Motor &motor)
{
    if (receive_data_lite[0][0] == 0x01)
    {
        // only decode motor's pos
        uint16_t pos_tmp;
        float pos, tor;
        pos_tmp = receive_data_lite[0][1] << 8 | receive_data_lite[0][2];
        pos = uint_to_float(pos_tmp, P_MIN, P_MAX, 16);
				pos = 0;
        tor = kps[Left_Hip_Yaw] * (pos - motor.getPos(Left_Hip_Yaw)) + kds[Left_Hip_Yaw] * (0 - motor.getVel(Left_Hip_Yaw));
        tor = CLIP(tor, -tor_limit, tor_limit);
        motor.setPos(Left_Hip_Yaw, pos);
        motor.setVel(Left_Hip_Yaw, 0.0f);
        motor.setTor(Left_Hip_Yaw, tor);

        pos_tmp = receive_data_lite[0][3] << 8 | receive_data_lite[0][4];
        pos = uint_to_float(pos_tmp, P_MIN, P_MAX, 16);
				pos = 0;
        tor = kps[Left_Hip_Roll] * (pos - motor.getPos(Left_Hip_Roll)) + kds[Left_Hip_Roll] * (0 - motor.getVel(Left_Hip_Roll));
        tor = CLIP(tor, -tor_limit, tor_limit);
        motor.setPos(Left_Hip_Roll, pos);
        motor.setVel(Left_Hip_Roll, 0.0f);
        motor.setTor(Left_Hip_Roll, tor);

        pos_tmp = receive_data_lite[0][5] << 8 | receive_data_lite[0][6];
        pos = uint_to_float(pos_tmp, P_MIN, P_MAX, 16);
				pos = 0.152f;
        tor = kps[Left_Hip_Pitch] * (pos - motor.getPos(Left_Hip_Pitch)) + kds[Left_Hip_Pitch] * (0 - motor.getVel(Left_Hip_Pitch));
        tor = CLIP(tor, -tor_limit, tor_limit);
        motor.setPos(Left_Hip_Pitch, pos);
        motor.setVel(Left_Hip_Pitch, 0.0f);
        motor.setTor(Left_Hip_Pitch, tor);

        pos_tmp = receive_data_lite[0][7] << 8 | receive_data_lite[1][0];
        pos = uint_to_float(pos_tmp, P_MIN, P_MAX, 16);
				pos = -0.36f;
        tor = kps[Left_Knee_Pitch] * (pos - motor.getPos(Left_Knee_Pitch)) + kds[Left_Knee_Pitch] * (0 - motor.getVel(Left_Knee_Pitch));
        tor = CLIP(tor, -tor_limit, tor_limit);
        motor.setPos(Left_Knee_Pitch, pos);
        motor.setVel(Left_Knee_Pitch, 0.0f);
        motor.setTor(Left_Knee_Pitch, tor);

        pos_tmp = receive_data_lite[1][1] << 8 | receive_data_lite[1][2];
        pos = uint_to_float(pos_tmp, P_MIN, P_MAX, 16);
				pos = 0.208f;
        tor = kps[Left_Ankle_Pitch] * (pos - motor.getPos(Left_Ankle_Pitch)) + kds[Left_Ankle_Pitch] * (0 - motor.getVel(Left_Ankle_Pitch));
        tor = CLIP(tor, -tor_limit, tor_limit);
        motor.setPos(Left_Ankle_Pitch, pos);
        motor.setVel(Left_Ankle_Pitch, 0.0f);
        motor.setTor(Left_Ankle_Pitch, tor);

        pos_tmp = receive_data_lite[1][3] << 8 | receive_data_lite[1][4];
        pos = uint_to_float(pos_tmp, P_MIN, P_MAX, 16);
				pos = 0;
        tor = kps[Right_Hip_Yaw] * (pos - motor.getPos(Right_Hip_Yaw)) + kds[Right_Hip_Yaw] * (0 - motor.getVel(Right_Hip_Yaw));
        tor = CLIP(tor, -tor_limit, tor_limit);
        motor.setPos(Right_Hip_Yaw, pos);
        motor.setVel(Right_Hip_Yaw, 0.0f);
        motor.setTor(Right_Hip_Yaw, tor);

        pos_tmp = receive_data_lite[1][5] << 8 | receive_data_lite[1][6];
        pos = uint_to_float(pos_tmp, P_MIN, P_MAX, 16);
				pos = 0;
        tor = kps[Right_Hip_Roll] * (pos - motor.getPos(Right_Hip_Roll)) + kds[Right_Hip_Roll] * (0 - motor.getVel(Right_Hip_Roll));
        tor = CLIP(tor, -tor_limit, tor_limit);
        motor.setPos(Right_Hip_Roll, pos);
        motor.setVel(Right_Hip_Roll, 0.0f);
        motor.setTor(Right_Hip_Roll, tor);

        pos_tmp = receive_data_lite[1][7] << 8 | receive_data_lite[2][0];
        pos = uint_to_float(pos_tmp, P_MIN, P_MAX, 16);
				pos = 0.152f;
        tor = kps[Right_Hip_Pitch] * (pos - motor.getPos(Right_Hip_Pitch)) + kds[Right_Hip_Pitch] * (0 - motor.getVel(Right_Hip_Pitch));
        tor = CLIP(tor, -tor_limit, tor_limit);
        motor.setPos(Right_Hip_Pitch, pos);
        motor.setVel(Right_Hip_Pitch, 0.0f);
        motor.setTor(Right_Hip_Pitch, tor);

        pos_tmp = receive_data_lite[2][1] << 8 | receive_data_lite[2][2];
        pos = uint_to_float(pos_tmp, P_MIN, P_MAX, 16);
				pos = -0.36f;
        tor = kps[Right_Knee_Pitch] * (pos - motor.getPos(Right_Knee_Pitch)) + kds[Right_Knee_Pitch] * (0 - motor.getVel(Right_Knee_Pitch));
        tor = CLIP(tor, -tor_limit, tor_limit);
        motor.setPos(Right_Knee_Pitch, pos);
        motor.setVel(Right_Knee_Pitch, 0.0f);
        motor.setTor(Right_Knee_Pitch, tor);

        pos_tmp = receive_data_lite[2][3] << 8 | receive_data_lite[2][4];
        pos = uint_to_float(pos_tmp, P_MIN, P_MAX, 16);
				pos = 0.208f;
        tor = kps[Right_Ankle_Pitch] * (pos - motor.getPos(Right_Ankle_Pitch)) + kds[Right_Ankle_Pitch] * (0 - motor.getVel(Right_Ankle_Pitch));
        tor = CLIP(tor, -tor_limit, tor_limit);
        motor.setPos(Right_Ankle_Pitch, pos);
        motor.setVel(Right_Ankle_Pitch, 0.0f);
        motor.setTor(Right_Ankle_Pitch, tor);
    }
}
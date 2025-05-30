#include "STM32_protocol.h"

void STM32::encodeData(Motor &motor)
{
    uint16_t pos_tmp, vel_tmp, tor_tmp;
    pos_tmp = float_to_uint(motor.getRefPos(Left_Hip_Yaw), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getRefVel(Left_Hip_Yaw), V_MIN, V_MAX, 12);
    tor_tmp = float_to_uint(motor.getRefTor(Left_Hip_Yaw), T_MIN, T_MAX, 12);
    send_data[0][0] = 0x01; // reserved bit 1
    send_data[0][1] = (pos_tmp >> 8);
    send_data[0][2] = pos_tmp;
    send_data[0][3] = (vel_tmp >> 4);
    send_data[0][4] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[0][5] = tor_tmp;

    pos_tmp = float_to_uint(motor.getRefPos(Left_Hip_Roll), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getRefVel(Left_Hip_Roll), V_MIN, V_MAX, 12);
    tor_tmp = float_to_uint(motor.getRefTor(Left_Hip_Roll), T_MIN, T_MAX, 12);
    send_data[0][6] = (pos_tmp >> 8);
    send_data[0][7] = pos_tmp;
    send_data[1][0] = (vel_tmp >> 4);
    send_data[1][1] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[1][2] = tor_tmp;

    pos_tmp = float_to_uint(motor.getRefPos(Left_Hip_Pitch), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getRefVel(Left_Hip_Pitch), V_MIN, V_MAX, 12);
    tor_tmp = float_to_uint(motor.getRefTor(Left_Hip_Pitch), T_MIN, T_MAX, 12);
    send_data[1][3] = (pos_tmp >> 8);
    send_data[1][4] = pos_tmp;
    send_data[1][5] = (vel_tmp >> 4);
    send_data[1][6] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[1][7] = tor_tmp;

    pos_tmp = float_to_uint(motor.getRefPos(Left_Knee_Pitch), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getRefVel(Left_Knee_Pitch), V_MIN, V_MAX, 12);
    tor_tmp = float_to_uint(motor.getRefTor(Left_Knee_Pitch), T_MIN, T_MAX, 12);
    send_data[2][0] = (pos_tmp >> 8);
    send_data[2][1] = pos_tmp;
    send_data[2][2] = (vel_tmp >> 4);
    send_data[2][3] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[2][4] = tor_tmp;

    pos_tmp = float_to_uint(motor.getRefPos(Left_Ankle_Pitch), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getRefVel(Left_Ankle_Pitch), V_MIN, V_MAX, 12);
    tor_tmp = float_to_uint(motor.getRefTor(Left_Ankle_Pitch), T_MIN, T_MAX, 12);
    send_data[2][5] = (vel_tmp >> 4);
    send_data[2][6] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[2][7] = tor_tmp;
    send_data[3][0] = 0x02; // reserved bit 2
    send_data[3][1] = (pos_tmp >> 8);
    send_data[3][2] = pos_tmp;

    pos_tmp = float_to_uint(motor.getRefPos(Right_Hip_Yaw), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getRefVel(Right_Hip_Yaw), V_MIN, V_MAX, 12);
    tor_tmp = float_to_uint(motor.getRefTor(Right_Hip_Yaw), T_MIN, T_MAX, 12);
    send_data[3][3] = (pos_tmp >> 8);
    send_data[3][4] = pos_tmp;
    send_data[3][5] = (vel_tmp >> 4);
    send_data[3][6] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[3][7] = tor_tmp;

    pos_tmp = float_to_uint(motor.getRefPos(Right_Hip_Roll), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getRefVel(Right_Hip_Roll), V_MIN, V_MAX, 12);
    tor_tmp = float_to_uint(motor.getRefTor(Right_Hip_Roll), T_MIN, T_MAX, 12);
    send_data[4][0] = (pos_tmp >> 8);
    send_data[4][1] = pos_tmp;
    send_data[4][2] = (vel_tmp >> 4);
    send_data[4][3] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[4][4] = tor_tmp;

    pos_tmp = float_to_uint(motor.getRefPos(Right_Hip_Pitch), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getRefVel(Right_Hip_Pitch), V_MIN, V_MAX, 12);
    tor_tmp = float_to_uint(motor.getRefTor(Right_Hip_Pitch), T_MIN, T_MAX, 12);
    send_data[4][5] = (vel_tmp >> 4);
    send_data[4][6] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[4][7] = tor_tmp;
    send_data[5][0] = 0x03; // reserved bit 3
    send_data[5][1] = pos_tmp >> 8;
    send_data[5][2] = pos_tmp;

    pos_tmp = float_to_uint(motor.getRefPos(Right_Knee_Pitch), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getRefVel(Right_Knee_Pitch), V_MIN, V_MAX, 12);
    tor_tmp = float_to_uint(motor.getRefTor(Right_Knee_Pitch), T_MIN, T_MAX, 12);
    send_data[5][3] = (pos_tmp >> 8);
    send_data[5][4] = pos_tmp;
    send_data[5][5] = (vel_tmp >> 4);
    send_data[5][6] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[5][7] = tor_tmp;

    pos_tmp = float_to_uint(motor.getRefPos(Right_Ankle_Pitch), P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor.getRefVel(Right_Ankle_Pitch), V_MIN, V_MAX, 12);
    tor_tmp = float_to_uint(motor.getRefTor(Right_Ankle_Pitch), T_MIN, T_MAX, 12);
    send_data[6][0] = (pos_tmp >> 8);
    send_data[6][1] = pos_tmp;
    send_data[6][2] = (vel_tmp >> 4);
    send_data[6][3] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);
    send_data[6][4] = tor_tmp;
}

void STM32::encodeDataLite(Motor &motor)
{
    // only send position
    uint16_t pos_tmp;
    pos_tmp = float_to_uint(motor.getRefPos(Left_Hip_Yaw), P_MIN, P_MAX, 16);
    send_data_lite[0][0] = 0x01; // reserved bit 1
    send_data_lite[0][1] = (pos_tmp >> 8);
    send_data_lite[0][2] = pos_tmp;

    pos_tmp = float_to_uint(motor.getRefPos(Left_Hip_Roll), P_MIN, P_MAX, 16);
    send_data_lite[0][3] = (pos_tmp >> 8);
    send_data_lite[0][4] = pos_tmp;

    pos_tmp = float_to_uint(motor.getRefPos(Left_Hip_Pitch), P_MIN, P_MAX, 16);
    send_data_lite[0][5] = (pos_tmp >> 8);
    send_data_lite[0][6] = pos_tmp;

    pos_tmp = float_to_uint(motor.getRefPos(Left_Knee_Pitch), P_MIN, P_MAX, 16);
    send_data_lite[0][7] = (pos_tmp >> 8);
    send_data_lite[1][0] = pos_tmp;

    pos_tmp = float_to_uint(motor.getRefPos(Left_Ankle_Pitch), P_MIN, P_MAX, 16);
    send_data_lite[1][1] = (pos_tmp >> 8);
    send_data_lite[1][2] = pos_tmp;

    pos_tmp = float_to_uint(motor.getRefPos(Right_Hip_Yaw), P_MIN, P_MAX, 16);
    send_data_lite[1][3] = (pos_tmp >> 8);
    send_data_lite[1][4] = pos_tmp;

    pos_tmp = float_to_uint(motor.getRefPos(Right_Hip_Roll), P_MIN, P_MAX, 16);
    send_data_lite[1][5] = (pos_tmp >> 8);
    send_data_lite[1][6] = pos_tmp;

    pos_tmp = float_to_uint(motor.getRefPos(Right_Hip_Pitch), P_MIN, P_MAX, 16);
    send_data_lite[1][7] = (pos_tmp >> 8);
    send_data_lite[2][0] = pos_tmp;

    pos_tmp = float_to_uint(motor.getRefPos(Right_Knee_Pitch), P_MIN, P_MAX, 16);
    send_data_lite[2][1] = (pos_tmp >> 8);
    send_data_lite[2][2] = pos_tmp;

    pos_tmp = float_to_uint(motor.getRefPos(Right_Ankle_Pitch), P_MIN, P_MAX, 16);
    send_data_lite[2][3] = (pos_tmp >> 8);
    send_data_lite[2][4] = pos_tmp;
}

void STM32::decodeData(Motor &motor, IMU &imu, Command &command)
{
    uint16_t pos_tmp, vel_tmp, tor_tmp;
    pos_tmp = receive_data[0][1] << 8 | receive_data[0][2];
    vel_tmp = receive_data[0][3] << 4 | receive_data[0][4] >> 4;
    tor_tmp = (receive_data[0][4] & 0xF) << 8 | receive_data[0][5];
    motor.setActPos(Left_Hip_Yaw, uint_to_float(pos_tmp, P_MIN, P_MAX, 16));
    motor.setActVel(Left_Hip_Yaw, uint_to_float(vel_tmp, V_MIN, V_MAX, 12));
    motor.setActTor(Left_Hip_Yaw, uint_to_float(tor_tmp, T_MIN, T_MAX, 12));

    pos_tmp = receive_data[0][6] << 8 | receive_data[0][7];
    vel_tmp = receive_data[1][0] << 4 | receive_data[1][1] >> 4;
    tor_tmp = (receive_data[1][1] & 0xF) << 8 | receive_data[1][2];
    motor.setActPos(Left_Hip_Roll, uint_to_float(pos_tmp, P_MIN, P_MAX, 16));
    motor.setActVel(Left_Hip_Roll, uint_to_float(vel_tmp, V_MIN, V_MAX, 12));
    motor.setActTor(Left_Hip_Roll, uint_to_float(tor_tmp, T_MIN, T_MAX, 12));

    pos_tmp = receive_data[1][3] << 8 | receive_data[1][4];
    vel_tmp = receive_data[1][5] << 4 | receive_data[1][6] >> 4;
    tor_tmp = (receive_data[1][6] & 0xF) << 8 | receive_data[1][7];
    motor.setActPos(Left_Hip_Pitch, uint_to_float(pos_tmp, P_MIN, P_MAX, 16));
    motor.setActVel(Left_Hip_Pitch, uint_to_float(vel_tmp, V_MIN, V_MAX, 12));
    motor.setActTor(Left_Hip_Pitch, uint_to_float(tor_tmp, T_MIN, T_MAX, 12));

    pos_tmp = receive_data[2][0] << 8 | receive_data[2][1];
    vel_tmp = receive_data[2][2] << 4 | receive_data[2][3] >> 4;
    tor_tmp = (receive_data[2][3] & 0xF) << 8 | receive_data[2][4];
    motor.setActPos(Left_Knee_Pitch, uint_to_float(pos_tmp, P_MIN, P_MAX, 16));
    motor.setActVel(Left_Knee_Pitch, uint_to_float(vel_tmp, V_MIN, V_MAX, 12));
    motor.setActTor(Left_Knee_Pitch, uint_to_float(tor_tmp, T_MIN, T_MAX, 12));

    pos_tmp = receive_data[3][1] << 8 | receive_data[3][2];
    vel_tmp = receive_data[2][5] << 4 | receive_data[2][6] >> 4;
    tor_tmp = (receive_data[2][6] & 0xF) << 8 | receive_data[2][7];
    motor.setActPos(Left_Ankle_Pitch, uint_to_float(pos_tmp, P_MIN, P_MAX, 16));
    motor.setActVel(Left_Ankle_Pitch, uint_to_float(vel_tmp, V_MIN, V_MAX, 12));
    motor.setActTor(Left_Ankle_Pitch, uint_to_float(tor_tmp, T_MIN, T_MAX, 12));

    pos_tmp = receive_data[3][3] << 8 | receive_data[3][4];
    vel_tmp = receive_data[3][5] << 4 | receive_data[3][6] >> 4;
    tor_tmp = (receive_data[3][6] & 0xF) << 8 | receive_data[3][7];
    motor.setActPos(Right_Hip_Yaw, uint_to_float(pos_tmp, P_MIN, P_MAX, 16));
    motor.setActVel(Right_Hip_Yaw, uint_to_float(vel_tmp, V_MIN, V_MAX, 12));
    motor.setActTor(Right_Hip_Yaw, uint_to_float(tor_tmp, T_MIN, T_MAX, 12));

    pos_tmp = receive_data[4][0] << 8 | receive_data[4][1];
    vel_tmp = receive_data[4][2] << 4 | receive_data[4][3] >> 4;
    tor_tmp = (receive_data[4][3] & 0xF) << 8 | receive_data[4][4];
    motor.setActPos(Right_Hip_Roll, uint_to_float(pos_tmp, P_MIN, P_MAX, 16));
    motor.setActVel(Right_Hip_Roll, uint_to_float(vel_tmp, V_MIN, V_MAX, 12));
    motor.setActTor(Right_Hip_Roll, uint_to_float(tor_tmp, T_MIN, T_MAX, 12));

    pos_tmp = receive_data[5][1] << 8 | receive_data[5][2];
    vel_tmp = receive_data[4][5] << 4 | receive_data[4][6] >> 4;
    tor_tmp = (receive_data[4][6] & 0xF) << 8 | receive_data[4][7];
    motor.setActPos(Right_Hip_Pitch, uint_to_float(pos_tmp, P_MIN, P_MAX, 16));
    motor.setActVel(Right_Hip_Pitch, uint_to_float(vel_tmp, V_MIN, V_MAX, 12));
    motor.setActTor(Right_Hip_Pitch, uint_to_float(tor_tmp, T_MIN, T_MAX, 12));

    pos_tmp = receive_data[5][3] << 8 | receive_data[5][4];
    vel_tmp = receive_data[5][5] << 4 | receive_data[5][6] >> 4;
    tor_tmp = (receive_data[5][6] & 0xF) << 8 | receive_data[5][7];
    motor.setActPos(Right_Knee_Pitch, uint_to_float(pos_tmp, P_MIN, P_MAX, 16));
    motor.setActVel(Right_Knee_Pitch, uint_to_float(vel_tmp, V_MIN, V_MAX, 12));
    motor.setActTor(Right_Knee_Pitch, uint_to_float(tor_tmp, T_MIN, T_MAX, 12));

    pos_tmp = receive_data[6][0] << 8 | receive_data[6][1];
    vel_tmp = receive_data[6][2] << 4 | receive_data[6][3] >> 4;
    tor_tmp = (receive_data[6][3] & 0xF) << 8 | receive_data[6][4];
    motor.setActPos(Right_Ankle_Pitch, uint_to_float(pos_tmp, P_MIN, P_MAX, 16));
    motor.setActVel(Right_Ankle_Pitch, uint_to_float(vel_tmp, V_MIN, V_MAX, 12));
    motor.setActTor(Right_Ankle_Pitch, uint_to_float(tor_tmp, T_MIN, T_MAX, 12));

    uint8_t vx_tmp, vy_tmp, wz_tmp;
    vx_tmp = receive_data[6][5];
    vy_tmp = receive_data[6][6];
    wz_tmp = receive_data[6][7];
    command.setLinearVel({uint_to_float(vx_tmp, LINEAR_VEL_MIN, LINEAR_VEL_MAX, 8), uint_to_float(vy_tmp, LINEAR_VEL_MIN, LINEAR_VEL_MAX, 8), 0});
    command.setAngularVel({0.0f, 0.0f, uint_to_float(wz_tmp, ANGULAR_VEL_MIN, ANGULAR_VEL_MAX, 8)});

    float acc_x_tmp, acc_y_tmp, acc_z_tmp, gyro_x_tmp, gyro_y_tmp, gyro_z_tmp;
    acc_x_tmp = uint_to_float(receive_data[7][0] << 8 | receive_data[7][1], A_MIN, A_MAX, 16);
    acc_y_tmp = uint_to_float(receive_data[7][2] << 8 | receive_data[7][3], A_MIN, A_MAX, 16);
    acc_z_tmp = uint_to_float(receive_data[7][4] << 8 | receive_data[7][5], A_MIN, A_MAX, 16);
    gyro_x_tmp = uint_to_float(receive_data[7][6] << 8 | receive_data[7][7], G_MIN, G_MAX, 16);
    gyro_y_tmp = uint_to_float(receive_data[8][0] << 8 | receive_data[8][1], G_MIN, G_MAX, 16);
    gyro_z_tmp = uint_to_float(receive_data[8][2] << 8 | receive_data[8][3], G_MIN, G_MAX, 16);

    imu.setAccel({acc_y_tmp, -acc_x_tmp, acc_z_tmp});   // Rotate the IMU data to match the robot frame
    imu.setGyro({gyro_y_tmp, -gyro_x_tmp, gyro_z_tmp}); // Rotate the IMU data to match the robot frame
}

void STM32::decodeDataLite(Motor &motor, IMU &imu, Command &command)
{
    // only receive position and velocity
    uint16_t pos_tmp, vel_tmp;
    float pos, vel;
    pos_tmp = receive_data_lite[0][0] << 8 | receive_data_lite[0][1];
    vel_tmp = receive_data_lite[0][2] << 4 | receive_data_lite[0][3] >> 4;
    pos = uint_to_float(pos_tmp, P_MIN, P_MAX, 16);
    vel = uint_to_float(vel_tmp, V_MIN, V_MAX, 12);
    motor.setActPos(Left_Hip_Yaw, pos);
    motor.setActVel(Left_Hip_Yaw, vel);

    pos_tmp = receive_data_lite[0][4] << 8 | receive_data_lite[0][5];
    vel_tmp = receive_data_lite[0][6] << 4 | receive_data_lite[0][7] >> 4;
    pos = uint_to_float(pos_tmp, P_MIN, P_MAX, 16);
    vel = uint_to_float(vel_tmp, V_MIN, V_MAX, 12);
    motor.setActPos(Left_Hip_Roll, pos);
    motor.setActVel(Left_Hip_Roll, vel);

    pos_tmp = receive_data_lite[1][0] << 8 | receive_data_lite[1][1];
    vel_tmp = receive_data_lite[1][2] << 4 | receive_data_lite[1][3] >> 4;
    pos = uint_to_float(pos_tmp, P_MIN, P_MAX, 16);
    vel = uint_to_float(vel_tmp, V_MIN, V_MAX, 12);
    motor.setActPos(Left_Hip_Pitch, pos);
    motor.setActVel(Left_Hip_Pitch, vel);

    pos_tmp = receive_data_lite[1][4] << 8 | receive_data_lite[1][5];
    vel_tmp = receive_data_lite[1][6] << 4 | receive_data_lite[1][7] >> 4;
    pos = uint_to_float(pos_tmp, P_MIN, P_MAX, 16);
    vel = uint_to_float(vel_tmp, V_MIN, V_MAX, 12);
    motor.setActPos(Left_Knee_Pitch, pos);
    motor.setActVel(Left_Knee_Pitch, vel);

    pos_tmp = receive_data_lite[2][0] << 8 | receive_data_lite[2][1];
    vel_tmp = receive_data_lite[2][2] << 4 | receive_data_lite[2][3] >> 4;
    pos = uint_to_float(pos_tmp, P_MIN, P_MAX, 16);
    vel = uint_to_float(vel_tmp, V_MIN, V_MAX, 12);
    motor.setActPos(Left_Ankle_Pitch, pos);
    motor.setActVel(Left_Ankle_Pitch, vel);

    pos_tmp = receive_data_lite[2][4] << 8 | receive_data_lite[2][5];
    vel_tmp = receive_data_lite[2][6] << 4 | receive_data_lite[2][7] >> 4;
    pos = uint_to_float(pos_tmp, P_MIN, P_MAX, 16);
    vel = uint_to_float(vel_tmp, V_MIN, V_MAX, 12);
    motor.setActPos(Right_Hip_Yaw, pos);
    motor.setActVel(Right_Hip_Yaw, vel);

    pos_tmp = receive_data_lite[3][0] << 8 | receive_data_lite[3][1];
    vel_tmp = receive_data_lite[3][2] << 4 | receive_data_lite[3][3] >> 4;
    pos = uint_to_float(pos_tmp, P_MIN, P_MAX, 16);
    vel = uint_to_float(vel_tmp, V_MIN, V_MAX, 12);
    motor.setActPos(Right_Hip_Roll, pos);
    motor.setActVel(Right_Hip_Roll, vel);

    pos_tmp = receive_data_lite[3][4] << 8 | receive_data_lite[3][5];
    vel_tmp = receive_data_lite[3][6] << 4 | receive_data_lite[3][7] >> 4;
    pos = uint_to_float(pos_tmp, P_MIN, P_MAX, 16);
    vel = uint_to_float(vel_tmp, V_MIN, V_MAX, 12);
    motor.setActPos(Right_Hip_Pitch, pos);
    motor.setActVel(Right_Hip_Pitch, vel);

    pos_tmp = receive_data_lite[4][0] << 8 | receive_data_lite[4][1];
    vel_tmp = receive_data_lite[4][2] << 4 | receive_data_lite[4][3] >> 4;
    pos = uint_to_float(pos_tmp, P_MIN, P_MAX, 16);
    vel = uint_to_float(vel_tmp, V_MIN, V_MAX, 12);
    motor.setActPos(Right_Knee_Pitch, pos);
    motor.setActVel(Right_Knee_Pitch, vel);

    pos_tmp = receive_data_lite[4][4] << 8 | receive_data_lite[4][5];
    vel_tmp = receive_data_lite[4][6] << 4 | receive_data_lite[4][7] >> 4;
    pos = uint_to_float(pos_tmp, P_MIN, P_MAX, 16);
    vel = uint_to_float(vel_tmp, V_MIN, V_MAX, 12);
    motor.setActPos(Right_Ankle_Pitch, pos);
    motor.setActVel(Right_Ankle_Pitch, vel);

    uint8_t vx_tmp, vy_tmp, wz_tmp;
    vx_tmp = receive_data_lite[5][0];
    vy_tmp = receive_data_lite[5][1];
    wz_tmp = receive_data_lite[5][2];
    float vx = uint_to_float(vx_tmp, LINEAR_VEL_MIN, LINEAR_VEL_MAX, 8);
    float vy = uint_to_float(vy_tmp, LINEAR_VEL_MIN, LINEAR_VEL_MAX, 8);
    float wz = uint_to_float(wz_tmp, ANGULAR_VEL_MIN, ANGULAR_VEL_MAX, 8);
    vx = fabs(vx) < 0.05f ? 0.0f : vx;
    vy = fabs(vy) < 0.05f ? 0.0f : vy;
    wz = fabs(wz) < 0.05f ? 0.0f : wz;
    command.setLinearVel({vx, vy, 0.0f});
    command.setAngularVel({0.0f, 0.0f, wz});

    float acc_x_tmp, acc_y_tmp, acc_z_tmp, gyro_x_tmp, gyro_y_tmp, gyro_z_tmp;
    acc_x_tmp = uint_to_float(receive_data_lite[5][3] << 8 | receive_data_lite[5][4], A_MIN, A_MAX, 16);
    acc_y_tmp = uint_to_float(receive_data_lite[5][5] << 8 | receive_data_lite[5][6], A_MIN, A_MAX, 16);
    acc_z_tmp = uint_to_float(receive_data_lite[5][7] << 8 | receive_data_lite[6][0], A_MIN, A_MAX, 16);
    gyro_x_tmp = uint_to_float(receive_data_lite[6][1] << 8 | receive_data_lite[6][2], G_MIN, G_MAX, 16);
    gyro_y_tmp = uint_to_float(receive_data_lite[6][3] << 8 | receive_data_lite[6][4], G_MIN, G_MAX, 16);
    gyro_z_tmp = uint_to_float(receive_data_lite[6][5] << 8 | receive_data_lite[6][6], G_MIN, G_MAX, 16);

    imu.setAccel({acc_y_tmp, -acc_x_tmp, acc_z_tmp});   // Rotate the IMU data to match the robot frame
    imu.setGyro({gyro_y_tmp, -gyro_x_tmp, gyro_z_tmp}); // Rotate the IMU data to match the robot frame
}

void STM32::sendData(CAN &can)
{
    can_frame frame = {};
    frame.can_id = CAN_SEND_START_ID;
    frame.can_dlc = PACKAGE_SIZE;
#ifdef USE_LITE_PACKAGE
    for (int i = 0; i < SEND_PACKAGE_NUM_LITE; i++)
    {
        frame.can_id = CAN_SEND_START_ID + i;
        memcpy(frame.data, send_data_lite[i], PACKAGE_SIZE);
        can.send(frame);
    }
#else
    for (int i = 0; i < SEND_PACKAGE_NUM; i++)
    {
        frame.can_id = CAN_SEND_START_ID + i;
        memcpy(frame.data, send_data[i], PACKAGE_SIZE);
        can.send(frame);
    }
#endif
}

void STM32::receiveData(CAN &can)
{
    can_frame frame = {};
    frame.can_id = CAN_RECEIVE_START_ID;
    frame.can_dlc = PACKAGE_SIZE;
    can.receive(frame);
#ifdef USE_LITE_PACKAGE
    switch (frame.can_id)
    {
    case CAN_RECEIVE_START_ID:
        memcpy(receive_data_lite[0], frame.data, PACKAGE_SIZE);
        break;
    case CAN_RECEIVE_START_ID + 1:
        memcpy(receive_data_lite[1], frame.data, PACKAGE_SIZE);
        break;
    case CAN_RECEIVE_START_ID + 2:
        memcpy(receive_data_lite[2], frame.data, PACKAGE_SIZE);
        break;
    case CAN_RECEIVE_START_ID + 3:
        memcpy(receive_data_lite[3], frame.data, PACKAGE_SIZE);
        break;
    case CAN_RECEIVE_START_ID + 4:
        memcpy(receive_data_lite[4], frame.data, PACKAGE_SIZE);
        break;
    case CAN_RECEIVE_START_ID + 5:
        memcpy(receive_data_lite[5], frame.data, PACKAGE_SIZE);
        break;
    case CAN_RECEIVE_START_ID + 6:
        memcpy(receive_data_lite[6], frame.data, PACKAGE_SIZE);
        break;
    }
#else
    can.receive(frame);
    switch (frame.can_id)
    {
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
    case CAN_RECEIVE_START_ID + 7:
        memcpy(receive_data[7], frame.data, PACKAGE_SIZE);
        break;
    case CAN_RECEIVE_START_ID + 8:
        memcpy(receive_data[8], frame.data, PACKAGE_SIZE);
        break;
    }
#endif
}
/*
 * mpu6050.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#endif /* INC_UTILS_H_ */



// GYRO_ACC structure
typedef struct
{

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax_cal;
    double Ay_cal;
    double Az_cal;

    float Gyro_X_RAW;
    float Gyro_Y_RAW;
    float Gyro_Z_RAW;

    double Gx_cal;
    double Gy_cal;
    double Gz_cal;

    double KalmanAngleX;
    double KalmanAngleY;
} GYRO_ACC_t;


typedef struct
{

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax_cal;
    double Ay_cal;
    double Az_cal;

    float Gyro_X_RAW;
    float Gyro_Y_RAW;
    float Gyro_Z_RAW;

    double Gx_cal;
    double Gy_cal;
    double Gz_cal;

    double KalmanAngleX;
    double KalmanAngleY;
} GYRO_ACC_t;



// Kalman structure
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;


void GYRO_Calibrate(GYRO_ACC_t);

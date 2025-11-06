/*
 * ICM.h
 *
 *  Created on: 2025年10月30日
 *      Author: ji_rencai
 */

#ifndef CODE_ICM_H_
#define CODE_ICM_H_

#include "zf_common_headfile.h"


#define Ang2Rad 0.01745329252f //角度制转弧度制
#define Rad2Ang 57.295779513f  //弧度制转角度制

// 状态维度：4个四元数 + 3个陀螺偏置
#define STATE_DIM 7
#define MEAS_DIM  3

// EKF 状态结构体
typedef struct {
    float x[STATE_DIM];                // 状态向量：[q0,q1,q2,q3, bgx, bgy, bgz]
    float P[STATE_DIM][STATE_DIM];     // 协方差矩阵
    float Q[STATE_DIM][STATE_DIM];     // 过程噪声协方差
    float R[MEAS_DIM][MEAS_DIM];       // 测量噪声协方差
} EKF_State;
extern EKF_State ekf;
extern int ekf_initialized;

typedef struct
{
    float a0, a1, a2, a3, a4;
    float x1, x2, y1, y2;
}biquad_state;

typedef enum
{
    BIQUAD_LOWPASS,//低通滤波器（允许频率低于某一特定值（即截止频率）的信号通过，而频率高于该值的信号会被衰减）
    BIQUAD_HIGHPASS,//高通滤波器
    BIQUAD_BANDPASS_PEAK,//带通滤波器（允许位于两个特定频率之间的信号通过，限制范围之外所有频段），峰值类型配置
    BIQUAD_BANDSTOP_NOTCH,//带通滤波器（创建某个特定的频率范围，阻止该范围内的信号通过）
}biquad_type;

typedef struct{
    float Kp;
    float Ki;
    float Kd;
}PID_t;


extern volatile float twoKp;
extern volatile float twoKi;
extern volatile float q0, q1, q2, q3;

extern float Yaw_angle_counter;
extern float Yaw_angle;
extern float Yaw_gyro;

extern float Pitch_a;//实时姿态角（我修改的）
extern float Roll_a;
extern float Yaw_a;
extern float Pitch_a_Pi, Roll_a_Pi, Yaw_a_Pi;
extern float Pitch_b, Roll_b, Yaw_b;
extern float Pitch_c, Roll_c, Yaw_c;
extern float Pitch_b_use, Roll_b_use, Yaw_b_use;
extern float aim_roll_a,aim_pitch_a,aim_yaw_a,d_aim_yaw_a;
extern float Pitch, Roll, Yaw;
extern float Yaw_angular_acc,Pitch_angular_acc;
extern float Pitch_g_F, Roll_g_F, Yaw_g_F;
extern float Pitch_acc_F, Roll_acc_F, Yaw_acc_F;

void IMU_calibration(void);
float get_Yaw_data();
float get_yaw();
float get_yaw_g();
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void updateMadgwickEulerIMU(float gx, float gy, float gz,
                            float ax, float ay, float az, float dt);
void get_ICM_data(void);
void Filter_Init(void);
float biquad(biquad_state *state, float data);
void biquad_filter_init(biquad_state *state, biquad_type type, int fs, float fc, float q_value);


#endif /* CODE_ICM_H_ */

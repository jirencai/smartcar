/*
 * IPM.c
 *
 *  Created on: 2025年11月4日
 *      Author: ji_rencai
 */
#include "IPM.h"

#define DeltaX    /*过摄像头且平行于支撑杆的直线与旋转中心的距离（向前平移为正，单位：cm）*/   deltax
#define Height    /*摄像头高度（单位：cm）*/                                      height   //wheel_y-4.0 44
#define CAM_Angle /*镜头光轴与竖杆夹角的角度（角度制）*/                              cam_angle //60  73
#define Alpha     /*摄像头X方向最大张角的半角(角度制)*/                              alpha     //先调这个 43 46 51.8

double K_x_1, K_x_2, K_x_3, K_x_4, C_x_1, C_x_2;
double K_y_1, K_y_2, K_y_3, K_y_4, C_y_1, C_y_2;
double K_r, K_r_1, K_r_2, K_r_3, K_r_4, K_r_5, K_r_6, K_r_7, K_r_8;
double X_0, Y_0, Z_0;
double Beta;
double TAN_A;
double W = (double)IMAGEW, H = (double)IMAGEH;
double Theta,Phi;

Quaternion q_imu;

void IPM_init(float ay, float ax){
    Beta = Ang2Rad*CAM_Angle;
    TAN_A = tan(Ang2Rad*Alpha);
    double h=(double)Height;
//    if(motorstart ==2 && bridge_type == BRIDGE_NONE && (sprintboard_type == SPRINTBOARD_NONE || Jump_enable == 0)){
//    h += (Left_compensate+Right_compensate)/2;
//    }
    double Delta = (double)DeltaX;
    Theta = Ang2Rad*ay; //Pitch
    Phi   = Ang2Rad*ax; //Roll
    double Theta_Beta = Theta-Beta;
    K_x_1 = TAN_A*(cos(Theta)*cos(Beta)+sin(Theta)*cos(Phi)*sin(Beta))/W;
    K_x_2 = TAN_A*(sin(Theta)*sin(Phi))/W;
    K_x_3 = TAN_A*(-sin(Theta)*cos(Beta)+sin(Beta)*cos(Theta)*cos(Phi))/W;
    K_x_4 = TAN_A*(cos(Theta)*sin(Phi))/W;
    C_x_1 = sin(Beta)*cos(Theta)-sin(Theta)*cos(Phi)*cos(Beta);
    C_x_2 = -sin(Theta)*sin(Beta)-cos(Theta)*cos(Phi)*cos(Beta);

    K_y_1 = -TAN_A*(sin(Phi)*sin(Beta))/W;
    K_y_2 = TAN_A*cos(Phi)/W;
    K_y_3 = K_x_3;
    K_y_4 = K_x_4;
    C_y_1 = sin(Phi)*cos(Beta);
    C_y_2 = C_x_2;
//    K_x_1 = TAN_A*cos(Theta_Beta)/W;
//    K_x_2 = TAN_A*sin(Theta_Beta)*sin(Phi)/W;
//    K_x_3 = -TAN_A*sin(Theta_Beta)/W;
//    K_x_4 = TAN_A*cos(Theta_Beta)*sin(Phi)/W;
//    C_x_1 = -sin(Theta_Beta)*cos(Phi);
//    C_x_2 = -cos(Theta_Beta)*cos(Phi);
//
//    K_y_1 = TAN_A*cos(Phi)/W;
//    K_y_2 = K_x_3;
//    K_y_3 = K_x_4;
//    C_y_1 = sin(Phi);
//    C_y_2 = C_x_2;

    X_0 = Delta*cos(Theta)+h*sin(Theta)*cos(Phi);
    Y_0 = -h*sin(Phi);
    Z_0 = -Delta*sin(Theta)+h*cos(Theta)*cos(Phi);

    K_r = W/TAN_A;
    K_r_1 = -C_y_1;
    K_r_2 = cos(Phi);
    K_r_3 = -C_x_1;
    K_r_4 = sin(Theta_Beta)*sin(Phi);
    K_r_5 = -C_x_2;
    K_r_6 = cos(Theta_Beta);
    K_r_7 = cos(Theta_Beta)*sin(Phi);
    K_r_8 = -sin(Theta_Beta);
}
/*坐标的变换*/
void IPM(sint16 x, sint16 y, float *result){
    double a = (double)(H-2*y), b = (double)(W-2*x);
    float x0 = (float)(Y_0-Z_0*(a*K_y_1+b*K_y_2+C_y_1)/(a*K_y_3+b*K_y_4+C_y_2));
    float y0 = (float)(X_0-Z_0*(a*K_x_1+b*K_x_2+C_x_1)/(a*K_x_3+b*K_x_4+C_x_2));
    result[0] = 94.0f - (x0 ); //x' - y0*tan(Phi)
//    result[0] = 94.0f - (float)(Y_0-Z_0*(a*K_y_1+b*K_y_2+C_y_1)/(a*K_y_3+b*K_y_4+C_y_2)); //x'
    result[1] = 120.0f - y0; //y'
}
/*若图像反变换后超出图像可展示的地区，则返回false*/
bool Re_IPM(float x, float y, int *result){
    double X_1=(X_0-120.0+y), Y_1=(Y_0-94.0+x);
    result[0] = (int)round(0.5*(W+K_r*(X_1*K_r_1+Y_1*K_r_2)/(X_1*K_r_3+Y_1*K_r_4+Z_0*K_r_5))); //x'
    result[1] = (int)round(0.5*(H+K_r*(X_1*K_r_6+Y_1*K_r_7+Z_0*K_r_8)/(X_1*K_r_3+Y_1*K_r_4+Z_0*K_r_5))); //y'
    if(result[0]<0 || result[0]>=188 || result[1]<0 || result[1]>=120) return false;
    return true;
}




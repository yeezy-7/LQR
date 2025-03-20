#ifndef __KF_H
#define __KF_H
#include "sys.h"


float KF_Y(float acce_X, float acce_Z, float gyro_Y);
float KF_X(float Acce_Y, float Acce_Z, float Gyro_X);
void mul(int A_row, int A_col, int B_row, int B_col, float A[][A_col], float B[][B_col], float C[][B_col]);
















#endif






/**
 * @file AHRS.h
 * @author Yang-RUi Li (you@domain.com)
 * @brief AHRS library for real-time attitude estimation.
 * @version 0.1
 * @date 2022-05-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include <math.h>

#define invSqrt(x, y, z, w) 1.0 / sqrt(x *x + y * y + z * z + w * w)

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

#ifdef __cplusplus
extern "C"
{
#endif

    void AHRS_Init(double *fc, double T);
    void AHRS_Estimate_IC(double *acc, double *mag);
    void AHRS_Update(double *gyro, double *acc, double *mag);
    void AHRS_Get_Quat(double *quaternion);
    void AHRS_Get_Euler(double *euler_angle);
//    static void SAAM(double *acc, double *mag);

#ifdef __cplusplus
}
#endif

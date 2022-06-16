/**
 * @file Attitude_Kinematics.h
 * @author Yang-Rui Li (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-06-02
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include <cmath>

#define PI 3.1415926535897932384626433832795
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105


void eul2dcm(double eul[3], double* dcm);
void qua2eul(double* qua, double* eul);
void qua2dcm(double* qua, double* dcm);
void rotateVector(double* dcm, double* vec, double* vec_r);
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

void eul2dcm(double eul[3], double *dcm);
void qua2dcm(double *qua, double *dcm);
void rotateVector(double *dcm, double *vec, double *vec_r);
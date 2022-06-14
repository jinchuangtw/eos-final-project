#include "Attitude_Kinematics.h"

void eul2dcm(double *eul, double *dcm)
{
    static double sphi, cphi, stheta, ctheta, spsi, cpsi;
    sphi = sin(eul[0]);
    cphi = cos(eul[0]);
    stheta = sin(eul[1]);
    ctheta = cos(eul[1]);
    spsi = sin(eul[2]);
    cpsi = cos(eul[2]);

    dcm[0] = cpsi * ctheta;
    dcm[1] = cpsi * sphi * stheta - cphi * spsi;
    dcm[2] = sphi * spsi + cphi * cpsi * stheta;
    dcm[3] = ctheta * spsi;
    dcm[4] = cphi * cpsi + sphi * spsi * stheta;
    dcm[5] = cphi * spsi * stheta - cpsi * sphi;
    dcm[6] = -stheta;
    dcm[7] = ctheta * sphi;
    dcm[8] = cphi * ctheta;
}

void qua2dcm(double *qua, double *dcm)
{
    static double q0, q1, q2, q3;
    q0 = qua[0];
    q1 = qua[1]; 
    q2 = qua[2];
    q3 = qua[3];

    dcm[0] = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    dcm[1] = 2.0 * q1 * q2 - 2.0 * q0 * q3;
    dcm[2] = 2.0 * q1 * q3 + 2.0 * q0 * q2;
    dcm[3] = 2.0 * q1 * q2 + 2.0 * q0 * q3;
    dcm[4] = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
    dcm[5] = 2.0 * q2 * q3 - 2.0 * q0 * q1;
    dcm[6] = 2.0 * q1 * q3 - 2.0 * q0 * q2;
    dcm[7] = 2.0 * q2 * q3 + 2.0 * q0 * q1;
    dcm[8] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
}

void rotateVector(double *dcm, double *vec, double *vec_r)
{
    for (int i = 0; i < 3; i++)
    {
        vec_r[i] = dcm[3 * i + 0] * vec[0] + dcm[3 * i + 1] * vec[1] + dcm[3 * i + 2] * vec[2];
    }
}
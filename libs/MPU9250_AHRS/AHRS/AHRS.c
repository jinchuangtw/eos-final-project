#include "AHRS.h"

static double q0, q1, q2, q3;
static double q0_ref, q1_ref, q2_ref, q3_ref;
static double q0_dot, q1_dot, q2_dot, q3_dot;
static double reciprocal_norm;
static double dt, wc[4];
static void SAAM(double *acc, double *mag);

static void SAAM(double *acc, double *mag)
{
    static double alpha, mN, mD;
    static double ax, ay, az, mx, my, mz;

    /* Accelerometer Measurement */
    ax = acc[0];
    ay = acc[1];
    az = acc[2];

    /* Magnetometer Measurement */
    mx = mag[0];
    my = mag[1];
    mz = mag[2];

    /* Normalize Gravity Vector */
    reciprocal_norm = invSqrt(ax, ay, az, 0.0);
    ax *= reciprocal_norm;
    ay *= reciprocal_norm;
    az *= reciprocal_norm;

    /* Normalize Magnetic Vector */
    reciprocal_norm = invSqrt(mx, my, mz, 0.0);
    mx *= reciprocal_norm;
    my *= reciprocal_norm;
    mz *= reciprocal_norm;

    /* Estimate the reference quaternion using SAAM */
    alpha = ax * mx + ay * my + az * mz;
    if (alpha * alpha < 1.0)
    {
        mN = sqrt(1.0 - alpha * alpha);
        mD = alpha;
    }

    q0_ref = -ay * (mN + mx) + ax * my;
    q1_ref = (az - 1.0) * (mN + mx) + ax * (mD - mz);
    q2_ref = (az - 1.0) * my + ay * (mD - mz);
    q3_ref = az * mD - ax * mN - mz;

    /* Quaternion Double Cover */
    if ((q0_ref * q0 + q1_ref * q1 + q2_ref * q2 + q3_ref * q3) < 0.0)
    {
        q0_ref = -q0_ref;
        q1_ref = -q1_ref;
        q2_ref = -q2_ref;
        q3_ref = -q3_ref;
    }

    /* Normalize Reference Quaternion */
    reciprocal_norm = invSqrt(q0_ref, q1_ref, q2_ref, q3_ref);
    q0_ref *= reciprocal_norm;
    q1_ref *= reciprocal_norm;
    q2_ref *= reciprocal_norm;
    q3_ref *= reciprocal_norm;
}

void AHRS_Init(double *fc, double T)
{
    dt = T;
    for (int i = 0; i < 4; i++)
    {
        wc[i] = 1.0 / (2.0 * PI * fc[i] * T + 1.0);
    }
    q0 = 1.0;
}

void AHRS_Estimate_IC(double *acc, double *mag)
{
    /* Estimate the quaternion from the reference fields */
    SAAM(acc, mag);
    q0 = q0_ref;
    q1 = q1_ref;
    q2 = q2_ref;
    q3 = q3_ref;
}

void AHRS_Update(double *gyro, double *acc, double *mag)
{
    double wx, wy, wz;

    /* Gyroscope Measurement */
    wx = gyro[0];
    wy = gyro[1];
    wz = gyro[2];

    /* SAAM */
    SAAM(acc, mag);

    /* First-order Complementary Filtering */
    q0 = wc[0] * (q0 + dt * q0_dot) + (1.0 - wc[0]) * q0_ref;
    q1 = wc[1] * (q1 + dt * q1_dot) + (1.0 - wc[1]) * q1_ref;
    q2 = wc[2] * (q2 + dt * q2_dot) + (1.0 - wc[2]) * q2_ref;
    q3 = wc[3] * (q3 + dt * q3_dot) + (1.0 - wc[3]) * q3_ref;
    reciprocal_norm = invSqrt(q0, q1, q2, q3);
    q0 *= reciprocal_norm;
    q1 *= reciprocal_norm;
    q2 *= reciprocal_norm;
    q3 *= reciprocal_norm;

    /* Estimate the rate of quaternion qDot[k+1|k]_ */
    q0_dot = 0.5 * (-q1 * wx - q2 * wy - q3 * wz);
    q1_dot = 0.5 * (q0 * wx - q3 * wy + q2 * wz);
    q2_dot = 0.5 * (q3 * wx + q0 * wy - q1 * wz);
    q3_dot = 0.5 * (q1 * wy - q2 * wx + q0 * wz);
}

void AHRS_Get_Quat(double *quaternion)
{
    quaternion[0] = q0;
    quaternion[1] = q0;
    quaternion[2] = q0;
    quaternion[3] = q0;
}

void AHRS_Get_Euler(double *euler_angle)
{
    euler_angle[0] = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
    euler_angle[1] = asin(2.0 * (q0 * q2 - q3 * q1));
    euler_angle[2] = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
}

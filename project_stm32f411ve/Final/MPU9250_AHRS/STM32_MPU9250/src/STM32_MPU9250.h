/**
 * @file STM_MPU9250.h
 * @author Yang-Rui Li (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-04-19
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include "stm32f4xx_hal.h"
#include "stdint.h"

/* Register Map */
// #define AD0_LOW          0x68 // Address of MPU9250
// #define AD0_HIGH         0x69
// #define PWR_MGMT_1       0x6B // 107 Power ()
// #define GYRO_CONFIG      0x1B
// #define ACCEL_CONFIG     0x1C
// #define ACCEL_X_OUT_H    0x3B // Data Register
// #define AK8963_I2C_ADDR 0x0C
// #define AK8963_HXL 0x03
// #define AK8963_CNTL1 0x0A
// #define AK8963_PWR_DOWN 0x00
// #define AK8963_CNT_MEAS1 0x12
// #define AK8963_CNT_MEAS2 0x16
// #define AK8963_FUSE_ROM 0x0F
// #define AK8963_CNTL2 0x0B
// #define AK8963_RESET 0x01
// #define AK8963_ASA 0x10

/* Register Map of Accelerometer and Gyroscope */
#define MPU9250_ADDRESS 0x68      //
#define MPU9250_WHO_AM_I 0x75     // WHO_AM_I Register
#define MPU9250_WHO_AM_I_ANS 0x71 // 9250
#define MPU9250_ACCEL_X_H 0x3B
#define MPU9250_PWR_MGMNT_1 0x6B
#define MPU9250_GYRO_CONFIG 0x1B // (27)
#define MPU9250_ACCEL_CONFIG_1 0x1C
 #define MPU9250_ACCEL_CONFIG_2 0x1D 29

#define MPU9250_INT_BYPASS_CONFIG 55 // 0x37
#define MPU9250_USER_CTRL 106        // 0x6A
// #define mpu_6050_sig_path_rst        104

/* Register Map of Magnetometer */
#define MPU9250_MAG_CONTROL_CONFIG 0x0A
#define MPU9250_MAG_ASAX_CONFIG 0x10
#define MPU9250_MAG_STATUS_1_CONFIG 0x02
#define MPU9250_AK8963_DEVICE_ID 0x0C // 0x48
#define MPU9250_AK8963_WHO_AM_I 0x00
#define MPU9250_AK8963_WHO_AM_I_ANS 0x48
#define MPU9250_DATA_READY_MASK 0x01
#define MPU9250_DATA_READY 0x01
#define MPU9250_MAGIC_OVERFLOW_MASK 0x8
#define MPU9250_MAG_HXL_AD 0x03

#define INIT_BYTE_107 0b00000001 // 0b00000001
// #define INIT_BYTE_28 0b00001000
// #define init_byte_106 0b00000001
// #define init_byte_104 0b00000111
// #define INIT_BYTE_27 0b00001000
// #define INIT_BYTE_29 0b00000101
// #define INIT_BYTE_26 0b00000101
#define INIT_BYTE_106 0b00000000
#define INIT_BYTE_55 0b00000010
#define INIT_BYTE_MAG_10 0b00011111        // fuse mode 0x16
#define RESET_BYTE_MAG_10 0b00000000       // reset
#define RESET_BYTE_MAG_MODE2_10 0b00010110 // continuous mode 2 at 100Hz and 16 bit output

#define SMPDIV 0x19 // Sample Rate Devider

/* Some Useful Macros ... */
#define I2C_TIME_OUT_MS 1000 // Delay Time of I2C Communication
#define PI 3.1415926535897932384626433832795
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define G 9.807 // Gravity Constant

/* Define Gyroscope Full Scale Reading Range (xx bits)*/
enum GYRO_RANGE
{
    GYRO_RANGE_250DPS,
    GYRO_RANGE_500DPS,
    GYRO_RANGE_1000DPS,
    GYRO_RANGE_2000DPS
};

/* Define Accelerometer Full Scale Reading Range */
enum ACC_RANGE
{
    ACC_RANGE_2G,
    ACC_RANGE_4G,
    ACC_RANGE_8G,
    ACC_RANGE_16G
};

typedef struct xIMU_Hardware_Config
{

} IMU_HW_CONFIG_t;

typedef struct xIMU_Parameters
{

} IMU_PARA_t;

typedef struct xIMU_Raw_Data
{

} IMU_RAW_DATA_t;

/* Data Structure of MPU-9250's IMU 9-Axis Data */
typedef struct xIMU_Infomation
{
    double acc[3];      // m/s^2
    double gyro[3];     // rad/sec
    double mag[3];      // mu T
    double temperature; // do C (?)
    IMU_HW_CONFIG_t HW_Config;
    IMU_PARA_t Para;
} IMU_t;

/* Variable Declaration */
uint8_t ADDR, ADDR_AK8963;        // Address
I2C_HandleTypeDef *pI2Cx_Handler; // Pointer of I2C Handler
double acc_res;                   // Resolution of Accelerometer
double gyro_res;                  // Resolution of Gyroscope
double temperature_scale;
double temperature_offset;
double mag_sensitivity[3];

/* Function Declaration */
int MPU_I2C_Init(I2C_HandleTypeDef *_pI2Cx_Handler, int Acc_Range, int Gyro_Range);
int MPU_I2C_Read(IMU_t *pimu);

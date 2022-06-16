#include "STM32_MPU9250.h"

/* I2C Initialization of MPU9250 */
int MPU_I2C_Init(I2C_HandleTypeDef *_pI2Cx_Handler, int Acc_Range, int Gyro_Range)
{
    /* Local Variables */
    uint8_t check; // Buffer
    uint8_t select; // Buffer

    /* Variable Initialization */
    ADDR = MPU9250_ADDRESS << 1;
    pI2Cx_Handler = _pI2Cx_Handler;

    temperature_scale = 333.87;
    temperature_offset = 21.0;

    /********* Accelerometer and Gyroscope Initialization *********/
    /* Confirm device ID of MPU-9250 (Accelerometer and Gyroscope) */
    HAL_I2C_Mem_Read(pI2Cx_Handler, ADDR, MPU9250_WHO_AM_I, 1, &check, 1, I2C_TIME_OUT_MS);
    HAL_Delay(100);

    if (check != MPU9250_WHO_AM_I_ANS)
    {
        return -1;
    }

    /* Tie the clock source to X axis of the gyro for a better accuracy */
    select = INIT_BYTE_107;
    HAL_I2C_Mem_Write(pI2Cx_Handler, ADDR, MPU9250_PWR_MGMNT_1, 1, &select, 1, I2C_TIME_OUT_MS);

    /* Set Reading Range of Accelerometer (TODO:) */
    switch (Acc_Range)
    {
    case ACC_RANGE_2G:
        acc_res = 16384.0;
        select = 0x00; // ACCEL_FS_SEL_2G
        break;
    case ACC_RANGE_4G:
        acc_res = 8192.0;
        select = 0x08; // ACCEL_FS_SEL_4G
        break;
    case ACC_RANGE_8G:
        acc_res = 4096.0;
        select = 0x10; // ACCEL_FS_SEL_8G
        break;
    case ACC_RANGE_16G:
        acc_res = 2048.0;
        select = 0x18; // ACCEL_FS_SEL_16G
        break;
    default:
        acc_res = 8192.0;
        select = 0x08;
        break;
    }
    acc_res = acc_res / G; // Unit: m/s^2
    HAL_I2C_Mem_Write(pI2Cx_Handler, ADDR, MPU9250_ACCEL_CONFIG_1, 1, &select, 1, I2C_TIME_OUT_MS);
    HAL_Delay(100);

    /* Set Reading Range of Gyroscope (TODO:) */
    switch (Gyro_Range)
    {
    case GYRO_RANGE_250DPS:
        gyro_res = 131.0;
        select = 0x00; // GYRO_FS_SEL_250DPS
        break;
    case GYRO_RANGE_500DPS:
        gyro_res = 65.5;
        select = 0x08; // GYRO_FS_SEL_500DPS
        break;
    case GYRO_RANGE_1000DPS:
        gyro_res = 32.8;
        select = 0x10; // GYRO_FS_SEL_1000DPS
        break;
    case GYRO_RANGE_2000DPS:
        gyro_res = 16.4;
        select = 0x18;// GYRO_FS_SEL_2000DPS
        break;
    default:
        gyro_res = 65.5;
        select = 0x08;
        break;
    }
    gyro_res = gyro_res / DEG_TO_RAD;
    HAL_I2C_Mem_Write(pI2Cx_Handler, ADDR, MPU9250_GYRO_CONFIG, 1, &select, 1, I2C_TIME_OUT_MS);
    HAL_Delay(100);

    /********* Magnetometer Initialization *********/
    ADDR_AK8963 = MPU9250_AK8963_DEVICE_ID << 1;

    /* Disable I2C master interface. Precondition to enable bypass multiplexer of the I2C master interface */
    select = INIT_BYTE_106;
    HAL_I2C_Mem_Write(pI2Cx_Handler, ADDR, MPU9250_USER_CTRL, 1, &select, 1, I2C_TIME_OUT_MS);

    /* Enable I2C master interface bypass multiplexer */
    select = INIT_BYTE_55;
    HAL_I2C_Mem_Write(pI2Cx_Handler, ADDR, MPU9250_INT_BYPASS_CONFIG, 1, &select, 1, I2C_TIME_OUT_MS);

    /* Setup the magnetometer: Fuse ROM access mode and 16 bit output */
    select = INIT_BYTE_MAG_10;
    HAL_I2C_Mem_Write(pI2Cx_Handler, ADDR_AK8963, MPU9250_INT_BYPASS_CONFIG, 1, &select, 1, I2C_TIME_OUT_MS);

    /* Read the sensitivity adjustment values */
    uint8_t buf[3];
    HAL_I2C_Mem_Read(pI2Cx_Handler, ADDR_AK8963, MPU9250_MAG_ASAX_CONFIG, 1, buf, 3, I2C_TIME_OUT_MS);

    mag_sensitivity[0] = (((double) (buf[0]) - 128.0) / 256.0 + 1.0) * 4912.0 / 32760.0;
    mag_sensitivity[1] = (((double) (buf[1]) - 128.0) / 256.0 + 1.0) * 4912.0 / 32760.0;
    mag_sensitivity[2] = (((double) (buf[2]) - 128.0) / 256.0 + 1.0) * 4912.0 / 32760.0;

    /* Reset the magnetometer to power down mode */
    select = RESET_BYTE_MAG_10;
    HAL_I2C_Mem_Write(pI2Cx_Handler, ADDR_AK8963, MPU9250_INT_BYPASS_CONFIG, 1, &select, 1, I2C_TIME_OUT_MS);

    /* Enable chip to continuous mode 2 (100Hz) and 16-bit output */
    select = RESET_BYTE_MAG_MODE2_10;
    HAL_I2C_Mem_Write(pI2Cx_Handler, ADDR_AK8963, MPU9250_MAG_CONTROL_CONFIG, 1, &select, 1, I2C_TIME_OUT_MS);

    /* Get AK8963 WHO_AM_I Register. Default value: 0x48 */
    HAL_I2C_Mem_Read(pI2Cx_Handler, ADDR_AK8963, MPU9250_AK8963_WHO_AM_I, 1, &check, 1, I2C_TIME_OUT_MS);
    HAL_Delay(100);

    /* Confirm device ID of Magnetometer (AK8963) */
    if (check != MPU9250_AK8963_WHO_AM_I_ANS)
    {
        return -2;
    }

    /* Set sample rate divider (SRD) */
    select = 0x00;
    HAL_I2C_Mem_Write(pI2Cx_Handler, ADDR, SMPDIV, 1, &select, 1, I2C_TIME_OUT_MS);
    return 1;
}

/* Read MPU9250 Data */
int MPU_I2C_Read(IMU_t *pimu)
{
    uint8_t buf[14];
    static int16_t acc_counts[3], gyro_counts[3], mag_counts[3], temperature_counts;

    if (HAL_I2C_Mem_Read(pI2Cx_Handler, ADDR, MPU9250_ACCEL_X_H, 1, buf, 14, I2C_TIME_OUT_MS) != HAL_OK)
    {
        return -1;
    }

    // Bit shift the data
    acc_counts[0] = (int16_t) ((uint8_t) buf[0] << 8) | buf[1];
    acc_counts[1] = (int16_t) ((uint8_t) buf[2] << 8) | buf[3];
    acc_counts[2] = (int16_t) ((uint8_t) buf[4] << 8) | buf[5];
    temperature_counts = (int16_t) ((uint8_t) buf[6] << 8) | buf[7];
    gyro_counts[0] = (int16_t) ((uint8_t) buf[8] << 8) | buf[9];
    gyro_counts[1] = (int16_t) ((uint8_t) buf[10] << 8) | buf[11];
    gyro_counts[2] = (int16_t) ((uint8_t) buf[12] << 8) | buf[13];

    /* Read DRDY bit from Status 1 register to check if the data in ready "1" ready "0" not ready  */
    HAL_I2C_Mem_Read(pI2Cx_Handler, ADDR_AK8963, MPU9250_MAG_STATUS_1_CONFIG, 1, buf, 1, I2C_TIME_OUT_MS);

    if ((buf[0] & MPU9250_DATA_READY_MASK) == MPU9250_DATA_READY)
    {
        HAL_I2C_Mem_Read(pI2Cx_Handler, ADDR_AK8963, MPU9250_MAG_HXL_AD, 1, buf, 7, I2C_TIME_OUT_MS);

        /* check if magnetic sensor is overflow: If yes discard the reading */
        if (!(buf[6] & MPU9250_MAGIC_OVERFLOW_MASK))
        {
//            mag_counts[0] = (int16_t) (buf[0] | (uint8_t) (buf[1] << 8));
//            mag_counts[1] = (int16_t) (buf[2] | (uint8_t) (buf[3] << 8));
//            mag_counts[2] = (int16_t) (buf[4] | (uint8_t) (buf[5] << 8));
//            mag_counts[0] = (buf[1] << 8) | buf[0];
//            mag_counts[1] = (buf[3] << 8) | buf[2];
//            mag_counts[2] = (buf[5] << 8) | buf[4];
            mag_counts[1] = (((int16_t)buf[1]) << 8) | buf[0];
            mag_counts[0] = (((int16_t)buf[3]) << 8) | buf[2];
            mag_counts[2] = -((((int16_t)buf[5]) << 8) | buf[4]);
        }
    }

    for (int i = 0; i < 3; i++)
    {
        pimu->acc[i] = (double) (acc_counts[i]) / acc_res; // Unit: m/s^2
        pimu->gyro[i] = (double) (gyro_counts[i]) / gyro_res; // Unit: rad/sec
        pimu->mag[i] = (double) (mag_counts[i]) * mag_sensitivity[i]; // Unit: mu T
        pimu->temperature =
                ((double) (temperature_counts) - temperature_offset) / temperature_scale
                        + temperature_offset; // Unit: do C (?)
    }
    return 1;
}

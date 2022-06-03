# STM32_MPU9250

## How to use

* Initialization

    Prototype: `int MPU_I2C_Init(I2C_HandleTypeDef *_pI2Cx_Handler, int Acc_Range, int Gyro_Range);`
    
```c
I2C_HandleTypeDef hi2c3; // Declare I2C Handler 

if (MPU_I2C_Init(&hi2c3, ACC_RANGE, GYRO_RANGE) < 0)
{
    while (1)
    {
        /* Do something when initialization failed ... */
        Error_Handler();
    }
}
```
ACC_RANGE: `ACC_RANGE_2G`, `ACC_RANGE_4G`, `ACC_RANGE_8G`, `ACC_RANGE_16G`

GYRO_RANGE: `GYRO_RANGE_250DPS`, `GYRO_RANGE_500DPS` `GYRO_RANGE_1000DPS`, `GYRO_RANGE_2000DPS`

* Update the sensor readings

    Prototype: `int MPU_I2C_Read(IMU_t *pimu)`
```c
IMU_t IMU; // Declare IMU Data Structure

int IsUpdated = MPU_I2C_Read(&IMU);
```
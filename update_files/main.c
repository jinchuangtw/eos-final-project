/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stdio.h"
#include "stdlib.h"
#include "STM32_MPU9250.h"
#include "AHRS.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct DataPackage
{
    double qua[4];
    int status;
} Data_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_PORT GPIOD
#define LED_GREEN GPIO_PIN_12
#define LED_ORANGE GPIO_PIN_13
#define LED_RED GPIO_PIN_14
#define LED_BLUE GPIO_PIN_15

#define BUTTON_PORT GPIOA
#define BUTTON GPIO_PIN_0

#define AHRS_TIME_MS 10 // Delay 10 milliseconds for 100 Hz sample frequency
#define UART_TIME_MS AHRS_TIME_MS

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#define vector_norm(x) sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2])
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
IMU_t IMU;
Data_t UART_Tx;
UART_HandleTypeDef *phuart = &huart2;
SemaphoreHandle_t xSemaphore_Mutex_UART = NULL;

double eul[3], qua[4];
int situation = 0;
_Bool alarmStatus = 0;

// 偵測撞擊
double acc_norm = 0.0;
double acc_norm_thr = 100;

// 偵測睡著(移動平均)
double sroll_inc = 0.0, sroll_pre = 0.0, spitch_inc = 0.0, spitch_pre = 0.0,
        new_pose_inc = 0.0, sum_pose = 0.0, avg_sum_pose = 0.0;
double pose_thr = 0.8 * DEG_TO_RAD;

// 偵測姿態 0.5 秒內改變
double roll_thr = 60.0 * DEG_TO_RAD;
double pitch_thr = 60.0 * DEG_TO_RAD;
double roll_inc = 0.0, roll_inc_pre = 0.0, pitch_inc = 0.0, pitch_inc_pre = 0.0;

// 立flag立起來
_Bool Sleep;
_Bool Impact;
_Bool RocknRoll;
_Bool Nothing;
TickType_t Sleep_time, Impact_time, RocknRoll_time; // timestamps

enum Down // 沒用到
{
    Down_mode_1, Down_mode_2
};

void is_sleep(double avg_sum_pose);
void reset_flag();

/* printf function prototype */
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(phuart, (uint8_t*) (&ch), 1, 10);
    return ch;
}

/* AHRS Task */
void vTask_AHRS(void *pvPara)
{
    /* Local Variable Declaration */
    TickType_t previous_time = xTaskGetTickCount();
    TickType_t time_increment = pdMS_TO_TICKS(AHRS_TIME_MS);
    int task_counter = 0;
    _Bool AHRS_IsUpdated;

    while (1)
    {
        vTaskDelayUntil(&previous_time, time_increment);

        /* Put the MAIN TASK here and execute it repeatedly. */
        /* MPU9250 - Read Raw Data */
        if (MPU_I2C_Read(&IMU) == 1)
        {
            AHRS_IsUpdated = 1;

            /* Calibrating the Acc. and Mag. Measruements */
            ARHS_IMU_Calibration(IMU.acc, IMU.mag, IMU.acc, IMU.mag);

            /* Update AHRS Estimates */
            AHRS_Update(IMU.gyro, IMU.acc, IMU.mag);
            AHRS_Get_Euler(eul);
            AHRS_Get_Quat(qua);
        }
        else
        {
            AHRS_IsUpdated = 0;
        }

        /* Display Task Status by LED Control */
        if (task_counter++ == 50) // TODO:: Hard code
        {
            task_counter = 0;

            if (AHRS_IsUpdated == 1)
            {
                HAL_GPIO_TogglePin(LED_PORT, LED_GREEN);
                HAL_GPIO_WritePin(LED_PORT, LED_RED, 0);
            }
            else
            {
                HAL_GPIO_WritePin(LED_PORT, LED_GREEN, 0);
                HAL_GPIO_TogglePin(LED_PORT, LED_RED);
            }
        }
        /* End of MAIN TASK */
    }
}

void vTask_UART(void *pvPara)
{
    TickType_t previous_time = xTaskGetTickCount();
    TickType_t time_increment = pdMS_TO_TICKS(UART_TIME_MS);
    int task_counter = 0;

    const uint8_t start_byte = 's';
    const uint8_t finish_byte = 'f';

    while (1)
    {
        vTaskDelayUntil(&previous_time, time_increment);

        /* Take Semaphore */
        xSemaphoreTake(xSemaphore_Mutex_UART, portMAX_DELAY);

        /* Copy Data  */
        UART_Tx.status = 5;
        memcpy(UART_Tx.qua, qua, sizeof(qua));

        /* Transmit the Binary Data by UART */
        HAL_UART_Transmit(phuart, (uint8_t*) (&start_byte), 1, 10);
        HAL_UART_Transmit(phuart, (uint8_t*) (&UART_Tx), sizeof(Data_t), HAL_MAX_DELAY);
        HAL_UART_Transmit(phuart, (uint8_t*) (&finish_byte), 1, 10);

//        printf("Sleep: %d \t Impact: %d \t RocknRoll: %d \t alarmStatus: %d \r\n ",
//                Sleep,Impact,RocknRoll,alarmStatus);

//        for (int i = 0; i < 4; i++)
//        {
//            printf("%.2f ", qua[i]);
//        }
//        printf("\r\n");

        /* Give Mutex Semaphore */
        xSemaphoreGive(xSemaphore_Mutex_UART);

        if (task_counter++ == 50) // TODO::Hard code
        {
            task_counter = 0;
            HAL_GPIO_TogglePin(LED_PORT, LED_BLUE);
        }
    }
}

/* Fall Detection */
void vTask_Fall(void *pvPara)
{
    // 迴圈執行時間
    TickType_t task_delay_time_ticks = pdMS_TO_TICKS(10); // 10 ms = 0.01 sec = 100 Hz
    TickType_t previous_time = xTaskGetTickCount();
    TickType_t time_increment = task_delay_time_ticks;

    // 偵測姿態(增量)
    TickType_t Tick_Current;
    TickType_t Tick_pose_period = pdMS_TO_TICKS(500); // 0.5 sec
    TickType_t Tick_pose_pre = xTaskGetTickCount();

    // 偵測睡著(平均增量)
    int task_counter_sleep = 0;
    int sleep_duration = 8000 / task_delay_time_ticks; // 8 sec
    double pose_inc[sleep_duration]; //紀錄每個 tick roll 跟 pitch 的改變量

    while (1)
    {
        vTaskDelayUntil(&previous_time, time_increment); //確保每隔這段時間會執行一次此task

        Tick_Current = xTaskGetTickCount();
        // 偵測姿態(增量)
        if (Tick_Current - Tick_pose_pre >= Tick_pose_period) // if 經過0.5 sec
        {
            Tick_pose_pre = xTaskGetTickCount();

            /* Calculate Pose Increment */
            roll_inc = eul[0] - roll_inc_pre;
            roll_inc_pre = eul[0];
            pitch_inc = eul[1] - pitch_inc_pre;
            pitch_inc_pre = eul[1];
        }

        if (fabs(roll_inc) > roll_thr || fabs(pitch_inc) > pitch_thr)
        {
        	Impact=0;//確保撞擊會在劇烈角度改變之後
            RocknRoll = 1; // 0.5秒內 pitch and roll 有大幅度改變
            alarmStatus=0;
            RocknRoll_time = xTaskGetTickCount();
            task_counter_sleep = 0; // counter 歸零
            Sleep=0;
        }

        // 偵測睡著(移動平均增量)
        sroll_inc = eul[0] - sroll_pre; // 現在相對於 0.01 sec 前的增量
        sroll_pre = eul[0];
        spitch_inc = eul[1] - spitch_pre;
        spitch_pre = eul[1];
        new_pose_inc = (fabs(sroll_inc) + fabs(spitch_inc)) / 2.0;

        if (task_counter_sleep < sleep_duration) // 小於8秒
        {
            pose_inc[task_counter_sleep++] = new_pose_inc;
        }
        else if (task_counter_sleep == sleep_duration) // reach 8 sec
        {
            task_counter_sleep++; // ask: 這動作可以改成 每進一次 while 就 ++ 嗎

            HAL_GPIO_TogglePin(LED_PORT, LED_ORANGE);
            pose_inc[sleep_duration] = new_pose_inc;

            for (int i = 0; i < sleep_duration; i++)
            {
                sum_pose = sum_pose + pose_inc[i];
            }

            avg_sum_pose = sum_pose / sleep_duration; // 過去 800 筆 increments ㄉ平均
            is_sleep(avg_sum_pose);
            if (Sleep)
            {
                Sleep_time = xTaskGetTickCount();
            }
            sum_pose = 0;
        }
        else
        {
            //shift
            for (int i = 0; i < sleep_duration - 1; i++)
            {
                pose_inc[i] = pose_inc[i + 1]; // 把最舊一筆擠掉
            }
            pose_inc[sleep_duration] = new_pose_inc; // 把最新的塞到尾八八八阿我先好我覺得註解可以不用山~好好笑 好

            for (int i = 0; i < sleep_duration; i++)
            {
                sum_pose = sum_pose + pose_inc[i]; // 算出最近 800 筆資料的 summation
            }
            avg_sum_pose = sum_pose / sleep_duration;
            is_sleep(avg_sum_pose);
            if (Sleep)
            {
                Sleep_time = xTaskGetTickCount();
            }
            sum_pose = 0;
        }

//      // 偵測睡著(平均增量)
//      if (task_counter_sleep++ == sleep_duration)
//      {
//          HAL_GPIO_TogglePin(LED_PORT, LED_ORANGE);
//          task_counter_sleep = 0;
//
//          if (total_inc / sleep_duration < pose_thr_deg)
//          {
//              situation = Sleep;
//          }
//          else
//          {
//              situation = Dynamic;
//          }
//          total_inc = 0;
//      }

        // 偵測撞擊
        acc_norm = vector_norm(IMU.acc);

        if (acc_norm > acc_norm_thr)
        {
        	Sleep=0;//撞擊之後偵測睡著
            Impact = 1;
            alarmStatus=0;
            Impact_time = xTaskGetTickCount();
            task_counter_sleep = 0;
        }

        // TODO alarm (TODO 會變色欸)
    	if(Sleep && Impact) // 倒地狀況1: 撞擊後靜止
		{//撞擊16秒後還在sleep
            // 除以 portTICK_PERIOD_MS 好像形同於 pdMS_TO_TICKS() 反正就是把 ms 轉成 ticks
    		if (fabs(Sleep_time - Impact_time) > 16000 / portTICK_PERIOD_MS)
        	{
    			alarmStatus = 1;
    			reset_flag();
    			task_counter_sleep=0;
        		printf("hiiiii");
        	}
        	else
        	{
        		alarmStatus = 0;
        	}

        }
        else if (RocknRoll && Impact) // 倒地狀況2: 角度大幅變化後撞擊
        {
            // TODO 底下這個你斯可 以繼續打我用別的電腦好其實我已經好挖屋，可以可以你忙完再來 我先 休息 玩耍XDDDDDD好低♥笑死竟然還記得號碼我只記得這個忘記ㄌ掰鋪拍餔ㄆ88
            if (fabs(Impact_time - RocknRoll_time) < 2000 / portTICK_PERIOD_MS)
            {
                alarmStatus = 1;
                reset_flag();
    			task_counter_sleep=0;
        		printf("ohhhhhh");
            }
            else
            {
            	alarmStatus =0;
            }
        }


        static int tc;
        if (tc ++ == 50) // TODO:: HARD CODE
        {
            tc = 0;
            HAL_GPIO_TogglePin(LED_PORT, LED_ORANGE);
        }
    }
}

void is_sleep(double avg_sum_pose)
{
    if (avg_sum_pose < pose_thr)
    {
        Sleep = 1;
    }
    else
    {
    	printf("sleeeeepppppppppppppp");
    	Sleep = 0;
    }
}

void reset_flag()
{
	Sleep=0;
	RocknRoll=0;
	Impact=0;
}


/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C3_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */
    /* MPU9250 - I2C Initialization */
    if (MPU_I2C_Init(&hi2c3, ACC_RANGE_16G, GYRO_RANGE_2000DPS) < 0) // Set full scale reading range of accelerometer and gyroscope
    {
        while (1)
        {
            HAL_GPIO_TogglePin(LED_PORT, LED_RED);
            HAL_Delay(500);
        }
    }

    /* AHRS - Initialization */
    /* Set AHRS parameters */
    double fc[4] =
            { 0.25, 0.25, 0.25, 0.25 }; // Cut-off frequency
    double T = (double) (AHRS_TIME_MS) / 1000.0; // Sample time interval
    AHRS_Init(fc, T);

    double s_acc[3] =
            { 1.0, 1.0, 1.0 };
    double b_acc[3] =
            { 0.0, 0.0, 0.0 };
    double s_mag[3] =
            { 1.0, 1.0, 1.0 };
    double b_mag[3] =
            { 0.0, 0.0, 0.0 };
    AHRS_Set_IMU_Cal_Para(s_acc, b_acc, s_mag, b_mag);

    /* Estimate the initial orientation from average gravitational and magnetic fields */
    const int NUM_OF_SAMPLES = 100;
    double acc_f[3] =
            { 0.0, 0.0, 0.0 };
    double mag_f[3] =
            { 0.0, 0.0, 0.0 };
    for (int i = 0; i < NUM_OF_SAMPLES; i++)
    {
        MPU_I2C_Read(&IMU);
        HAL_Delay(AHRS_TIME_MS);

        for (int i = 0; i < 3; i++)
        {
            acc_f[i] += IMU.acc[i];
            mag_f[i] += IMU.mag[i];
        }
    }
    for (int i = 0; i < 3; i++)
    {
        acc_f[i] /= (double) (NUM_OF_SAMPLES);
        mag_f[i] /= (double) (NUM_OF_SAMPLES);
    }
    ARHS_IMU_Calibration(acc_f, mag_f, acc_f, mag_f);
    AHRS_Estimate_IC(acc_f, mag_f);

    /* Initialize Data Package */
    memset((void *) &UART_Tx, 0, sizeof(Data_t));

    /* Create Mutex Semaphore to protect the Hardware source of UART */
    xSemaphore_Mutex_UART = xSemaphoreCreateMutex(); // Create Mutex Semaphore

    /* Create Tasks */
    xTaskCreate(vTask_AHRS, "AHRS", 1000, NULL, 4, NULL);
    xTaskCreate(vTask_Fall, "Fall", 2000, NULL, 3, NULL);
    xTaskCreate(vTask_UART, "UART", 1000, NULL, 2, NULL);

    /* Begin Task Scheduling */
    vTaskStartScheduler();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct =
            { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct =
            { 0 };

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 192;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 8;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void)
{

    /* USER CODE BEGIN I2C3_Init 0 */

    /* USER CODE END I2C3_Init 0 */

    /* USER CODE BEGIN I2C3_Init 1 */

    /* USER CODE END I2C3_Init 1 */
    hi2c3.Instance = I2C3;
    hi2c3.Init.ClockSpeed = 100000;
    hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c3.Init.OwnAddress1 = 0;
    hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c3.Init.OwnAddress2 = 0;
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c3) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C3_Init 2 */

    /* USER CODE END I2C3_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct =
            { 0 };

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin
            | Audio_RST_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : PE2 */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pin : CS_I2C_SPI_Pin */
    GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PE4 PE5 MEMS_INT2_Pin */
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | MEMS_INT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
    GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PDM_OUT_Pin */
    GPIO_InitStruct.Pin = PDM_OUT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PA0 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : I2S3_WS_Pin */
    GPIO_InitStruct.Pin = I2S3_WS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
    GPIO_InitStruct.Pin = SPI1_SCK_Pin | SPI1_MISO_Pin | SPI1_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : CLK_IN_Pin PB12 */
    GPIO_InitStruct.Pin = CLK_IN_Pin | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
     Audio_RST_Pin */
    GPIO_InitStruct.Pin = LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin
            | Audio_RST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
    GPIO_InitStruct.Pin = I2S3_MCK_Pin | I2S3_SCK_Pin | I2S3_SD_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : VBUS_FS_Pin */
    GPIO_InitStruct.Pin = VBUS_FS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
    GPIO_InitStruct.Pin = OTG_FS_ID_Pin | OTG_FS_DM_Pin | OTG_FS_DP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
    GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
    GPIO_InitStruct.Pin = Audio_SCL_Pin | Audio_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM3 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM3)
    {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
    * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include <stdio.h>
#include <stdarg.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define THREAD_STACK_SIZE 1024
#define LTR390_UVSMode 0x02
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t sensor_thread_stack[THREAD_STACK_SIZE];
uint8_t storage_thread_stack[THREAD_STACK_SIZE];

TX_THREAD sensor_thread;
TX_THREAD storage_thread;
TX_QUEUE sensor_data_queue;

char RTCBuffer[100];
extern I2C_HandleTypeDef h12c1;
extern UART_HandleTypeDef huart1;
extern FX_FILE file;


float lux_index_global = 0.0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

VOID sensor_thread_entry(ULONG initial_input);
VOID storage_thread_entry(ULONG initial_input);

void UART_Printf(const char *fmt, ...);
float calculate_lux(uint8_t *buffer);
void write_lux_to_file(const char *data);

/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;

  /* USER CODE BEGIN App_ThreadX_MEM_POOL */
  static ULONG queue_buffer[32];

  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */
  tx_thread_create(&sensor_thread, "Sensor Thread", sensor_thread_entry, 0,
                       sensor_thread_stack, THREAD_STACK_SIZE, 10, 10, TX_NO_TIME_SLICE, TX_AUTO_START);

  tx_thread_create(&storage_thread, "Storage Thread", storage_thread_entry, 0,
                       storage_thread_stack, THREAD_STACK_SIZE, 12, 12, TX_NO_TIME_SLICE, TX_AUTO_START);

  tx_queue_create(&sensor_data_queue, "Sensor Data Queue", TX_1_ULONG,
                      queue_buffer, sizeof(queue_buffer));
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

  /**
  * @brief  Function that implements the kernel's initialization.
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN Before_Kernel_Start */

  /* USER CODE END Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN Kernel_Start_Error */

  /* USER CODE END Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */

VOID sensor_thread_entry(ULONG initial_input)
{
    uint8_t buffer[4] = {0};
    uint8_t buf[1];
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

    while (1)
    {
        UART_Printf("Reading ALS data...\r\n");
        LTR390_ReadRegister(LTR390UV_INPUTREG_ALS_DATA_LOW, buffer, 4);

        float lux = calculate_lux(buffer);

        HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
        HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

        snprintf(RTCBuffer, sizeof(RTCBuffer), "Time:%02d:%02d:%02d\r\n",
                 sTime.Hours, sTime.Minutes, sTime.Seconds);

        HAL_UART_Transmit(&huart1, (uint8_t *)RTCBuffer, strlen(RTCBuffer), HAL_MAX_DELAY);

        tx_queue_send(&sensor_data_queue, &lux, TX_NO_WAIT); // Send data to queue
        tx_thread_sleep(10);                                // Cycle every 10 ms
    }
}

VOID storage_thread_entry(ULONG initial_input)
{
    float lux;

    while (1)
    {
        if (tx_queue_receive(&sensor_data_queue, &lux, TX_WAIT_FOREVER) == TX_SUCCESS)
        {
            char buffer[50];
            snprintf(buffer, sizeof(buffer), "LUX data: %f\r\n", lux);
            write_lux_to_file(buffer); // Write to SD card
        }
    }
}

float calculate_lux(uint8_t *buffer)
{
    uint8_t a_gain[5] = {1, 3, 6, 9, 18};
    double a_int[6] = {4.0, 2.0, 1.0, 0.5, 0.25, 0.03125}; // lux
    uint8_t gain = LTR390_GAIN3;                           // lux
    uint8_t resolution = (LTR390_RES18_BIT & 0xf0) >> 4;   // lux
    uint32_t originalData;

    uint8_t size = 4;
    for (uint8_t i = 0; i < size;)
    {
        uint8_t temp = buffer[i];
        buffer[i] = buffer[i + 1];
        buffer[i + 1] = temp;
        i += 2;
    }

    originalData = (uint32_t)buffer[2] << 24 | (uint32_t)buffer[3] << 16 |
                   (uint16_t)buffer[0] << 8 | buffer[1];
    float lux = (0.6 * originalData) / (a_gain[gain] * a_int[resolution]);
    return lux;
}

void write_lux_to_file(const char *data)
{
    UINT status;

    fx_file_seek(&file, 0, FX_SEEK_END);

    status = fx_file_write(&file, data, strlen(data));

    if (status != FX_SUCCESS)
    {
        UART_Printf("Failed to save data %u\n", status);
    }

    fx_file_flush(&file);
}

void UART_Printf(const char *fmt, ...)
{
    char buffer[100];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/* USER CODE END 1 */

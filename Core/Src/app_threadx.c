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
#include <app_filex.h>
//#include <adc.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
#define THREAD_STACK_SIZE 1024
#define QUEUE_SIZE 32;

//Threads variables
TX_THREAD uv_sensor_thread;
TX_THREAD accelerometr_thread;
TX_THREAD sd_storage_thread;
TX_QUEUE uv_data_queue;
TX_QUEUE accel_data_queue;


uint8_t uv_thread_stack[THREAD_STACK_SIZE];
uint8_t accel_thread_stack[THREAD_STACK_SIZE];
uint8_t sd_thread_stack[THREAD_STACK_SIZE];

static ULONG uv_queue_buffer[QUEUE_SIZE];
static ULONG accel_queue_buffer[QUEUE_SIZE];

extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc1;
extern FX_FILE file;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

VOID uv_sensor_thread_entry(ULONG initial_input);
VOID accelerometer_thread_entry(ULONG initial_input);
VOID sd_storage_thread_entry(ULONG initial_input);

void UART_Printf(const char *fmt, ...);
void write_data_to_file(const char *data);

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
  	tx_queue_create(&uv_data_queue, "UV Data Queue", TX_1_ULONG, uv_queue_buffer, sizeof(uv_queue_buffer));
    tx_queue_create(&accel_data_queue, "Accelerometer Queue", TX_1_ULONG, accel_queue_buffer, sizeof(accel_queue_buffer));


  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */
  tx_thread_create(
		  &uv_sensor_thread,
		  "UV Thread",
		  uv_sensor_thread_entry,0,
		  uv_thread_stack,
		  THREAD_STACK_SIZE,
		  10, 10,
		  TX_NO_TIME_SLICE,
		  TX_AUTO_START);

  tx_thread_create(
		  &accelerometer_thread,
		  "Accelerometer Thread",
		  accelerometer_thread_entry,0,
		  accel_thread_stack,
		  THREAD_STACK_SIZE,
		  10, 10,
		  TX_NO_TIME_SLICE,
		  TX_AUTO_START);

  tx_thread_create(
		  &sd_storage_thread,
		  "SD Thread",
		  sd_storage_thread_entry,0,
		  sd_thread_stack,
		  THREAD_STACK_SIZE,
		  5,5,
		  TX_NO_TIME_SLICE,
		  TX_AUTO_START);

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

HAL_StatusTypeDef LTR390_WriteRegister(uint8_t reg, uint8_t value) {
    uint8_t data[3] = {reg + 5, value, 0};
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, LTR390UV_DEVICE_ADDR, data, 3, HAL_MAX_DELAY);

    if (status != HAL_OK) {
        UART_Printf("I2C Write Error: reg=0x%X, value=0x%X, status=%d\r\n", reg, value, status);
    } else {
        UART_Printf("I2C Write Success: reg=0x%X, value=0x%X\r\n", reg, value);
    }
    return status;
}

HAL_StatusTypeDef LTR390_ReadRegister(uint8_t reg, uint8_t *data, uint8_t length) {
    HAL_StatusTypeDef status;

    // Wysłanie adresu rejestru
    status = HAL_I2C_Master_Transmit(&hi2c1, LTR390UV_DEVICE_ADDR, &reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        UART_Printf("I2C TX Error: %d\r\n", status);
        return status;
    }

    // Odczyt danych z rejestru
    status = HAL_I2C_Master_Receive(&hi2c1, LTR390UV_DEVICE_ADDR, data, length, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        UART_Printf("I2C RX Error: %d\r\n", status);
    }

    return status;
}

void LTR390_Init() {

    // Reset czujnika przed konfiguracją
    LTR390_WriteRegister(0x00, 0x00);
    HAL_Delay(10);
    LTR390_WriteRegister(0x00, 0x01);
    HAL_Delay(10);
    LTR390_WriteRegister(0x00, 0x00);
    HAL_Delay(10);

    //Sprawdzenie ID czujnika
    uint8_t sensor_id = 0;
    if (LTR390_ReadRegister(0x06, &sensor_id, 1) == HAL_OK) {
        UART_Printf("LTR390UV sensor responded ID: 0x%X\r\n", sensor_id);
        if (sensor_id == 0xB2) {
            UART_Printf("LTR390UV detected correctly\n");
        } else {
            UART_Printf("LTR390UV returned wrong ID: 0x%X\r\n", sensor_id);
        }
    } else {
        UART_Printf("LTR390UV sensor communication error\r\n");
    }

    // KONFIGURACJA CZUJNIKA
    LTR390_WriteRegister(0x00, 0x0A);  // Włączenie w trybie UV
    HAL_Delay(10);
    LTR390_WriteRegister(0x04, 0x06);  // 20-bit, 800ms integracji (dla stabilności)
    LTR390_WriteRegister(0x05, 0x03);  // Gain 9x
    HAL_Delay(10);

    // Wymuszenie startu pomiaru + ustawienie aktywnego trybu
    LTR390_WriteRegister(0x00, 0x0B);  // Tryb UV + aktywacja
    HAL_Delay(100);

    // Sprawdzenie, czy konfiguracja zapisała się
    uint8_t gain_value = 0;
    LTR390_ReadRegister(0x05, &gain_value, 1);
    UART_Printf("Odczytany GAIN: 0x%X\r\n", gain_value);

    HAL_Delay(500);
}

uint32_t LTR390_ReadUV() {
    uint8_t raw_data[3] = {0};
    LTR390_ReadRegister(0x10, raw_data, 3);

    return ((uint32_t)raw_data[2] << 16) | ((uint32_t)raw_data[1] << 8) | raw_data[0];
}

void I2C_Scan() {
    UART_Printf("Skanuję I2C...\n");
    for (uint8_t addr = 1; addr < 127; addr++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 3, HAL_MAX_DELAY) == HAL_OK) {
            UART_Printf("Znaleziono urządzenie na adresie: 0x%X\n", addr);
        }
    }
}

float convertToUVI(uint32_t uv_data) {
    float uv_sensitivity = 2300.0;  // Wartość zależna od specyfikacji czujnika
    return (float)uv_data / uv_sensitivity;
}

uint8_t LTR390_ReadStatus() {
	    uint8_t status = 0;
	    LTR390_ReadRegister(0x07, &status, 1);
//	    UART_Printf("Status: 0x%X\r\n", status);  // DEBUG
	    return status;
	}

void Start_UV_Measurement() {
    UART_Printf("Wymuszanie startu pomiaru UV...\r\n");
    LTR390_WriteRegister(0x00, 0x0A);  // Tryb UV
    HAL_Delay(10);
    LTR390_WriteRegister(0x00, 0x02);  // Start pomiaru
    HAL_Delay(100);

    // Sprawdzenie statusu czy czujnik gotowy
    uint8_t status = 0;
    for (int i = 0; i < 10; i++) {
        LTR390_ReadRegister(0x07, &status, 1);
        UART_Printf("Status: 0x%X\r\n", status);
        if (status & 0x08) {  // Bit 3 == DATA_READY
            UART_Printf("Dane gotowe do odczytu!\r\n");
            return;
        }
        HAL_Delay(100);
    }
    UART_Printf("Błąd: czujnik nie przechodzi do trybu pomiaru!\r\n");
}

VOID uv_sensor_thread_entry(ULONG inital_input){

	uint32_t buffer[4] = {0};

	//reset and put into uv mode
	LTR390_Init();



	while(1){

		LTR390_ReadRegister(0x10, buffer, 4);
		float uv = convertToUVI(buffer);

		tx_queue_send(&uv_data_queue, &uv, TX_NO_WAIT);

		UART_Printf("UV Data sent: %.2f lux\n", uv);
		tx_thread_sleep(100);

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

/* Accelerometer Thread */
VOID accelerometer_thread_entry(ULONG initial_input)
{
    uint32_t adc_value_z = 0, adc_value_xy = 0;
    float acceleration_z, acceleration_xy;
    while (1)
    {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        adc_value_z = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);

        acceleration_z = (3.3 * adc_value_z / 4095.0) - 1.65;

        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        adc_value_xy = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);

        acceleration_xy = (3.3 * adc_value_xy / 4095.0) - 1.65;

        tx_queue_send(&accel_data_queue, &acceleration_z, TX_NO_WAIT);
        tx_queue_send(&accel_data_queue, &acceleration_xy, TX_NO_WAIT);
        UART_Printf("Acceleration Z: %.2f g, Acceleration XY: %.2f g\n", acceleration_z, acceleration_xy);
        tx_thread_sleep(100);
    }
}

/* SD Storage Thread */
VOID sd_storage_thread_entry(ULONG initial_input)
{
    float lux, acceleration_z, acceleration_xy;
    while (1)
    {
        if (tx_queue_receive(&uv_data_queue, &lux, TX_WAIT_FOREVER) == TX_SUCCESS)
        {
            char buffer[50];
            snprintf(buffer, sizeof(buffer), "UV: %.2f lux\n", lux);
            write_data_to_file(buffer);
        }
        if (tx_queue_receive(&accel_data_queue, &acceleration_z, TX_WAIT_FOREVER) == TX_SUCCESS &&
            tx_queue_receive(&accel_data_queue, &acceleration_xy, TX_WAIT_FOREVER) == TX_SUCCESS)
        {
            char buffer[50];
            snprintf(buffer, sizeof(buffer), "Accel Z: %.2f g, XY: %.2f g\n", acceleration_z, acceleration_xy);
            write_data_to_file(buffer);
        }
    }
}



//float calculate_lux(uint8_t *buffer)
//{
//    uint8_t a_gain[5] = {1, 3, 6, 9, 18};
//    double a_int[6] = {4.0, 2.0, 1.0, 0.5, 0.25, 0.03125}; // lux
//    uint8_t gain = LTR390_GAIN3;                           // lux
//    uint8_t resolution = (LTR390_RES18_BIT & 0xf0) >> 4;   // lux
//    uint32_t originalData;
//
//    uint8_t size = 4;
//    for (uint8_t i = 0; i < size;)
//    {
//        uint8_t temp = buffer[i];
//        buffer[i] = buffer[i + 1];
//        buffer[i + 1] = temp;
//        i += 2;
//    }
//
//    originalData = (uint32_t)buffer[2] << 24 | (uint32_t)buffer[3] << 16 |
//                   (uint16_t)buffer[0] << 8 | buffer[1];
//    float lux = (0.6 * originalData) / (a_gain[gain] * a_int[resolution]);
//    return lux;
//}

//void write_lux_to_file(const char *data)
//{
//    UINT status;
//
//    fx_file_seek(&file, 0, FX_SEEK_END);
//
//    status = fx_file_write(&file, data, strlen(data));
//
//    if (status != FX_SUCCESS)
//    {
//        UART_Printf("Failed to save data %u\n", status);
//    }
//
//    fx_file_flush(&file);
//}

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

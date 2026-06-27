/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "control_loop.h"
#include "pwm_adc.h"
#include "usbd_cdc_if.h"

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for controlLoopTask */
const osThreadAttr_t controlLoopTask_attributes = {
  .name = "ctrlLoop",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  controlLoopTaskHandle = osThreadNew(ControlLoopTask, NULL, &controlLoopTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  const uint8_t boot_msg[] = "YDrive boot\r\n";
  char msg[192];

  HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_RESET);
  osDelay(1000);
  CDC_Transmit_FS((uint8_t *)boot_msg, sizeof(boot_msg) - 1U);
  pwm_adc_init();
  pwm_adc_start_timing();

  /* Infinite loop */
  for(;;)
  {
    GPIO_PinState nfault_state = HAL_GPIO_ReadPin(nFAULT_GPIO_Port, nFAULT_Pin);
    static uint32_t last_adc_count = 0;
    static uint32_t last_current_count = 0;
    PwmAdcStatus pwm_adc_status;
    ControlLoopStatus control_status;

    HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_RESET);
    pwm_adc_get_status(&pwm_adc_status);
    control_loop_get_status(&control_status);

    uint32_t adc_hz = pwm_adc_status.adc_irq_count - last_adc_count;
    uint32_t current_hz = pwm_adc_status.current_meas_count - last_current_count;
    last_adc_count = pwm_adc_status.adc_irq_count;
    last_current_count = pwm_adc_status.current_meas_count;

    int len = snprintf(msg, sizeof(msg),
                       "nFAULT=%u adc_irq=%lu current=%lu dc_cal=%lu dir=%u axis=%lu cal=%u vbus=%lu ib=%lu ic=%lu off_b=%ld off_c=%ld ia=%ld ibc=%ld icc=%ld\r\n",
                       (nfault_state == GPIO_PIN_SET) ? 1U : 0U,
                       (unsigned long)adc_hz,
                       (unsigned long)current_hz,
                       (unsigned long)pwm_adc_status.dc_cal_count,
                       pwm_adc_status.counting_down,
                       (unsigned long)control_status.loop_count,
                       pwm_adc_status.offset_calibrated,
                       (unsigned long)pwm_adc_status.vbus_raw,
                       (unsigned long)pwm_adc_status.ib_raw,
                       (unsigned long)pwm_adc_status.ic_raw,
                       (long)pwm_adc_status.ib_offset,
                       (long)pwm_adc_status.ic_offset,
                       (long)control_status.latest_ia,
                       (long)control_status.latest_ib,
                       (long)control_status.latest_ic);

    if (len > 0) {
      CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
    }

    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

